#!/usr/bin/env python3
"""
SeptiSense laptop backend  (Python standard library + numpy/pandas/scikit-learn)

  ESP32  --Wi-Fi/HTTP POST-->  this server  -->  SQLite  -->  ML inference  -->  dashboard

Run:   python server.py                 (real time: 1 model-bin = 1 hour)
       python server.py --demo-fast     (DEMO ONLY: 1 model-bin = 60 seconds)

Dashboard: http://localhost:8000
"""
import argparse
import json
import socket
import sqlite3
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

BASE = Path(__file__).parent
STATIC = BASE / "static"
DB_PATH = BASE / "septisense.db"
MODEL_PATH = BASE / "models" / "sepsis_model.joblib"
METRICS_PATH = BASE / "models" / "metrics.json"

BIN_SECONDS = 3600
INFER_EVERY_S = 10
MIN_OBSERVED_BINS = 4          # of the last WINDOW bins
LOOKBACK_BINS = 24

DEFAULT_CONFIG = {
    "alert_enabled": 1,
    "alert_threshold": 0.5,        # replaced by the model's threshold when available
    "consecutive_required": 2,     # predictions in a row above threshold
    "cooldown_seconds": 600,
    "temp_offset_c": 0.0,          # surface -> core correction (see README!)
    "offline_seconds": 45,
}
CONFIG_LIMITS = {
    "alert_enabled": (0, 1), "alert_threshold": (0.001, 0.999),
    "consecutive_required": (1, 10), "cooldown_seconds": (0, 86400),
    "temp_offset_c": (-5.0, 10.0), "offline_seconds": (10, 600),
}

MODEL = None            # loaded bundle
MODEL_ERROR = None
STATE = {"streak": {}, "last_alert": {}, "offline_flag": {}}
STATE_LOCK = threading.Lock()


# ----------------------------------------------------------------- database
def db():
    c = sqlite3.connect(DB_PATH, timeout=10)
    c.row_factory = sqlite3.Row
    return c


def init_db():
    with db() as c:
        c.execute("PRAGMA journal_mode=WAL")
        c.executescript("""
        CREATE TABLE IF NOT EXISTS telemetry(
          id INTEGER PRIMARY KEY AUTOINCREMENT, device_id TEXT, ts REAL, received REAL,
          source TEXT, mlx_ok INT, obj REAL, amb REAL, led INT,
          max_ok INT, finger INT, hr REAL, spo2 REAL,
          ecg_bpm REAL, ecg_q INT, lead_off INT);
        CREATE INDEX IF NOT EXISTS ix_tel ON telemetry(device_id, ts);
        CREATE TABLE IF NOT EXISTS predictions(
          id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL, device_id TEXT, status TEXT,
          reason TEXT, risk REAL, level TEXT, inputs TEXT, bins_used INT, bin_seconds INT);
        CREATE TABLE IF NOT EXISTS alerts(
          id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL, device_id TEXT, type TEXT,
          level TEXT, message TEXT, risk REAL, acknowledged INT DEFAULT 0);
        CREATE TABLE IF NOT EXISTS config(key TEXT PRIMARY KEY, value REAL);
        """)


def get_config():
    cfg = dict(DEFAULT_CONFIG)
    if MODEL is not None:
        cfg["alert_threshold"] = round(float(MODEL["threshold"]), 4)
    with db() as c:
        for r in c.execute("SELECT key,value FROM config"):
            if r["key"] in cfg:
                cfg[r["key"]] = r["value"]
    return cfg


def num_or_none(v, lo, hi):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    if v != v or v < lo or v > hi:
        return None
    return v


# ---------------------------------------------------------------- telemetry
def ingest(payload):
    device = str(payload.get("device_id", "unknown"))[:40]
    now = time.time()
    sent_ms = payload.get("sent_ms")
    rows = []
    for r in payload.get("records", []):
        # ESP32 has no real-time clock: reconstruct sample time from the
        # device's millisecond counter relative to the moment it was sent.
        try:
            age = (float(sent_ms) - float(r["t"])) / 1000.0
        except (TypeError, ValueError, KeyError):
            age = 0.0
        age = min(max(age, 0.0), 6 * 3600)
        ts = now - age
        hr = num_or_none(r.get("hr"), 30, 220) if r.get("hrOK") else None
        spo2 = num_or_none(r.get("spo2"), 70, 100) if r.get("spo2OK") else None
        ecg = num_or_none(r.get("ecg"), 30, 220) if r.get("ecgOK") else None
        obj = num_or_none(r.get("obj"), -20, 80) if r.get("tOK") else None
        amb = num_or_none(r.get("amb"), -20, 80) if r.get("tOK") else None
        rows.append((device, ts, now, "simulator" if device.startswith("sim") else "device",
                     int(bool(r.get("mlx"))), obj, amb, int(bool(r.get("led"))),
                     int(bool(r.get("max"))), int(bool(r.get("fin"))), hr, spo2,
                     ecg, int(r.get("ecgQ", -1)), int(bool(r.get("lo")))))
    with db() as c:
        c.executemany("""INSERT INTO telemetry(device_id,ts,received,source,mlx_ok,obj,amb,led,
                         max_ok,finger,hr,spo2,ecg_bpm,ecg_q,lead_off)
                         VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""", rows)
    return len(rows)


# ---------------------------------------------------------------- inference
def load_model():
    global MODEL, MODEL_ERROR
    try:
        import joblib
        MODEL = joblib.load(MODEL_PATH)
        MODEL_ERROR = None
        print(f"Model loaded: {MODEL_PATH.name} (alert threshold {MODEL['threshold']:.3f})")
    except FileNotFoundError:
        MODEL_ERROR = "No trained model found. Run train_model.py first."
        print("WARNING:", MODEL_ERROR)
    except Exception as e:                       # noqa
        MODEL_ERROR = f"Model failed to load: {e}"
        print("WARNING:", MODEL_ERROR)


def predict_device(device_id, now, cfg):
    """Returns a dict describing the prediction (or why none is possible)."""
    if MODEL is None:
        return {"status": "no_model", "reason": MODEL_ERROR or "model not loaded"}
    import numpy as np
    import pandas as pd
    from features import FILL_LIMIT, VITALS, WINDOW, add_features

    since = now - LOOKBACK_BINS * BIN_SECONDS
    with db() as c:
        rows = c.execute("""SELECT ts,hr,ecg_bpm,spo2,obj FROM telemetry
                            WHERE device_id=? AND ts>=? ORDER BY ts""",
                         (device_id, since)).fetchall()
    if not rows:
        return {"status": "insufficient", "reason": "no recent telemetry"}

    bins = {}
    for r in rows:
        b = int(r["ts"] // BIN_SECONDS)
        d = bins.setdefault(b, {v: [] for v in VITALS})
        hr = r["hr"] if r["hr"] is not None else r["ecg_bpm"]
        if hr is not None:
            d["HR"].append(hr)
        if r["spo2"] is not None:
            d["O2Sat"].append(r["spo2"])
        if r["obj"] is not None:
            d["Temp"].append(r["obj"] + cfg["temp_offset_c"])
    lo, hi = min(bins), max(bins)
    idx = list(range(lo, hi + 1))
    df = pd.DataFrame({v: [float(np.mean(bins[b][v])) if b in bins and bins[b][v] else np.nan
                           for b in idx] for v in VITALS}, index=idx)

    tail = df.tail(WINDOW)
    if (tail["HR"].notna().sum() < MIN_OBSERVED_BINS):
        return {"status": "insufficient",
                "reason": f"need heart-rate data in at least {MIN_OBSERVED_BINS} of the last "
                          f"{WINDOW} bins ({BIN_SECONDS}s each); have {int(tail['HR'].notna().sum())}"}
    for v in VITALS:                       # staleness check: observed in last 2 bins
        if df[v].tail(2).notna().sum() == 0:
            return {"status": "insufficient", "reason": f"no recent valid {v} reading"}

    dff = df.ffill(limit=FILL_LIMIT).reset_index(drop=True)
    dff["pid"] = "live"
    feats = add_features(dff)
    x = feats.iloc[[-1]].astype("float32")
    from features import REQUIRED_NON_NULL
    if x[REQUIRED_NON_NULL].isna().any(axis=None):
        return {"status": "insufficient", "reason": "not enough history for rolling features"}
    raw = float(MODEL["model"].predict_proba(x[MODEL["features"]])[:, 1][0])
    risk = float(MODEL["calibrator"].predict([raw])[0])
    thr = cfg["alert_threshold"]
    level = "HIGH" if risk >= thr else ("ELEVATED" if risk >= 0.5 * thr else "LOW")
    last = dff.iloc[-1]
    return {"status": "ok", "risk": risk, "level": level,
            "bins_used": int(tail["HR"].notna().sum()),
            "inputs": {"HR": round(float(last["HR"]), 1), "O2Sat": round(float(last["O2Sat"]), 1),
                       "Temp_model_input": round(float(last["Temp"]), 2)}}


def raise_alert(c, now, device, typ, level, msg, risk=None):
    c.execute("INSERT INTO alerts(ts,device_id,type,level,message,risk) VALUES(?,?,?,?,?,?)",
              (now, device, typ, level, msg, risk))
    print(f"[ALERT] {typ} {level}: {msg}")


def inference_cycle():
    now = time.time()
    cfg = get_config()
    with db() as c:
        devices = [r["device_id"] for r in c.execute(
            "SELECT DISTINCT device_id FROM telemetry WHERE ts>=?", (now - 24 * 3600,))]
        for dev in devices:
            last = c.execute("SELECT MAX(received) m FROM telemetry WHERE device_id=?",
                             (dev,)).fetchone()["m"]
            # ---- connectivity alert
            offline = (now - last) > cfg["offline_seconds"]
            with STATE_LOCK:
                was = STATE["offline_flag"].get(dev, False)
                STATE["offline_flag"][dev] = offline
            if offline and not was and cfg["alert_enabled"]:
                raise_alert(c, now, dev, "DEVICE_OFFLINE", "WARNING",
                            f"No telemetry from {dev} for over {int(cfg['offline_seconds'])} s.")
            if offline:
                continue
            # ---- prediction
            res = predict_device(dev, now, cfg)
            c.execute("""INSERT INTO predictions(ts,device_id,status,reason,risk,level,inputs,
                         bins_used,bin_seconds) VALUES(?,?,?,?,?,?,?,?,?)""",
                      (now, dev, res["status"], res.get("reason"), res.get("risk"),
                       res.get("level"), json.dumps(res.get("inputs")),
                       res.get("bins_used"), BIN_SECONDS))
            if res["status"] != "ok":
                continue
            with STATE_LOCK:
                streak = STATE["streak"].get(dev, 0)
                streak = streak + 1 if res["risk"] >= cfg["alert_threshold"] else 0
                STATE["streak"][dev] = streak
                last_alert = STATE["last_alert"].get(dev, 0)
                fire = (cfg["alert_enabled"] and streak >= cfg["consecutive_required"]
                        and now - last_alert >= cfg["cooldown_seconds"])
                if fire:
                    STATE["last_alert"][dev] = now
            if fire:
                raise_alert(c, now, dev, "RISK", "HIGH",
                            f"Estimated risk {res['risk']:.1%} is at/above the alert threshold "
                            f"{cfg['alert_threshold']:.1%} (6 h horizon, experimental).", res["risk"])
        c.commit()


def inference_loop():
    while True:
        try:
            inference_cycle()
        except Exception:
            traceback.print_exc()
        time.sleep(INFER_EVERY_S)


# ---------------------------------------------------------------------- API
def status_payload():
    now = time.time()
    cfg = get_config()
    out = {"server_time": now, "bin_seconds": BIN_SECONDS,
           "demo_time_compression": BIN_SECONDS < 3600,
           "model_loaded": MODEL is not None, "model_error": MODEL_ERROR,
           "horizon_hours": MODEL["meta"]["horizon_hours"] if MODEL else 6,
           "config": cfg, "devices": []}
    with db() as c:
        for d in c.execute("SELECT DISTINCT device_id FROM telemetry WHERE ts>=?", (now - 86400,)):
            dev = d["device_id"]
            last = c.execute("SELECT * FROM telemetry WHERE device_id=? ORDER BY ts DESC LIMIT 1",
                             (dev,)).fetchone()
            recv = c.execute("SELECT MAX(received) m FROM telemetry WHERE device_id=?",
                             (dev,)).fetchone()["m"]
            w = c.execute("""SELECT COUNT(*) n, COUNT(hr) hr, COUNT(spo2) spo2, COUNT(obj) obj,
                             COUNT(ecg_bpm) ecg, MAX(CASE WHEN hr IS NOT NULL THEN ts END) hr_ts,
                             MAX(CASE WHEN spo2 IS NOT NULL THEN ts END) spo2_ts,
                             MAX(CASE WHEN obj IS NOT NULL THEN ts END) obj_ts,
                             MAX(CASE WHEN ecg_bpm IS NOT NULL THEN ts END) ecg_ts
                             FROM telemetry WHERE device_id=? AND ts>=?""",
                          (dev, now - 300)).fetchone()
            n = max(w["n"], 1)
            def q(valid, ts):
                return {"valid_pct": round(100.0 * valid / n, 1),
                        "last_valid_s_ago": round(now - ts, 1) if ts else None}
            out["devices"].append({
                "device_id": dev, "source": last["source"],
                "last_seen_s_ago": round(now - recv, 1),
                "connected": (now - recv) <= cfg["offline_seconds"],
                "samples_5min": w["n"],
                "sensors": {
                    "temperature (MLX90614)": {"hardware_ok": bool(last["mlx_ok"]), **q(w["obj"], w["obj_ts"])},
                    "pulse ox HR (MAX30102)": {"hardware_ok": bool(last["max_ok"]), **q(w["hr"], w["hr_ts"])},
                    "SpO2 (MAX30102)": {"hardware_ok": bool(last["max_ok"]), **q(w["spo2"], w["spo2_ts"])},
                    "ECG HR (AD8232)": {"hardware_ok": not bool(last["lead_off"]), **q(w["ecg"], w["ecg_ts"])},
                },
                "finger_present": bool(last["finger"]),
            })
    return out


def latest_payload(device=None):
    with db() as c:
        if device:
            r = c.execute("SELECT * FROM telemetry WHERE device_id=? ORDER BY ts DESC LIMIT 1", (device,)).fetchone()
        else:
            r = c.execute("SELECT * FROM telemetry ORDER BY ts DESC LIMIT 1").fetchone()
        if not r:
            return {"telemetry": None, "prediction": None}
        p = c.execute("SELECT * FROM predictions WHERE device_id=? ORDER BY ts DESC LIMIT 1",
                      (r["device_id"],)).fetchone()
    tel = dict(r)
    pred = None
    if p:
        pred = dict(p)
        pred["inputs"] = json.loads(pred["inputs"]) if pred["inputs"] else None
    return {"telemetry": tel, "prediction": pred}


def history_payload(minutes, device=None):
    now = time.time()
    with db() as c:
        dev = device or (c.execute("SELECT device_id FROM telemetry ORDER BY ts DESC LIMIT 1").fetchone() or [None])[0]
        rows = c.execute("""SELECT ts,hr,spo2,obj,ecg_bpm FROM telemetry
                            WHERE device_id=? AND ts>=? ORDER BY ts""",
                         (dev, now - minutes * 60)).fetchall()
    step = max(1, len(rows) // 800)
    return {"device_id": dev, "points": [dict(r) for r in rows[::step]]}


def rows_payload(table, limit, order="ts DESC"):
    with db() as c:
        rows = [dict(r) for r in c.execute(f"SELECT * FROM {table} ORDER BY {order} LIMIT ?", (limit,))]
    if table == "predictions":
        for r in rows:
            r["inputs"] = json.loads(r["inputs"]) if r["inputs"] else None
    return rows


class Handler(BaseHTTPRequestHandler):
    server_version = "SeptiSense/1.0"

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _json_body(self):
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(n) or b"{}")

    def do_GET(self):
        u = urlparse(self.path)
        qs = parse_qs(u.query)
        try:
            if u.path in ("/", "/index.html"):
                return self._send(200, (STATIC / "index.html").read_bytes(), "text/html; charset=utf-8")
            if u.path == "/api/status":
                return self._send(200, status_payload())
            if u.path == "/api/latest":
                return self._send(200, latest_payload(qs.get("device", [None])[0]))
            if u.path == "/api/history":
                return self._send(200, history_payload(float(qs.get("minutes", ["30"])[0]), qs.get("device", [None])[0]))
            if u.path == "/api/predictions":
                return self._send(200, rows_payload("predictions", int(qs.get("limit", ["200"])[0])))
            if u.path == "/api/alerts":
                return self._send(200, rows_payload("alerts", int(qs.get("limit", ["50"])[0])))
            if u.path == "/api/config":
                return self._send(200, get_config())
            if u.path == "/api/model":
                info = {"loaded": MODEL is not None, "error": MODEL_ERROR}
                if METRICS_PATH.exists():
                    info["metrics"] = json.loads(METRICS_PATH.read_text())
                return self._send(200, info)
            return self._send(404, {"error": "not found"})
        except Exception as e:                   # noqa
            traceback.print_exc()
            return self._send(500, {"error": str(e)})

    def do_POST(self):
        u = urlparse(self.path)
        try:
            body = self._json_body()
            if u.path == "/api/telemetry":
                return self._send(200, {"stored": ingest(body)})
            if u.path == "/api/config":
                with db() as c:
                    for k, v in body.items():
                        if k in CONFIG_LIMITS:
                            lo, hi = CONFIG_LIMITS[k]
                            v = min(max(float(v), lo), hi)
                            c.execute("INSERT INTO config(key,value) VALUES(?,?) "
                                      "ON CONFLICT(key) DO UPDATE SET value=excluded.value", (k, v))
                return self._send(200, get_config())
            if u.path == "/api/alerts/ack":
                with db() as c:
                    c.execute("UPDATE alerts SET acknowledged=1 WHERE id=?", (int(body.get("id", -1)),))
                return self._send(200, {"ok": True})
            return self._send(404, {"error": "not found"})
        except Exception as e:                   # noqa
            traceback.print_exc()
            return self._send(400, {"error": str(e)})


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:                            # noqa
        return "unknown"


def main():
    global BIN_SECONDS
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--bin-seconds", type=int, default=3600,
                    help="seconds per model time-bin (training used 3600 = 1 hour)")
    ap.add_argument("--demo-fast", action="store_true",
                    help="DEMO ONLY: 60-second bins so predictions appear within minutes")
    args = ap.parse_args()
    BIN_SECONDS = 60 if args.demo_fast else args.bin_seconds

    init_db()
    load_model()
    threading.Thread(target=inference_loop, daemon=True).start()
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print("=" * 60)
    print(f" SeptiSense backend running  (model bin = {BIN_SECONDS} s)")
    if BIN_SECONDS < 3600:
        print(" !! DEMO TIME COMPRESSION ACTIVE - predictions are NOT clinically meaningful !!")
    print(f" Dashboard:        http://localhost:{args.port}")
    print(f" ESP32 should POST to: http://{lan_ip()}:{args.port}/api/telemetry")
    print("=" * 60)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("Stopped.")


if __name__ == "__main__":
    main()
