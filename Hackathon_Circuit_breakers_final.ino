#include <Servo.h>

// ========== PIN CONNECTIONS ==========

// Door and motion
const int pingPin = 2;
const int pirPin = 3;
const int servoPin = 4;

// Main outputs
const int bedroomLightPin = 6;
const int fanPin = 7;
const int buzzerPin = 8;
const int motionLedPin = 9;

// Three ambience LEDs: PWM pins
const int ambienceLED1 = 5;
const int ambienceLED2 = 10;
const int ambienceLED3 = 11;

// Analog sensors
const int ldrPin = A0;
const int tempPin = A1;
const int waterPin = A2;
const int gasPin = A3;
const int forcePin = A4;


// ========== THRESHOLDS ==========

const int lightThreshold = 500;
const int waterThreshold = 500;
const int gasThreshold = 600;
const int forceThreshold = 500;

const float fanOnTemperature = 30.0;
const float fanOffTemperature = 27.0;


// ========== TIME SETTINGS ==========

const unsigned long pressureTime = 1500;
const unsigned long clearTime = 1000;
const unsigned long blinkInterval = 180;
const unsigned long bedroomFadeInterval = 20;

const unsigned long ambienceFadeTime = 3000;
const unsigned long ambienceCycleTime = 6000;


// ========== VARIABLES ==========

Servo doorServo;

bool roomOccupied = false;
bool fanOn = false;
bool sleepMode = false;
bool pressureTiming = false;
bool emergencyBlink = false;
bool approachArmed = true;
bool previousSleepMode = false;

unsigned long pressureStartTime = 0;
unsigned long clearStartTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastBedroomFadeTime = 0;
unsigned long sleepStartTime = 0;

int bedroomBrightness = 0;


// ========== SETUP ==========

void setup() {
  pinMode(pirPin, INPUT);

  pinMode(bedroomLightPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(motionLedPin, OUTPUT);

  pinMode(ambienceLED1, OUTPUT);
  pinMode(ambienceLED2, OUTPUT);
  pinMode(ambienceLED3, OUTPUT);

  doorServo.attach(servoPin);
  doorServo.write(0);

  Serial.begin(9600);
}


// ========== MAIN LOOP ==========

void loop() {
  unsigned long now = millis();


  // 1. AUTOMATIC DOOR USING PING))) SENSOR

  pinMode(pingPin, OUTPUT);

  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);

  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);

  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);

  long duration = pulseIn(pingPin, HIGH, 30000);
  int distance = duration * 0.034 / 2;

  bool personNear = (duration > 0 && distance < 20);

  if (personNear) {
    doorServo.write(90);
  } else {
    doorServo.write(0);
  }


  // 2. ROOM OCCUPANCY - OPTION A
  // Each new approach toggles the occupied state.

  if (personNear && approachArmed) {
    roomOccupied = !roomOccupied;
    approachArmed = false;
  }

  if (!personNear) {
    if (clearStartTime == 0) {
      clearStartTime = now;
    }

    if (now - clearStartTime >= clearTime) {
      approachArmed = true;
    }
  } else {
    clearStartTime = 0;
  }


  // 3. READ LDR

  int lightLevel = analogRead(ldrPin);

  // Assumes wiring:
  // 5V -> LDR -> A0 -> 10k resistor -> GND

  bool isDark = (lightLevel < lightThreshold);


  // 4. READ TEMPERATURE

  int tempReading = analogRead(tempPin);

  // TMP36 temperature conversion
  float voltage = tempReading * (5.0 / 1023.0);
  float temperatureC = (voltage - 0.5) * 100.0;


  // 5. FAN CONTROL
  // Fan requires room occupancy AND high temperature.

  if (!roomOccupied) {
    fanOn = false;
  } else {
    if (temperatureC > fanOnTemperature) {
      fanOn = true;
    }

    if (temperatureC < fanOffTemperature) {
      fanOn = false;
    }
  }

  digitalWrite(fanPin, fanOn ? HIGH : LOW);


  // 6. FORCE SENSOR - INDEPENDENT SLEEP MODE
  // Does NOT depend on roomOccupied or the door.

  int forceReading = analogRead(forcePin);

  // Assumes pressure increases the reading.
  bool bedPressed = (forceReading > forceThreshold);

  if (bedPressed) {

    if (!pressureTiming) {
      pressureStartTime = now;
      pressureTiming = true;
    }

    // Require continuous pressure for 1.5 seconds.
    if (now - pressureStartTime >= pressureTime) {
      sleepMode = true;
    }

  } else {
    pressureTiming = false;
    sleepMode = false;
  }


  // Start a fresh ambience cycle when Sleep Mode begins.

  if (sleepMode && !previousSleepMode) {
    sleepStartTime = now;
  }

  previousSleepMode = sleepMode;


  // 7. MOTION DETECTION

  bool motionDetected = (digitalRead(pirPin) == HIGH);


  // 8. WATER DETECTION

  int waterReading = analogRead(waterPin);

  // Assumes higher readings mean water is detected.
  bool waterDetected = (waterReading > waterThreshold);


  // 9. GAS DETECTION

  int gasReading = analogRead(gasPin);

  // Assumes higher readings mean gas is detected.
  bool gasDetected = (gasReading > gasThreshold);


  // 10. BEDROOM LIGHT CONTROL
  // Door approach OR darkness turns the light on.
  // Sleep Mode dims it.

  int targetBrightness;

  if (sleepMode) {
    targetBrightness = 51;  // About 20% brightness
  } else {
    targetBrightness = (personNear || isDark) ? 255 : 0;
  }

  if (now - lastBedroomFadeTime >= bedroomFadeInterval) {
    lastBedroomFadeTime = now;

    if (bedroomBrightness < targetBrightness) {
      bedroomBrightness += 5;

      if (bedroomBrightness > targetBrightness) {
        bedroomBrightness = targetBrightness;
      }

    } else if (bedroomBrightness > targetBrightness) {
      bedroomBrightness -= 5;

      if (bedroomBrightness < targetBrightness) {
        bedroomBrightness = targetBrightness;
      }
    }
  }


  // 11. AMBIENCE LEDS
  // Fade in for 3 seconds, then fade out for 3 seconds.
  // Controlled ONLY by the force sensor's Sleep Mode state.

  int ambienceBrightness = 0;

  if (sleepMode && !gasDetected) {

    unsigned long elapsed = now - sleepStartTime;
    unsigned long phase = elapsed % ambienceCycleTime;

    if (phase < ambienceFadeTime) {

      // Fade in
      ambienceBrightness =
          (phase * 255UL) / ambienceFadeTime;

    } else {

      // Fade out
      unsigned long fadeOutTime = phase - ambienceFadeTime;

      ambienceBrightness =
          255 - (fadeOutTime * 255UL) / ambienceFadeTime;
    }
  }

  analogWrite(ambienceLED1, ambienceBrightness);
  analogWrite(ambienceLED2, ambienceBrightness);
  analogWrite(ambienceLED3, ambienceBrightness);


  // 12. GAS EMERGENCY OVERRIDE
  // D6 and D9 flash alternately.

  if (gasDetected) {

    if (now - lastBlinkTime >= blinkInterval) {
      lastBlinkTime = now;
      emergencyBlink = !emergencyBlink;
    }

    digitalWrite(bedroomLightPin,
                 emergencyBlink ? HIGH : LOW);

    digitalWrite(motionLedPin,
                 emergencyBlink ? LOW : HIGH);

    // Emergency turns ambience LEDs off.
    analogWrite(ambienceLED1, 0);
    analogWrite(ambienceLED2, 0);
    analogWrite(ambienceLED3, 0);

  } else {

    // Restore normal bedroom-light brightness.
    analogWrite(bedroomLightPin, bedroomBrightness);

    // Motion-warning LED
    digitalWrite(motionLedPin,
                 motionDetected ? HIGH : LOW);

    emergencyBlink = false;
  }


  // 13. SHARED BUZZER
  // Gas, water, or motion activates the buzzer.

  if (gasDetected || waterDetected || motionDetected) {
    tone(buzzerPin, 1000);
  } else {
    noTone(buzzerPin);
  }


  // 14. SERIAL MONITOR

  Serial.print("Distance: ");
  Serial.print(distance);

  Serial.print(" cm | Occupied: ");
  Serial.print(roomOccupied ? "YES" : "NO");

  Serial.print(" | Fan: ");
  Serial.print(fanOn ? "ON" : "OFF");

  Serial.print(" | Light: ");
  Serial.print(lightLevel);

  Serial.print(" | Temperature: ");
  Serial.print(temperatureC);

  Serial.print(" C | Water: ");
  Serial.print(waterReading);

  Serial.print(" | Gas: ");
  Serial.print(gasReading);

  Serial.print(" | Force: ");
  Serial.print(forceReading);

  Serial.print(" | Sleep: ");
  Serial.println(sleepMode ? "ON" : "OFF");

  delay(20);
}