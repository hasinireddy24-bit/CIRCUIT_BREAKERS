
const int trigPin = 3;
const int echoPin = 2;

const int pin8 = 8;
const int pin10 = 10;
const int pin6 = 6;

long duration;
int Waterlevel;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(pin8, OUTPUT);
  pinMode(pin10, OUTPUT);
  pinMode(pin6, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // Measure distance using ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    Serial.println("Sensor not detecting distance");

    digitalWrite(pin8, LOW);
    digitalWrite(pin10, LOW);
    digitalWrite(pin6, LOW);
  } else {
    Waterlevel = duration * 0.034 / 2;

    Serial.print("Water distance: ");
    Serial.print(Waterlevel);
    Serial.println(" cm");

    if (Waterlevel > 200) {
      // Low water condition
      digitalWrite(pin8, HIGH);
      digitalWrite(pin10, LOW);
      digitalWrite(pin6, LOW);
    }
    else if (Waterlevel < 100) {
      // Water is closer to the sensor
      digitalWrite(pin8, HIGH);
      digitalWrite(pin10, HIGH);
      digitalWrite(pin6, HIGH);
    }
    else {
      // Medium water condition
      digitalWrite(pin8, LOW);
      digitalWrite(pin10, LOW);
      digitalWrite(pin6, LOW);
    }
  }

  delay(500);
}
