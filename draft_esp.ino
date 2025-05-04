#include <Arduino.h>

#define IN1 22           // Motor IN1 pin
#define IN2 21           // Motor IN2 pin
#define ENA 32           // Motor Enable pin for speed control
#define RECEIVE_PIN 4    // Pin D4 to receive signal from Arduino
#define RELAY_PIN 23     // Relay control pin

int motorSpeed = 200;
bool prevTrackerSensorState = HIGH;

void setup() {
  pinMode(RECEIVE_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Initial states
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(RELAY_PIN, LOW);  // Relay off initially
  
  Serial.begin(9600);
  Serial.println("ESP32 Ready");
}

void loop() {
  int trackerSensorState = digitalRead(RECEIVE_PIN);

  Serial.print("Tracker Sensor State: ");
  Serial.println(trackerSensorState);

  // If an object is detected, activate relay and move motor clockwise
  if (trackerSensorState == HIGH && prevTrackerSensorState == LOW) {
    Serial.println("Object detected! Turning on relay and moving motor clockwise.");

    // Turn on relay
    digitalWrite(RELAY_PIN, HIGH);

    // Motor moves clockwise
    analogWrite(ENA, motorSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    delay(2000);

    // Stop motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    delay(1000);
  }

  // If the object moves away, deactivate relay and move motor anticlockwise
  if (trackerSensorState == LOW && prevTrackerSensorState == HIGH) {
    Serial.println("Object moved away! Turning off relay and moving motor anticlockwise.");

    // Turn off relay
    digitalWrite(RELAY_PIN, LOW);

    // Motor moves anticlockwise
    analogWrite(ENA, motorSpeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    delay(2000);

    // Stop motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    delay(1000);
  }

  // Update previous tracker sensor state for edge detection
  prevTrackerSensorState = trackerSensorState;
  delay(100);
}
