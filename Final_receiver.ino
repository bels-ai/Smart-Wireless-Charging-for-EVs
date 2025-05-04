#include <Arduino.h>
#include <MFRC522.h>
#include <SPI.h>
#include <LiquidCrystal.h>

// LCD pin assignments
const int rs = 7, en = 8, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// RFID settings
#define SS_PIN 10
#define RST_PIN 9
MFRC522 mfrc522(SS_PIN, RST_PIN);

// RFID UIDs
byte powerOnUID[4] = { 0xAE, 0xEE, 0x70, 0xD5 };  // UID for Power On
byte powerOffUID[4] = { 0x65, 0x9D, 0x40, 0xE6 }; // UID for Power Off

// Transmitter and induction pins
#define TRANSMIT_PIN A5         // Pin to control power on/off
#define INDUCTION_PIN A3        // Pin to detect charging status

// Battery sensor pin and voltage scaling factor
const int voltagePin = A0;               // Voltage sensor input
const float scalingFactor = 25.0 / 1023.0;  // Scale factor for MAX 25V sensor on 10-bit ADC

// Battery parameters
const float maxVoltage = 12.0;
const float midVoltage = 8.0;
const float minVoltage = 4.0;

// Kalman filter parameters
const float Q = 0.001;
const float R = 0.01;
float P = 1.0;
float K;
float SoC = 1.0;
float estimatedVoltage = maxVoltage;

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);

  // Initialize Transmitter pin
  pinMode(TRANSMIT_PIN, OUTPUT);
  digitalWrite(TRANSMIT_PIN, HIGH);  // Start with transmitter off

  // Initialize Induction pin
  pinMode(INDUCTION_PIN, INPUT);

  // Initialize RFID
  SPI.begin();
  mfrc522.PCD_Init();
}

void loop() {
  // **Section 1: Check Induction (Charging) Status**
  if (digitalRead(INDUCTION_PIN) == HIGH) {
    lcd.clear();
    lcd.print("System Power On");
    lcd.setCursor(0, 1);
    lcd.print("EV Charging");
    delay(2000);
  } else {
    lcd.clear();
    lcd.print("System Power Off");
    lcd.setCursor(0, 1);
    lcd.print("EV Disconnected");
    delay(2000);
  }

  // **Section 2: RFID Tag Reading for Power On/Off**
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    if (checkUID(mfrc522.uid.uidByte, powerOnUID)) {
      if (digitalRead(INDUCTION_PIN) == LOW) {
        lcd.clear();
        lcd.print("Powering On...");
        digitalWrite(TRANSMIT_PIN, LOW);  // Activate the transmitter
        delay(2000);

        // Confirm power on
        if (digitalRead(INDUCTION_PIN) == HIGH) {
          lcd.clear();
          lcd.print("System Power On");
          lcd.setCursor(0, 1);
          lcd.print("EV Charging");
        } else {
          lcd.clear();
          lcd.print("Failed to Power On");
          delay(2000);
        }
      }
    } else if (checkUID(mfrc522.uid.uidByte, powerOffUID)) {
      if (digitalRead(INDUCTION_PIN) == HIGH) {
        lcd.clear();
        lcd.print("Powering Off...");
        digitalWrite(TRANSMIT_PIN, HIGH);  // Deactivate the transmitter
        delay(2000);

        // Confirm power off
        if (digitalRead(INDUCTION_PIN) == LOW) {
          lcd.clear();
          lcd.print("System Power Off");
          lcd.setCursor(0, 1);
          lcd.print("EV Disconnected");
        } else {
          lcd.clear();
          lcd.print("Failed to Power Off");
          delay(2000);
        }
      }
    }
    mfrc522.PICC_HaltA();
  }

  // **Section 3: Battery Voltage and SoC Calculation**
  int sensorValue = analogRead(voltagePin);
  float measuredVoltage = sensorValue * scalingFactor;
  SoC = EKF_Update(measuredVoltage);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Voltage: ");
  lcd.print(measuredVoltage);
  lcd.print(" V");

  lcd.setCursor(0, 1);
  lcd.print("SoC: ");
  lcd.print(SoC * 100.0);  // Display SoC as percentage
  lcd.print(" %");

  delay(2000);  // Delay for readability

  // **Fault Detection**
  checkFaults(measuredVoltage);
}

// Function to check if UID matches the tag
bool checkUID(byte *readUID, byte *targetUID) {
  for (int i = 0; i < 4; i++) {
    if (readUID[i] != targetUID[i]) {
      return false;
    }
  }
  return true;
}

// Extended Kalman Filter (EKF) Update Function for SoC estimation
float EKF_Update(float measuredVoltage) {
  estimatedVoltage = minVoltage + (maxVoltage - minVoltage) * SoC;
  P = P + Q;

  K = P / (P + R);

  SoC = SoC + K * (measuredVoltage - estimatedVoltage);
  P = (1 - K) * P;

  SoC = calculateSoC(measuredVoltage);

  if (SoC > 1.0) SoC = 1.0;
  if (SoC < 0.0) SoC = 0.0;

  return SoC;
}

// Function to calculate SoC based on voltage readings
float calculateSoC(float voltage) {
  if (voltage >= maxVoltage) return 1.0;
  else if (voltage <= minVoltage) return 0.0;
  else return (voltage - minVoltage) / (maxVoltage - minVoltage);
}

// Function to check for overvoltage and undervoltage faults
void checkFaults(float voltage) {
  const float overVoltageLimit = 15.0;
  const float underVoltageLimit = 4.0;

  if (voltage > overVoltageLimit) {
    Serial.println("Fault: Overvoltage detected!");
  } else if (voltage < underVoltageLimit) {
    Serial.println("Fault: Undervoltage detected!");
  } else {
    Serial.println("Battery status: Normal");
  }
}
