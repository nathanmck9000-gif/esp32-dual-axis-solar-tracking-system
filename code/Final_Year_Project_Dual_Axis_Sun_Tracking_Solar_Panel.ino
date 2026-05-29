/*
Final Year Project for BEng(Hons) in Sustainable Energy and Environmental Engineering.

This code takes in power output data for both a dual-axis sun-tracking solar panel and
a static solar panel simultaneously. 4x LDRs are used to measure the direction of highest
light intensity, and 2x servo motors are used to move the dual-axis solar panel. Data is
transmitted via Bluetooth communication.

Features:
  - Dual-axis sun tracking
  - ESP32 control
  - LDR sensing
  - Servo motor positioning
  - Bluetooth telemetry
  - Power monitoring

Written by Nathan McKee on 02/Apr/2026
*/ 

#include <ESP32Servo.h>        // For Servos
#include <Wire.h>              // Enables I2C Communication
#include <Adafruit_INA219.h>   // For Current Sensors
#include "BluetoothSerial.h"   // For Bluetooth Communication

// Define two INA219 Current Sensors with different addresses
Adafruit_INA219 ina219_1(0x40); // Static Panel (Black Wires)
Adafruit_INA219 ina219_2(0x41); // Tracked Panel (Blue Wires)

// Define Bluetooth
BluetoothSerial SerialBT;

// Analog pins for LDRs
const int ldrTR = 34;
const int ldrBL = 35;
const int ldrBR = 32;
const int ldrTL = 33;

// Variables for average LDR values so that they can be printed for debugging
float topAvg = 0;
float bottomAvg = 0;
float leftAvg = 0;
float rightAvg = 0;

// Variables to hold these values (later printed for debugging)
float diff_LR = 0;
float diff_UD = 0;

// Servo pins
const int servoXPin = 19; // Left & Right
const int servoYPin = 18; // Up & Down

// Define Servos
Servo servoX;
Servo servoY;

// Initial servo angles
int angleX = 90;
int angleY = 90;

// Tuning parameters
const int threshold = 100;  // Sensitivity threshold
const int stepSize  = 1;    // How much to move per adjustment
const int minAngleX = 0;    // Facing towards ServoX wires
const int maxAngleX = 180;  // Facing away from ServoX wires
const int minAngleY = 40;   // Facing downwards (angled)
const int maxAngleY = 150;  // Facing upwards (directly)

// Time intervals for "void loop()"
const unsigned long SERVO_INTERVAL = 250;   // ms
const unsigned long LOG_INTERVAL   = 1000;  // ms
unsigned long lastServoUpdate = 0;
unsigned long lastLogTime = 0;

// Needed for headers to consistently print
bool headerSent = false;

// Values that determine if servos are safe to attach
bool servosEnabled = false;
unsigned long servoEnableTime = 0;

// LDR Values
int TL_val = 0;
int TR_val = 0;
int BL_val = 0;
int BR_val = 0;

void setup() {

  Serial.begin(115200);

  // Bluetooth Connection Troubleshoot
  if (!SerialBT.begin("ESP32_Solar")) {
    Serial.println("Bluetooth failed to start!");
  } else {
    Serial.println("Bluetooth started. Device is discoverable.");
  }

  // Let ESP32 connect to PuTTY
  delay(2000);

  // Initialise INA219 Current Sensors + Debugging Messages
  if (!ina219_1.begin()) {
  Serial.println("INA219_1 not found");
  }
  if (!ina219_2.begin()) {
  Serial.println("INA219_2 not found");
  }

  // Calibrate Current Sensors
  ina219_1.setCalibration_16V_400mA();
  ina219_2.setCalibration_16V_400mA();

  // Keep servo pins OFF until they're ready (filters noise)
  pinMode(servoXPin, OUTPUT);
  pinMode(servoYPin, OUTPUT);
  digitalWrite(servoXPin, LOW);
  digitalWrite(servoYPin, LOW);

  // Servos are enabled 3 seconds after they attach
  servoEnableTime = millis() + 3000;
}

void loop() {
  unsigned long current_time = millis();
  
  // IF the servos are NOT enabled yet
  // AND the current time is later than the safety time
  // THEN do the servo setup
  if (!servosEnabled && current_time > servoEnableTime) {
  servoY.attach(servoYPin);
  delay(500);
  servoX.attach(servoXPin);

  // Initialise servo angles
  // Delays to avoid current spikes
  servoY.write(angleY);
  delay(250);
  servoX.write(angleX);
  delay(250);
  
  // servos are now active so don’t run this part again
  servosEnabled = true;
  }
  
  // Table Headings Print Only When Connected to Laptop
  if (SerialBT.hasClient() && !headerSent) {
    SerialBT.println("Measuring voltage and current with INA219 ...");
    SerialBT.println(",Static Panel,,,,,Tracked Panel");
    SerialBT.println("Time (s),Bus Voltage (V),Shunt Voltage (mV),Load Voltage (V),Current (mA),Power (mW),Bus Voltage (V),Shunt Voltage (mV),Load Voltage (V),Current (mA),Power (mW),angleX,angleY,ldrTL,ldrTR,ldrBL,ldrBR,diff_LR,diff_UD");
    headerSent = true;
  }

  // ===== SERVO CONTROL =====
  if (current_time - lastServoUpdate >= SERVO_INTERVAL) {
  lastServoUpdate = current_time;
  
  // Read LDRs
  TL_val = analogRead(ldrTL);
  TR_val = analogRead(ldrTR);
  BL_val = analogRead(ldrBL);
  BR_val = analogRead(ldrBR);

  // Calculate LDR cardinal direction averages
  topAvg = (TL_val + TR_val) / 2;
  bottomAvg = (BL_val + BR_val) / 2;
  leftAvg = (TL_val + BL_val) / 2;
  rightAvg = (TR_val + BR_val) / 2;

  // Values to print for debugging
  diff_LR = abs(leftAvg - rightAvg);
  diff_UD = abs(topAvg - bottomAvg);

  // Vertical adjustment (Servo Y)
  if (abs(topAvg - bottomAvg) > threshold) {
    if (topAvg > bottomAvg) {
      angleY = constrain(angleY + stepSize, minAngleY, maxAngleY);
    } else {
      angleY = constrain(angleY - stepSize, minAngleY, maxAngleY);
    }
    servoY.write(angleY);
  }

  // Horizontal adjustment (Servo X)
  if (abs(leftAvg - rightAvg) > threshold) {
    if (leftAvg > rightAvg) {
      angleX = constrain(angleX + stepSize, minAngleX, maxAngleX);
    } else {
      angleX = constrain(angleX - stepSize, minAngleX, maxAngleX);
    }
    servoX.write(angleX);
  }
  }
  // ===== SERVO CONTROL (END) =====

  // ===== DATA LOGGING =====
  if (current_time - lastLogTime >= LOG_INTERVAL) {
  lastLogTime = current_time;
  
  // Values for Static Panel
  float shuntvoltage_1 = ina219_1.getShuntVoltage_mV();
  float busvoltage_1 = ina219_1.getBusVoltage_V();
  float current_mA_1 = ina219_1.getCurrent_mA();
  float power_mW_1 = ina219_1.getPower_mW();
  float loadvoltage_1 = busvoltage_1 + (shuntvoltage_1 / 1000);

  // Values for Tracked Panel
  float shuntvoltage_2 = ina219_2.getShuntVoltage_mV();
  float busvoltage_2 = ina219_2.getBusVoltage_V();
  float current_mA_2 = ina219_2.getCurrent_mA();
  float power_mW_2 = ina219_2.getPower_mW();
  float loadvoltage_2 = busvoltage_2 + (shuntvoltage_2 / 1000);

  float time_s = current_time / 1000;

  // Smooth out noise so graphs look cleaner (Keep an eye on this)
  if (abs(current_mA_1) < 0.2) current_mA_1 = 0;
  if (busvoltage_1 < 0.01) busvoltage_1 = 0;

  if (abs(current_mA_2) < 0.2) current_mA_2 = 0;
  if (busvoltage_2 < 0.01) busvoltage_2 = 0;
  
  // Print Values to PuTTY
  SerialBT.printf(
    "%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%.3f,%.3f\r\n",
    time_s,
    busvoltage_1,
    shuntvoltage_1,
    loadvoltage_1,
    current_mA_1,
    power_mW_1,
    busvoltage_2,
    shuntvoltage_2,
    loadvoltage_2,
    current_mA_2,
    power_mW_2,
    angleX,
    angleY,
    TL_val,         // LDR Values for Debugging
    TR_val,         // LDR Values for Debugging
    BL_val,         // LDR Values for Debugging
    BR_val,         // LDR Values for Debugging
    diff_LR,        // LDR Diff Values for Debugging
    diff_UD         // LDR Diff Values for Debugging
  );
  }
    // ===== DATA LOGGING (END) =====
}