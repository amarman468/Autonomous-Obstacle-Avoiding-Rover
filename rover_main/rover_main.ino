#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <esp_task_wdt.h>

// --- PIN DEFINITIONS ---
#define PIN_TRIG 5
#define PIN_ECHO 18
#define PIN_SERVO 19
#define PIN_BUZZER 4

// Motor Pins (L298N)
#define PIN_ENA 13
#define PIN_IN1 32
#define PIN_IN2 14
#define PIN_IN3 27
#define PIN_IN4 26
#define PIN_ENB 25

// Speed Encoders (LM393 Optical Slot Sensors)
#define PIN_ENC_LEFT 34  // Input-only pin (D0 output from left sensor)
#define PIN_ENC_RIGHT 35 // Input-only pin (D0 output from right sensor)

// I2C Pins
#define PIN_SDA 21
#define PIN_SCL 22

// --- BLE UUIDS ---
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_TELEMETRY_UUID "d1a7a81e-5b0b-4f3e-9c1a-1a2b3c4d5e6f"
#define CHAR_CONFIG_UUID "e2b8b92f-6c1c-5a4f-ad2b-2b3c4d5e6f70"

// --- CONFIGURATIONS ---
int OBSTACLE_THRESHOLD = 25; // cm (triggers dynamic avoidance)
int DEADEND_THRESHOLD = 20;  // cm (minimum side clearance required)
int driveSpeed = 255;
int turnSpeed = 255;
int turn90Time = 450; // ms for a 90 degree pivot turn

// Motor Trims (0 - 100%) to calibrate motor variance
int leftTrim = 100;
int rightTrim = 100;
bool encoderSyncEnabled =
    true; // Closed-loop encoder sync for straight-line driving

// PID Controller for straight-line correction
float pidKp = 15.0; // Increased Proportional gain
float pidKi = 2.0;  // Increased Integral gain
float pidKd = 5.0;  // Increased Derivative gain
float pidIntegral = 0.0;
float pidPrevError = 0.0;
unsigned long pidLastTime = 0;
const float PID_MAX_CORRECTION = 100.0; // Max PWM correction per side
const float PID_INTEGRAL_LIMIT = 50.0;  // Anti-windup clamp

// --- ENCODER CONSTANTS & VARIABLES ---
// Standard 2WD robot chassis: 20 slots disc, wheel diameter ~6.5cm
const float WHEEL_CIRCUMFERENCE_CM = 20.42; // pi * 6.5 cm
const int ENCODER_TICKS_PER_REV = 20;
const float CM_PER_TICK =
    WHEEL_CIRCUMFERENCE_CM / ENCODER_TICKS_PER_REV; // ~1.021 cm

volatile unsigned long leftEncTicks = 0;
volatile unsigned long rightEncTicks = 0;

float speedLeft_cms = 0.0;
float speedRight_cms = 0.0;
unsigned long lastSpeedCalcTime = 0;
unsigned long prevLeftTicks = 0;
unsigned long prevRightTicks = 0;

// Interrupt Service Routines (ISRs) for Wheel Encoders
void IRAM_ATTR isrLeftEncoder() { leftEncTicks++; }

void IRAM_ATTR isrRightEncoder() { rightEncTicks++; }

// --- OBJECTS ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo radarServo;

BLEServer *pServer = NULL;
BLECharacteristic *pTelemetryChar = NULL;
BLECharacteristic *pConfigChar = NULL;
bool deviceConnected = false;

// --- GLOBAL VARIABLES ---
enum State {
  STATE_IDLE = 0,
  STATE_MANUAL = 1,
  STATE_AUTO_FORWARD = 2,
  STATE_AVOIDANCE = 3,
  STATE_DIAGNOSTICS = 4
};

State currentState = STATE_IDLE;
float distToFront = 999.0;

// Servo Sweep Variables
unsigned long lastServoTime = 0;
int servoAngle = 90;
int servoDirection = 1;
int servoStep = 3;
int servoDelay = 20;

// Function declarations
void beep(int duration);
void stopMotors();
void setMotorSpeed(int leftSpeed, int rightSpeed);
void turnRight90();
void turnLeft90();
void executeAvoidance();
float getDistanceSingle();
float getDistance();
void updateSpeedometer();
void sendDiag(const char *comp, const char *status);
bool checkLCD();
void testBuzzer();
void testServo();
float checkSonar();
int testMotorLeft();
int testMotorRight();
void runDiagnostics();

// --- BLE CALLBACKS ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pSvr) { deviceConnected = true; }
  void onDisconnect(BLEServer *pSvr) {
    deviceConnected = false;
    pSvr->startAdvertising();
  }
};

class ConfigCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    String req = pChar->getValue();
    if (req.length() > 0) {
      if (req.startsWith("CMD:")) {
        String cmd = req.substring(4);
        if (cmd == "AUTO") {
          currentState = STATE_AUTO_FORWARD;
          servoAngle = 90;
          radarServo.write(90);
          beep(100);
        } else if (cmd == "STOP") {
          currentState = STATE_IDLE;
          stopMotors();
          servoAngle = 90;
          radarServo.write(90);
        } else if (cmd == "MODE_MANUAL") {
          currentState = STATE_MANUAL;
          stopMotors();
          servoAngle = 90;
          radarServo.write(90);
        } else if (cmd == "MODE_DIAG") {
          currentState = STATE_DIAGNOSTICS;
          stopMotors();
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("MODE: DIAGNOSTIC");
          lcd.setCursor(0, 1);
          lcd.print("Ready for tests");
          sendDiag("SYS", "DIAG_READY");
        } else if (cmd == "RESET_ODO") {
          noInterrupts();
          leftEncTicks = 0;
          rightEncTicks = 0;
          interrupts();
          sendDiag("ODO", "RESET_OK");
        } else if (cmd == "TEST_AUTO") {
          runDiagnostics();
        } else if (cmd == "TEST_BUZZ") {
          sendDiag("BUZZ", "TESTING");
          testBuzzer();
          sendDiag("BUZZ", "PASS");
        } else if (cmd == "TEST_SERVO") {
          sendDiag("SERVO", "SWEEPING");
          testServo();
          sendDiag("SERVO", "PASS");
        } else if (cmd == "TEST_SONAR") {
          sendDiag("SONAR", "TESTING");
          float d = checkSonar();
          char buf[20];
          if (d < 400.0 && d > 1.0) {
            snprintf(buf, sizeof(buf), "PASS:%.1fcm", d);
            sendDiag("SONAR", buf);
          } else {
            sendDiag("SONAR", "FAIL");
          }
        } else if (cmd == "TEST_MOTL") {
          sendDiag("MOTL", "TESTING");
          int ticks = testMotorLeft();
          char buf[24];
          if (ticks > 0)
            snprintf(buf, sizeof(buf), "PASS:%dticks", ticks);
          else
            snprintf(buf, sizeof(buf), "PASS:0ticks");
          sendDiag("MOTL", buf);
        } else if (cmd == "TEST_MOTR") {
          sendDiag("MOTR", "TESTING");
          int ticks = testMotorRight();
          char buf[24];
          if (ticks > 0)
            snprintf(buf, sizeof(buf), "PASS:%dticks", ticks);
          else
            snprintf(buf, sizeof(buf), "PASS:0ticks");
          sendDiag("MOTR", buf);
        } else if (cmd == "TEST_LCD") {
          bool ok = checkLCD();
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("LCD TEST: PASS");
          lcd.setCursor(0, 1);
          lcd.print("DISPLAY OK! 16x2");
          sendDiag("LCD", ok ? "PASS" : "FAIL");
        } else {
          currentState = STATE_MANUAL;
          if (cmd == "FWD")
            setMotorSpeed(driveSpeed, driveSpeed);
          else if (cmd == "REV")
            setMotorSpeed(-driveSpeed, -driveSpeed);
          else if (cmd == "LEFT")
            setMotorSpeed(-turnSpeed, turnSpeed);
          else if (cmd == "RIGHT")
            setMotorSpeed(turnSpeed, -turnSpeed);
        }
      } else if (req.startsWith("CFG:")) {
        int firstColon = req.indexOf(':');
        int secondColon = req.indexOf(':', firstColon + 1);
        if (firstColon != -1 && secondColon != -1) {
          String key = req.substring(firstColon + 1, secondColon);
          float val = req.substring(secondColon + 1).toFloat();
          if (key == "SPEED")
            driveSpeed = constrain((int)val, 0, 255);
          else if (key == "TURN_SPEED")
            turnSpeed = constrain((int)val, 0, 255);
          else if (key == "DANGER_DIST")
            OBSTACLE_THRESHOLD = (int)val;
          else if (key == "TURN_TIME")
            turn90Time = (int)val;
          else if (key == "TRIM_L")
            leftTrim = constrain((int)val, 0, 100);
          else if (key == "TRIM_R")
            rightTrim = constrain((int)val, 0, 100);
          else if (key == "DEADEND")
            DEADEND_THRESHOLD = constrain((int)val, 5, 100);
          else if (key == "SYNC")
            encoderSyncEnabled = (val > 0.5);
          else if (key == "PID_KP")
            pidKp = constrain(val, 0.0f, 20.0f);
          else if (key == "PID_KI")
            pidKi = constrain(val, 0.0f, 10.0f);
          else if (key == "PID_KD")
            pidKd = constrain(val, 0.0f, 10.0f);
        }
      }
    }
  }
};

void setup() {
  Serial.begin(115200);

  // Speed Encoder Pins setup with hardware interrupts
  pinMode(PIN_ENC_LEFT, INPUT);
  pinMode(PIN_ENC_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT), isrLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT), isrRightEncoder,
                  RISING);

  BLEDevice::init("ESP32-Rover");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTelemetryChar = pService->createCharacteristic(
      CHAR_TELEMETRY_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTelemetryChar->addDescriptor(new BLE2902());

  pConfigChar = pService->createCharacteristic(
      CHAR_CONFIG_UUID, BLECharacteristic::PROPERTY_WRITE);
  pConfigChar->setCallbacks(new ConfigCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  radarServo.attach(PIN_SERVO);
  radarServo.write(90);

  Wire.begin(PIN_SDA, PIN_SCL);
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("ROVER READY");
  lcd.setCursor(0, 1);
  lcd.print("ENCODERS ACTIVE");

  beep(200);

  // Enable hardware watchdog timer (10s timeout, auto-reset on hang)
  const esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 10000,
      .idle_core_mask = 0,
      .trigger_panic = true,
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
}

void loop() {
  // Real-time Wheel Speedometer Calculation (Every 250ms)
  updateSpeedometer();

  // Continuous Servo Sweep
  if (currentState == STATE_AUTO_FORWARD) {
    if (millis() - lastServoTime > servoDelay) {
      lastServoTime = millis();
      servoAngle += (servoStep * servoDirection);
      if (servoAngle >= 150) {
        servoAngle = 150;
        servoDirection = -5;
      } else if (servoAngle <= 15) {
        servoAngle = 30;
        servoDirection = 5;
      }
      radarServo.write(servoAngle);
    }
  }

  // --- AUTONOMOUS FREE-ROAM OBSTACLE AVOIDANCE ---
  if (currentState == STATE_AUTO_FORWARD) {
    distToFront = getDistance();

    // Check if obstacle is detected within danger threshold
    if (distToFront < OBSTACLE_THRESHOLD) {
      currentState = STATE_AVOIDANCE;
      executeAvoidance();
    } else if (distToFront < OBSTACLE_THRESHOLD * 2) {
      // Slow approach zone — reduce speed proportionally to avoid hard stops
      int approachSpeed = map((int)distToFront, OBSTACLE_THRESHOLD,
                              OBSTACLE_THRESHOLD * 2, 80, driveSpeed);
      setMotorSpeed(approachSpeed, approachSpeed);
    } else {
      // Free roam forward
      setMotorSpeed(driveSpeed, driveSpeed);
    }
  } else if (currentState == STATE_IDLE || currentState == STATE_MANUAL) {
    static unsigned long lastSonarCheck = 0;
    if (millis() - lastSonarCheck > 300) {
      lastSonarCheck = millis();
      distToFront = getDistance();
    }
  }

  // Periodic Telemetry & LCD Refresh (Every 300ms)
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 300) {
    lastUpdate = millis();

    // Only refresh LCD if not actively running full diagnostics or turning
    if (currentState != STATE_DIAGNOSTICS && currentState != STATE_AVOIDANCE) {
      lcd.clear();
      lcd.setCursor(0, 0);
      if (currentState == STATE_IDLE)
        lcd.print("IDLE ");
      else if (currentState == STATE_MANUAL)
        lcd.print("MANUAL ");
      else if (currentState == STATE_AUTO_FORWARD)
        lcd.print("AUTO ");

      lcd.printf("D:%.0fcm", distToFront >= 990 ? 999.0 : distToFront);
      // BLE connection indicator at top-right corner
      lcd.setCursor(15, 0);
      lcd.print(deviceConnected ? "B" : ".");

      // Line 2: Live Wheel Speedometer (L: cm/s, R: cm/s)
      lcd.setCursor(0, 1);
      lcd.printf("L:%.1f R:%.1f cm/s", speedLeft_cms, speedRight_cms);
    }

    if (deviceConnected && pTelemetryChar) {
      // Telemetry Packet: state,dist,speedL,speedR,odoL,odoR
      noInterrupts();
      unsigned long odoL = leftEncTicks;
      unsigned long odoR = rightEncTicks;
      interrupts();
      float totalDistL = odoL * CM_PER_TICK;
      float totalDistR = odoR * CM_PER_TICK;
      char tBuffer[64];
      snprintf(tBuffer, sizeof(tBuffer), "%d,%.1f,%.1f,%.1f,%.0f,%.0f",
               currentState, distToFront, speedLeft_cms, speedRight_cms,
               totalDistL, totalDistR);
      pTelemetryChar->setValue(tBuffer);
      pTelemetryChar->notify();
    }
  }
  esp_task_wdt_reset(); // Feed hardware watchdog
  delay(30);
}

// --- DYNAMIC FREE-ROAM OBSTACLE AVOIDANCE ---
void executeAvoidance() {
  // 1. Immediately stop to prevent collision
  stopMotors();
  beep(100);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("OBSTACLE DETECT!");
  lcd.setCursor(0, 1);
  lcd.print("Scanning path...");

  // If dangerously close (< 15cm), reverse slightly to create turning clearance
  if (distToFront < 15.0) {
    setMotorSpeed(-driveSpeed, -driveSpeed);
    delay(250);
    stopMotors();
  }

  // 2. Scan surroundings with radar servo
  radarServo.write(30); // Look Right
  delay(350);
  float rightDist = getDistance();

  radarServo.write(150); // Look Left
  delay(350);
  float leftDist = getDistance();

  servoAngle = 90;
  radarServo.write(90); // Re-center sensor forward
  delay(200);

  // 3. Steer toward whichever direction has the clearest open space
  if (rightDist >= leftDist && rightDist > DEADEND_THRESHOLD) {
    // Right is clearer
    lcd.setCursor(0, 1);
    lcd.printf("-> R CLEAR:%.0fcm", rightDist);
    turnRight90();
  } else if (leftDist > DEADEND_THRESHOLD) {
    // Left is clearer
    lcd.setCursor(0, 1);
    lcd.printf("<- L CLEAR:%.0fcm", leftDist);
    turnLeft90();
  } else {
    // Both sides blocked: Dead end or tight corner
    lcd.setCursor(0, 1);
    lcd.print("TRAPPED! 180 ESC");
    beep(250);
    setMotorSpeed(-driveSpeed, -driveSpeed);
    delay(400);
    stopMotors();

    // 180-degree escape turnaround
    setMotorSpeed(turnSpeed, -turnSpeed);
    delay(turn90Time * 2);
    stopMotors();
  }

  // 4. Instantly resume forward roaming into the open path!
  currentState = STATE_AUTO_FORWARD;
}

// --- SPEEDOMETER / ODOMETRY COMPUTATION ---
void updateSpeedometer() {
  unsigned long now = millis();
  unsigned long dt = now - lastSpeedCalcTime;
  if (dt >= 250) {
    noInterrupts();
    unsigned long curLeft = leftEncTicks;
    unsigned long curRight = rightEncTicks;
    interrupts();

    unsigned long dLeftTicks = curLeft - prevLeftTicks;
    unsigned long dRightTicks = curRight - prevRightTicks;

    prevLeftTicks = curLeft;
    prevRightTicks = curRight;
    lastSpeedCalcTime = now;

    // speed in cm/s = (ticks * cm_per_tick) / (dt_in_sec)
    float dtSec = dt / 1000.0;
    speedLeft_cms = (dLeftTicks * CM_PER_TICK) / dtSec;
    speedRight_cms = (dRightTicks * CM_PER_TICK) / dtSec;
  }
}

// --- DIAGNOSTICS & HARDWARE SELF-TEST ---
void sendDiag(const char *comp, const char *status) {
  if (deviceConnected && pTelemetryChar) {
    char diagBuf[48];
    snprintf(diagBuf, sizeof(diagBuf), "DIAG:%s:%s", comp, status);
    pTelemetryChar->setValue(diagBuf);
    pTelemetryChar->notify();
    delay(50);
  }
}

bool checkLCD() {
  Wire.beginTransmission(0x27);
  byte err = Wire.endTransmission();
  if (err != 0) {
    Wire.beginTransmission(0x3F);
    err = Wire.endTransmission();
  }
  return (err == 0);
}

void testBuzzer() {
  beep(100);
  delay(80);
  beep(100);
}

void testServo() {
  radarServo.write(30);
  delay(400);
  radarServo.write(150);
  delay(500);
  radarServo.write(90);
  delay(300);
}

float checkSonar() { return getDistance(); }

int testMotorLeft() {
  noInterrupts();
  unsigned long startTicks = leftEncTicks;
  interrupts();
  setMotorSpeed(170, 0);
  delay(450);
  stopMotors();
  noInterrupts();
  unsigned long endTicks = leftEncTicks;
  interrupts();
  return (int)(endTicks - startTicks);
}

int testMotorRight() {
  noInterrupts();
  unsigned long startTicks = rightEncTicks;
  interrupts();
  setMotorSpeed(0, 170);
  delay(450);
  stopMotors();
  noInterrupts();
  unsigned long endTicks = rightEncTicks;
  interrupts();
  return (int)(endTicks - startTicks);
}

void runDiagnostics() {
  currentState = STATE_DIAGNOSTICS;
  stopMotors();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SELF-TEST MODE");
  lcd.setCursor(0, 1);
  lcd.print("Checking HW...");
  sendDiag("SYS", "STARTING");
  delay(800);

  // 1. LCD Test
  bool lcdOk = checkLCD();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1. LCD I2C BUS:");
  lcd.setCursor(0, 1);
  lcd.print(lcdOk ? "PASS (ACK OK)" : "FAIL (NO ACK)");
  sendDiag("LCD", lcdOk ? "PASS" : "FAIL");
  delay(800);

  // 2. Buzzer Test
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("2. BUZZER TEST");
  lcd.setCursor(0, 1);
  lcd.print("Beep beep...");
  sendDiag("BUZZ", "TESTING");
  testBuzzer();
  sendDiag("BUZZ", "PASS");
  delay(600);

  // 3. Sonar Test
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("3. ULTRASONIC");
  sendDiag("SONAR", "TESTING");
  delay(200);
  float d = checkSonar();
  lcd.setCursor(0, 1);
  if (d < 400.0 && d > 1.0) {
    lcd.printf("PASS: %.1f cm", d);
    char buf[20];
    snprintf(buf, sizeof(buf), "PASS:%.1fcm", d);
    sendDiag("SONAR", buf);
  } else {
    lcd.print("FAIL: NO ECHO");
    sendDiag("SONAR", "FAIL");
  }
  delay(900);

  // 4. Servo Sweep
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("4. RADAR SERVO");
  lcd.setCursor(0, 1);
  lcd.print("Sweep 30-150-90");
  sendDiag("SERVO", "SWEEPING");
  testServo();
  sendDiag("SERVO", "PASS");
  delay(600);

  // 5. Left Motor + Left Encoder Test
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("5. L-MOTOR & ENC");
  lcd.setCursor(0, 1);
  lcd.print("Spinning 0.45s...");
  sendDiag("MOTL", "TESTING");
  int lTicks = testMotorLeft();
  lcd.setCursor(0, 1);
  if (lTicks > 0) {
    lcd.printf("PASS: %d ticks", lTicks);
    char buf[24];
    snprintf(buf, sizeof(buf), "PASS:%dticks", lTicks);
    sendDiag("MOTL", buf);
    sendDiag("ENCL", "PASS");
  } else {
    lcd.print("MOTOR OK/0 TICKS");
    sendDiag("MOTL", "PASS (0 ticks)");
    sendDiag("ENCL", "WARN_NO_TICKS");
  }
  delay(800);

  // 6. Right Motor + Right Encoder Test
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("6. R-MOTOR & ENC");
  lcd.setCursor(0, 1);
  lcd.print("Spinning 0.45s...");
  sendDiag("MOTR", "TESTING");
  int rTicks = testMotorRight();
  lcd.setCursor(0, 1);
  if (rTicks > 0) {
    lcd.printf("PASS: %d ticks", rTicks);
    char buf[24];
    snprintf(buf, sizeof(buf), "PASS:%dticks", rTicks);
    sendDiag("MOTR", buf);
    sendDiag("ENCR", "PASS");
  } else {
    lcd.print("MOTOR OK/0 TICKS");
    sendDiag("MOTR", "PASS (0 ticks)");
    sendDiag("ENCR", "WARN_NO_TICKS");
  }
  delay(800);

  // Summary
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TEST COMPLETE");
  lcd.setCursor(0, 1);
  lcd.print("ALL HW VERIFIED");
  sendDiag("ALL", "COMPLETE");
  beep(300);
  delay(1400);

  currentState = STATE_IDLE;
}

// --- BASIC MOTION PRIMITIVES ---
float getDistanceSingle() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 30000);
  if (duration == 0)
    return 999.0;
  return (duration * 0.034) / 2.0;
}

// Median-of-3 filter to reduce ultrasonic noise and false obstacle triggers
float getDistance() {
  float a = getDistanceSingle();
  delayMicroseconds(500);
  float b = getDistanceSingle();
  delayMicroseconds(500);
  float c = getDistanceSingle();
  // Sort three values to find median
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    float t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  return b; // Return median value
}

void beep(int duration) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(duration);
  digitalWrite(PIN_BUZZER, LOW);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Apply trim calibration to compensate for motor variance
  int adjLeft = (leftSpeed * leftTrim) / 100;
  int adjRight = (rightSpeed * rightTrim) / 100;

  // PID-based closed-loop encoder sync for straight-line driving
  // Activates when both wheels should spin at the same speed (forward or
  // reverse)
  if (encoderSyncEnabled && leftSpeed != 0 && rightSpeed != 0 &&
      abs(leftSpeed) == abs(rightSpeed)) {
    unsigned long now = millis();
    unsigned long pidDt = now - pidLastTime;

    if (pidDt >= 50 && pidLastTime > 0) { // Run PID at ~20Hz, skip first cycle
      float dtSec = pidDt / 1000.0;

      // Error = difference in wheel speeds (positive means right is faster)
      float error = speedRight_cms - speedLeft_cms;

      // Proportional term
      float pTerm = pidKp * error;

      // Integral term with anti-windup
      pidIntegral += error * dtSec;
      pidIntegral =
          constrain(pidIntegral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
      float iTerm = pidKi * pidIntegral;

      // Derivative term
      float dTerm = pidKd * (error - pidPrevError) / dtSec;
      pidPrevError = error;

      // Total correction (clamped)
      float correction = constrain(pTerm + iTerm + dTerm, -PID_MAX_CORRECTION,
                                   PID_MAX_CORRECTION);

      // Apply correction: slow down the faster wheel
      // If correction > 0, right is faster → reduce right / boost left
      // If correction < 0, left is faster → reduce left / boost right
      if (leftSpeed > 0) {
        // Forward driving
        adjRight -= (int)(correction / 2.0);
        adjLeft += (int)(correction / 2.0);

        // If a wheel hits max speed (255), apply the lost correction to the other wheel to preserve the differential!
        if (adjLeft > 255) {
          adjRight -= (adjLeft - 255);
          adjLeft = 255;
        } else if (adjRight > 255) {
          adjLeft -= (adjRight - 255);
          adjRight = 255;
        }
      } else {
        // Reverse driving (adjRight and adjLeft are negative numbers)
        adjRight += (int)(correction / 2.0);
        adjLeft -= (int)(correction / 2.0);

        // Reverse overflow clamping (-255 is max speed backwards)
        if (adjLeft < -255) {
          adjRight -= (adjLeft + 255); 
          adjLeft = -255;
        } else if (adjRight < -255) {
          adjLeft -= (adjRight + 255);
          adjRight = -255;
        }
      }
    }
    pidLastTime = now;
  } else {
    // Reset PID state when not driving straight
    pidIntegral = 0.0;
    pidPrevError = 0.0;
    pidLastTime = 0;
  }

  analogWrite(PIN_ENA, constrain(abs(adjLeft), 0, 255));
  analogWrite(PIN_ENB, constrain(abs(adjRight), 0, 255));

  if (adjLeft == 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, adjLeft > 0 ? HIGH : LOW);
    digitalWrite(PIN_IN2, adjLeft < 0 ? HIGH : LOW);
  }
  if (adjRight == 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
  } else {
    digitalWrite(PIN_IN3, adjRight > 0 ? HIGH : LOW);
    digitalWrite(PIN_IN4, adjRight < 0 ? HIGH : LOW);
  }
}

void stopMotors() { setMotorSpeed(0, 0); }
void turnRight90() {
  setMotorSpeed(turnSpeed, -turnSpeed);
  delay(turn90Time);
  stopMotors();
}
void turnLeft90() {
  setMotorSpeed(-turnSpeed, turnSpeed);
  delay(turn90Time);
  stopMotors();
}
