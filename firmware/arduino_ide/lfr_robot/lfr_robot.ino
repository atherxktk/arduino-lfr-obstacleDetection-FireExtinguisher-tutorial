/*
 * Arduino Nano — Line Follower, Obstacle & Fire Detection Robot
 * MCU: ATmega328P (Arduino Nano V3)
 * Motor Driver: L298N
 * Sensors: 5-IR Array, Obstacle Sensor (HC-SR04 or 3-pin IR), Flame Sensor
 */

// ======================== CONFIGURATION ========================
#define USE_ULTRASONIC_OBS   true   // true = HC-SR04 (Trig D4, Echo D2); false = 3-pin IR Obstacle on D2
#define LINE_IS_HIGH         true   // true if sensor reads HIGH (> threshold) on black line; false if LOW
#define RELAY_ACTIVE_LOW     true   // Most 5V relay modules trigger on LOW; set false for MOSFET/active-HIGH
#define DEBUG_SERIAL         false  // Set true to print sensor values to Serial Monitor (115200 baud)

// ======================== PIN DEFINITIONS ========================
// L298N Motor Driver (Timer0 PWM on D5/D6, Digital Dir on D7-D10)
const uint8_t ENA = 5;   // Left Motor Speed (PWM)
const uint8_t ENB = 6;   // Right Motor Speed (PWM)
const uint8_t IN1 = 7;   // Left Dir A
const uint8_t IN2 = 8;   // Left Dir B
const uint8_t IN3 = 9;   // Right Dir A
const uint8_t IN4 = 10;  // Right Dir B

// 5-IR Sensor Array (A6 is analog-only on Nano -> always read via analogRead)
const uint8_t IR_PINS[5] = {A0, A1, A2, A3, A6}; // Far Left -> Far Right
const int IR_WEIGHTS[5]  = {-2, -1, 0, 1, 2};
const int IR_THRESHOLD   = 512;  // Adjust after testing on your track (0-1023)

// Obstacle Detection Sensor
const uint8_t OBS_ECHO = 2;      // INT0 (also used as OUT if 3-pin IR sensor)
const uint8_t OBS_TRIG = 4;      // Unused if USE_ULTRASONIC_OBS == false
const float   OBS_STOP_CM = 15.0; // Stop distance in cm

// Fire / Flame Sensor (A7 is analog-only on Nano)
const uint8_t FLAME_DO = 3;      // INT1 (typically Active-LOW when flame detected)
const uint8_t FLAME_AO = A7;     // Analog intensity (lower = stronger flame on standard modules)
const int     FLAME_ANALOG_TRIG = 400; // Backup analog threshold

// Optional Extinguisher Actuator (Relay / MOSFET for Fan or Pump)
const uint8_t EXT_PIN = 12;

// ======================== PID & MOTOR TUNING ========================
const int BASE_SPEED = 140;      // Cruise PWM (0-255)
const int MAX_SPEED  = 220;      // Max PWM cap (0-255)

float Kp = 35.0;
float Ki = 0.0;
float Kd = 18.0;

float lastError = 0.0;
float integral  = 0.0;

// ======================== RUNTIME STATE ========================
volatile bool flameInterruptFlag = false;
unsigned long lastObsCheckMs     = 0;
const unsigned long OBS_INTERVAL = 60; // Check ultrasonic every 60 ms
bool obstacleDetected            = false;

// Interrupt Service Routine for instant flame detection (D3 / INT1)
void flameISR() {
  flameInterruptFlag = true;
}

// ======================== SETUP ========================
void setup() {
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();

  // Obstacle sensor pins
  if (USE_ULTRASONIC_OBS) {
    pinMode(OBS_TRIG, OUTPUT);
    digitalWrite(OBS_TRIG, LOW);
    pinMode(OBS_ECHO, INPUT);
  } else {
    pinMode(OBS_ECHO, INPUT);
  }

  // Flame sensor digital interrupt (A7 requires no pinMode)
  pinMode(FLAME_DO, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLAME_DO), flameISR, FALLING);

  // Extinguisher output (start OFF)
  pinMode(EXT_PIN, OUTPUT);
  setExtinguisher(false);

  // Heartbeat LED on D13
  pinMode(LED_BUILTIN, OUTPUT);

  if (DEBUG_SERIAL) {
    Serial.begin(115200);
    Serial.println(F("System Initialized."));
  }
}

// ======================== MAIN LOOP ========================
void loop() {
  // --- PRIORITY 1: FIRE / FLAME DETECTION ---
  if (isFireDetected()) {
    stopMotors();
    setExtinguisher(true);
    digitalWrite(LED_BUILTIN, HIGH);

    if (DEBUG_SERIAL) {
      Serial.print(F("[FIRE] AO="));
      Serial.println(analogRead(FLAME_AO));
    }

    delay(100); // Keep extinguisher active while polling
    return;
  } else {
    setExtinguisher(false);
    digitalWrite(LED_BUILTIN, LOW);
    flameInterruptFlag = false;
  }

  // --- PRIORITY 2: OBSTACLE DETECTION ---
  if (millis() - lastObsCheckMs >= OBS_INTERVAL) {
    lastObsCheckMs = millis();
    obstacleDetected = checkObstacle();
  }

  if (obstacleDetected) {
    stopMotors();
    if (DEBUG_SERIAL) Serial.println(F("[OBSTACLE] Halted."));
    return;
  }

  // --- PRIORITY 3: 5-IR PID LINE FOLLOWING ---
  followLinePID();
}

// ======================== SENSOR FUNCTIONS ========================
bool isFireDetected() {
  bool digitalFire = (digitalRead(FLAME_DO) == LOW);
  int analogVal    = analogRead(FLAME_AO);
  bool analogFire  = (analogVal < FLAME_ANALOG_TRIG);

  // Clear stale interrupt flag if neither sensor currently sees fire
  if (!digitalFire && !analogFire) {
    flameInterruptFlag = false;
    return false;
  }
  return (flameInterruptFlag || digitalFire || analogFire);
}

bool checkObstacle() {
  if (USE_ULTRASONIC_OBS) {
    digitalWrite(OBS_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(OBS_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(OBS_TRIG, LOW);

    // 15,000 us timeout (~2.5 m) prevents blocking the loop
    unsigned long duration = pulseIn(OBS_ECHO, HIGH, 15000UL);
    if (duration == 0) return false; // No echo within range

    float distanceCm = (duration * 0.0343f) / 2.0f;
    if (DEBUG_SERIAL) {
      Serial.print(F("Dist(cm): "));
      Serial.println(distanceCm);
    }
    return (distanceCm > 0.5f && distanceCm <= OBS_STOP_CM);
  } else {
    // Standard 3-pin IR obstacle module outputs LOW when obstacle is present
    return (digitalRead(OBS_ECHO) == LOW);
  }
}

void followLinePID() {
  int activeCount = 0;
  int weightedSum = 0;

  for (uint8_t i = 0; i < 5; i++) {
    int raw = analogRead(IR_PINS[i]);
    bool onLine = LINE_IS_HIGH ? (raw > IR_THRESHOLD) : (raw < IR_THRESHOLD);

    if (onLine) {
      weightedSum += IR_WEIGHTS[i];
      activeCount++;
    }

    if (DEBUG_SERIAL) {
      Serial.print(raw);
      Serial.print('\t');
    }
  }

  float error = 0.0;
  if (activeCount > 0) {
    error = (float)weightedSum / (float)activeCount;
  } else {
    // Line lost: keep turning in the direction the line was last seen
    error = (lastError >= 0) ? 2.5f : -2.5f;
  }

  integral = constrain(integral + error, -50.0f, 50.0f);
  float derivative = error - lastError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int leftSpeed  = constrain((int)(BASE_SPEED + correction), -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain((int)(BASE_SPEED - correction), -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);

  if (DEBUG_SERIAL) {
    Serial.print(F("Err: ")); Serial.print(error);
    Serial.print(F(" | L: ")); Serial.print(leftSpeed);
    Serial.print(F(" R: ")); Serial.println(rightSpeed);
  }
}

// ======================== ACTUATOR FUNCTIONS ========================
void setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor (IN1, IN2, ENA)
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftSpeed);
  }

  // Right Motor (IN3, IN4, ENB)
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightSpeed);
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void setExtinguisher(bool state) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(EXT_PIN, state ? LOW : HIGH);
  } else {
    digitalWrite(EXT_PIN, state ? HIGH : LOW);
  }
}