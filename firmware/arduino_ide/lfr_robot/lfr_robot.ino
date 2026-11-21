/*
 * Arduino Nano — Line Follower, Obstacle Detection & Fire Extinguisher Robot
 * MCU: ATmega328P (Arduino Nano V3)
 * Motor Driver: L298N (D5/D6 Timer0 PWM, D7-D10 Dir)
 * Sensors: 5-IR Array (A0-A3, A6), Obstacle (D2 INT0, D4 Trig), Flame (D3 INT1, A7)
 * Actuator: Extinguisher Pump/Fan via MOSFET or Relay (D12)
 */

// ======================== CONFIGURATION ========================
#define USE_ULTRASONIC_OBS   true   // true = HC-SR04 (Trig D4, Echo D2); false = 3-pin IR on D2
#define LINE_IS_HIGH         true   // true if black line reads > IR_THRESHOLD; false if < IR_THRESHOLD
#define EXT_ACTIVE_LOW       false  // false = Logic-Level N-MOSFET (Active-HIGH); true = Active-LOW Relay
#define DEBUG_SERIAL         false  // Set true for Serial Monitor diagnostics (115200 baud)

// ======================== PIN DEFINITIONS ========================
// L298N Motor Driver
const uint8_t ENA = 5;   // Left Motor Speed (Timer0 PWM)
const uint8_t ENB = 6;   // Right Motor Speed (Timer0 PWM)
const uint8_t IN1 = 7;   // Left Dir A
const uint8_t IN2 = 8;   // Left Dir B
const uint8_t IN3 = 9;   // Right Dir A
const uint8_t IN4 = 10;  // Right Dir B

// 5-IR Sensor Array (A6 is analog-only on Nano -> always read via analogRead)
const uint8_t IR_PINS[5] = {A0, A1, A2, A3, A6}; // Far Left -> Far Right
const int IR_WEIGHTS[5]  = {-2, -1, 0, 1, 2};
const int IR_THRESHOLD   = 512;

// Obstacle Detection Sensor
const uint8_t OBS_ECHO    = 2;    // INT0 (HC-SR04 Echo or 3-pin IR OUT)
const uint8_t OBS_TRIG    = 4;    // HC-SR04 Trigger (unused if 3-pin IR)
const float   OBS_STOP_CM = 15.0; // Stop distance in cm

// Fire / Flame Sensor (A7 is analog-only on Nano)
const uint8_t FLAME_DO          = 3;   // INT1 (Active-LOW)
const uint8_t FLAME_AO          = A7;  // Analog intensity (lower = stronger flame)
const int     FLAME_ANALOG_TRIG = 400; // Analog confirmation threshold
const uint8_t FLAME_ADC_SAMPLES = 8;   // Averaging count to reject IR spikes

// Extinguisher Driver (MOSFET Gate or Relay IN)
const uint8_t EXT_PIN = 12;

// ======================== TIMING & PID TUNING ========================
const unsigned long OBS_INTERVAL_MS   = 60;    // Ultrasonic ping interval
const unsigned long FLAME_DEBOUNCE_MS = 75;    // Ignore rapid INT1 chatter
const unsigned long EXT_HOLD_MS       = 1000;  // Min extinguisher run time to prevent chatter

const int BASE_SPEED = 140;
const int MAX_SPEED  = 220;

float Kp = 35.0;
float Ki = 0.0;
float Kd = 18.0;

float lastError = 0.0;
float integral  = 0.0;

// ======================== INTERRUPT & RUNTIME STATE ========================
// Ultrasonic non-blocking echo state
volatile unsigned long echoStartUs = 0;
volatile unsigned long echoWidthUs = 0;
volatile bool echoNewData          = false;
volatile bool irObstacleFlag       = false;

// Flame interrupt & debounce state
volatile bool flameIrqFlag         = false;
volatile unsigned long lastFlameIrqMs = 0;
unsigned long extinguisherUntilMs  = 0;

unsigned long lastObsPingMs        = 0;
bool obstacleDetected              = false;

// INT0 (D2): HC-SR04 Rising/Falling edge timing OR 3-pin IR Falling edge
void obstacleISR() {
  if (USE_ULTRASONIC_OBS) {
    if (digitalRead(OBS_ECHO) == HIGH) {
      echoStartUs = micros();
    } else {
      echoWidthUs = micros() - echoStartUs;
      echoNewData = true;
    }
  } else {
    irObstacleFlag = true;
  }
}

// INT1 (D3): Active-LOW flame trigger with software debounce
void flameISR() {
  unsigned long now = millis();
  if (now - lastFlameIrqMs >= FLAME_DEBOUNCE_MS) {
    flameIrqFlag = true;
    lastFlameIrqMs = now;
  }
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

  // Obstacle sensor setup
  pinMode(OBS_ECHO, INPUT);
  if (USE_ULTRASONIC_OBS) {
    pinMode(OBS_TRIG, OUTPUT);
    digitalWrite(OBS_TRIG, LOW);
    attachInterrupt(digitalPinToInterrupt(OBS_ECHO), obstacleISR, CHANGE);
  } else {
    attachInterrupt(digitalPinToInterrupt(OBS_ECHO), obstacleISR, FALLING);
  }

  // Flame sensor digital interrupt (A7 needs no pinMode)
  pinMode(FLAME_DO, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLAME_DO), flameISR, FALLING);

  // Extinguisher actuator (start OFF)
  pinMode(EXT_PIN, OUTPUT);
  setExtinguisher(false);

  // Status LED on D13
  pinMode(LED_BUILTIN, OUTPUT);

  if (DEBUG_SERIAL) {
    Serial.begin(115200);
    Serial.println(F("System Initialized."));
  }
}

// ======================== MAIN LOOP ========================
void loop() {
  unsigned long now = millis();

  // --- PRIORITY 1: FIRE DETECTION & EXTINGUISHER HOLD ---
  if (isFireDetected()) {
    extinguisherUntilMs = now + EXT_HOLD_MS;
  }

  if (now < extinguisherUntilMs) {
    stopMotors();
    setExtinguisher(true);
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  } else {
    setExtinguisher(false);
    digitalWrite(LED_BUILTIN, LOW);
  }

  // --- PRIORITY 2: NON-BLOCKING OBSTACLE DETECTION ---
  updateObstacleState(now);
  if (obstacleDetected) {
    stopMotors();
    if (DEBUG_SERIAL) Serial.println(F("[OBSTACLE] Halted."));
    return;
  }

  // --- PRIORITY 3: 5-IR PID LINE FOLLOWING ---
  followLinePID();
}

// ======================== SENSOR FUNCTIONS ========================
int readFlameAnalogAvg() {
  long sum = 0;
  for (uint8_t i = 0; i < FLAME_ADC_SAMPLES; i++) {
    sum += analogRead(FLAME_AO);
  }
  return (int)(sum / FLAME_ADC_SAMPLES);
}

bool isFireDetected() {
  bool digitalFire = (digitalRead(FLAME_DO) == LOW);
  int analogAvg    = readFlameAnalogAvg();
  bool analogFire  = (analogAvg < FLAME_ANALOG_TRIG);

  // Require analog confirmation if triggered by interrupt to reject ambient glitches
  bool confirmed = (digitalFire && analogFire) || (analogAvg < (FLAME_ANALOG_TRIG - 80));

  if (flameIrqFlag) {
    flameIrqFlag = false;
    if (digitalFire || analogFire) confirmed = true;
  }

  if (DEBUG_SERIAL && confirmed) {
    Serial.print(F("[FIRE] AO_Avg="));
    Serial.println(analogAvg);
  }
  return confirmed;
}

void updateObstacleState(unsigned long now) {
  if (USE_ULTRASONIC_OBS) {
    // 1. Trigger a new 10 us pulse every OBS_INTERVAL_MS
    if (now - lastObsPingMs >= OBS_INTERVAL_MS) {
      lastObsPingMs = now;
      digitalWrite(OBS_TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(OBS_TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(OBS_TRIG, LOW);
    }

    // 2. Process completed echo asynchronously
    if (echoNewData) {
      noInterrupts();
      unsigned long widthUs = echoWidthUs;
      echoNewData = false;
      interrupts();

      if (widthUs > 15000UL || widthUs == 0) {
        obstacleDetected = false;
      } else {
        float distCm = (widthUs * 0.0343f) / 2.0f;
        obstacleDetected = (distCm > 0.5f && distCm <= OBS_STOP_CM);
        if (DEBUG_SERIAL) {
          Serial.print(F("Dist(cm): "));
          Serial.println(distCm);
        }
      }
    }
  } else {
    // 3-pin digital IR obstacle sensor (Active-LOW)
    obstacleDetected = (digitalRead(OBS_ECHO) == LOW) || irObstacleFlag;
    if (digitalRead(OBS_ECHO) == HIGH) irObstacleFlag = false;
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
  }

  float error = (activeCount > 0)
                  ? ((float)weightedSum / (float)activeCount)
                  : ((lastError >= 0) ? 2.5f : -2.5f);

  integral = constrain(integral + error, -50.0f, 50.0f);
  float derivative = error - lastError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int leftSpeed  = constrain((int)(BASE_SPEED + correction), -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain((int)(BASE_SPEED - correction), -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
}

// ======================== ACTUATOR FUNCTIONS ========================
void setMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftSpeed);
  }

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
  if (EXT_ACTIVE_LOW) {
    digitalWrite(EXT_PIN, state ? LOW : HIGH);
  } else {
    digitalWrite(EXT_PIN, state ? HIGH : LOW);
  }
}