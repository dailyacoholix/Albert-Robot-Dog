#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#if defined(ESP32)
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ============================================================
// Servo configuration
// ============================================================
#define SERVOMIN   50
#define SERVOMAX   550
#define CENTER     325
#define NUM_SERVOS 8
#define NECK_SERVO 15

// Radar sweep — wider and slower
#define SWEEP_RANGE  120
#define SWEEP_STEPS   4

// Smooth neck move config
#define SMOOTH_STEPS  30          // interpolation steps per move
#define SMOOTH_DELAY  20          // ms per step → ~600ms per swing

// Walk motion parameters
#define DELAY_TIME    3
#define SPEED_FACTOR  1.5
#define MAX_STEP      8

// ============================================================
// Target detection / loss thresholds — HYSTERESIS
// ============================================================
#define DETECT_DIST   20          // cm — acquire target
#define LOSE_DIST     30          // cm — lose target (wider gap = more stable)

// Consecutive miss counter — ignore single noisy pings
#define MISS_THRESHOLD  5         // need N consecutive misses before giving up

// ============================================================
// Ultrasonic
// ============================================================
#define TRIG_PIN 12
#define ECHO_PIN 13

// ============================================================
// Walk oscillation globals
// ============================================================
const float FREQUENCY = 0.006;
const float AMPLITUDE = 50.0;

int direction = 1;
unsigned long currentMillis = 0, oldMillis = 0, innerTime = 0;

float off_set_walk[NUM_SERVOS] = {0, -100, 0, 100, -50, -50, 50, +50};
int   pulselen[NUM_SERVOS];

float phase[8] = {
  PI  +  PI/8,   PI/2 + PI  +  PI/8,
  0   +  PI/8,   PI/2        +  PI/8,
  0   -  PI/8,   PI/2        -  PI/8,
  PI  -  PI/8,   PI/2 + PI  -  PI/8
};

// ============================================================
// State machine
// ============================================================
enum DogState { STATE_SCAN, STATE_LEFT, STATE_RIGHT, STATE_FWD };
DogState state = STATE_SCAN;

// ============================================================
// Neck tracking
// ============================================================
int currentNeckPos = CENTER;

// ============================================================
// Easing + smooth servo
// ============================================================
float easeInOut(float t) {
  return 0.5f * (1.0f - cos(t * PI));
}

void smoothServo(uint8_t channel, int &currentPos, int targetPos,
                 int steps = SMOOTH_STEPS, int stepDelay = SMOOTH_DELAY) {
  if (currentPos == targetPos) return;
  int startPos = currentPos;
  for (int i = 1; i <= steps; i++) {
    float t      = (float)i / (float)steps;
    float eased  = easeInOut(t);
    int   newPos = startPos + (int)((targetPos - startPos) * eased);
    pwm.setPWM(channel, 0, newPos);
    delay(stepDelay);
  }
  currentPos = targetPos;
}

// ============================================================
// Distance measurement
// ============================================================
long readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  if (distance == 0) distance = 9999;
  return distance;
}

// Averaged to reduce ultrasonic jitter
long readDistanceAvg(int samples = 3) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readDistance();
    delay(5);
  }
  return sum / samples;
}

// ============================================================
// Target loss check — hysteresis + consecutive miss counter
// Returns true only if target is GENUINELY lost
// ============================================================
bool targetLost() {
  static int missCount = 0;

  long d = readDistanceAvg();

  if (d > LOSE_DIST) {
    missCount++;
    Serial.print("Miss "); Serial.print(missCount);
    Serial.print("/"); Serial.println(MISS_THRESHOLD);

    if (missCount >= MISS_THRESHOLD) {
      missCount = 0;
      return true;    // confirmed lost
    }
  } else {
    missCount = 0;    // any good reading resets the counter
  }

  return false;
}

// ============================================================
// Transition helper
// ============================================================
void transitionToScan() {
  Serial.println("Target lost → easing back to scan");
  smoothServo(NECK_SERVO, currentNeckPos, CENTER, SMOOTH_STEPS, SMOOTH_DELAY);
  state = STATE_SCAN;
}

// ============================================================
// Walk sequences
// ============================================================
void runOscillation(float localAmp[8]) {
  oldMillis     = currentMillis;
  currentMillis = millis();
  innerTime    += direction * (currentMillis - oldMillis);

  for (int s = 0; s < NUM_SERVOS; s += 2) {
    pulselen[s] = CENTER + off_set_walk[s]
                + localAmp[s] * AMPLITUDE * cos(FREQUENCY * innerTime + phase[s]);
    pwm.setPWM(s, 0, pulselen[s]);
  }

  for (int s = 1; s < NUM_SERVOS; s += 2) {
    float c     = max(0.0f, cos(FREQUENCY * innerTime + phase[s]));
    pulselen[s] = CENTER + off_set_walk[s] + localAmp[s] * AMPLITUDE * c;
    pwm.setPWM(s, 0, pulselen[s]);
  }

  delay(DELAY_TIME);
}

void runWalkSequence(Adafruit_PWMServoDriver &unused) {
  float a[8] = {1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0};
  runOscillation(a);
}

void runWalkSequenceLeft(Adafruit_PWMServoDriver &unused) {
  float a[8] = {1.5, 1.5, -0.2, -0.2, 1.5, 1.5, -0.2, -0.2};
  runOscillation(a);
}

void runWalkSequenceRight(Adafruit_PWMServoDriver &unused) {
  float a[8] = {0.2, 0.2, -1.5, -1.5, 0.2, 0.2, -1.5, -1.5};
  runOscillation(a);
}

void runWalkSequenceLeftSpot(Adafruit_PWMServoDriver &unused) {
  float a[8] = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
  runOscillation(a);
}

void runWalkSequenceRightSpot(Adafruit_PWMServoDriver &unused) {
  float a[8] = {-1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5};
  runOscillation(a);
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);

#if defined(ESP32)
  SerialBT.begin("ESP32_RadarDog");
#endif

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  for (int i = 0; i < NUM_SERVOS; i++)
    pwm.setPWM(i, 0, CENTER);

  currentNeckPos = CENTER;
  pwm.setPWM(NECK_SERVO, 0, CENTER);
  delay(500);
}

// ============================================================
// Main loop
// ============================================================
void loop() {

  switch (state) {

    // ----------------------------------------------------------
    // SCAN MODE
    // ----------------------------------------------------------
    case STATE_SCAN: {

      const int positions[SWEEP_STEPS] = {
        CENTER - SWEEP_RANGE,
        CENTER,
        CENTER + SWEEP_RANGE,
        CENTER
      };

      static int index = 0;
      int targetPos = positions[index];

      smoothServo(NECK_SERVO, currentNeckPos, targetPos, SMOOTH_STEPS, SMOOTH_DELAY);

      long d = readDistanceAvg();
      Serial.print("SCAN pos "); Serial.print(targetPos);
      Serial.print("  dist = "); Serial.println(d);

      // Use DETECT_DIST (tight threshold) to acquire target
      if (d < DETECT_DIST) {
        if      (targetPos > CENTER + 15) { Serial.println("→ RIGHT"); state = STATE_RIGHT; }
        else if (targetPos < CENTER - 15) { Serial.println("→ LEFT");  state = STATE_LEFT;  }
        else                              { Serial.println("→ FWD");   state = STATE_FWD;   }
        break;
      }

      index = (index + 1) % SWEEP_STEPS;
      break;
    }

    // ----------------------------------------------------------
    // TURN LEFT
    // Uses LOSE_DIST + miss counter to release lock
    // ----------------------------------------------------------
    case STATE_LEFT:
      smoothServo(NECK_SERVO, currentNeckPos, CENTER - SWEEP_RANGE / 2, 15, 15);
      runWalkSequenceLeft(pwm);
      if (targetLost()) transitionToScan();
      break;

    // ----------------------------------------------------------
    // TURN RIGHT
    // ----------------------------------------------------------
    case STATE_RIGHT:
      smoothServo(NECK_SERVO, currentNeckPos, CENTER + SWEEP_RANGE / 2, 15, 15);
      runWalkSequenceRight(pwm);
      if (targetLost()) transitionToScan();
      break;

    // ----------------------------------------------------------
    // FORWARD
    // ----------------------------------------------------------
    case STATE_FWD:
      smoothServo(NECK_SERVO, currentNeckPos, CENTER, 15, 15);
      runWalkSequence(pwm);
      if (targetLost()) transitionToScan();
      break;
  }
}
