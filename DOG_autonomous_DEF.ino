#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "motion.h"

#if defined(ESP32)
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ----------------------------------------------------
// Servo configuration
// ----------------------------------------------------
#define SERVOMIN 50
#define SERVOMAX 550
#define CENTER   325
#define NUM_SERVOS 8
#define DELAY_TIME 3
#define SPEED_FACTOR 1.5
#define MAX_STEP 8

// ----------------------------------------------------
// Ultrasonic pins
// ----------------------------------------------------
#define TRIG_PIN 12
#define ECHO_PIN 13

// Neck servo channel
#define NECK_SERVO 15

// ----------------------------------------------------
// Global variables
// ----------------------------------------------------
float currentPos[NUM_SERVOS];
float targetPos[NUM_SERVOS];

unsigned long lastCheck = 0;
unsigned long lastBackwardScan = 0;
long distFwd = 0;
long distLeft = 0;
long distRight = 0;

// ----------------------------------------------------
// STATE MACHINE
// ----------------------------------------------------
enum DogState {
  STATE_FORWARD,   // Cammina in avanti
  STATE_LEFT,      // Gira a sinistra
  STATE_RIGHT,     // Gira a destra
  STATE_BACKWARD   // Va indietro
};

DogState state = STATE_FORWARD;

// ----------------------------------------------------
// Read from Serial or BT
// ----------------------------------------------------
String readCommand() {
  String input = "";
  if (Serial.available() > 0) 
    input = Serial.readStringUntil('\n');

#if defined(ESP32)
  else if (SerialBT.available() > 0)
    input = SerialBT.readStringUntil('\n');
#endif

  input.trim();
  return input;
}

// ----------------------------------------------------
// Ultrasonic distance sensor
// ----------------------------------------------------
long readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return 999;  
  }

  long distance = duration * 0.034 / 2;

  if (distance <= 0) {
    return 999;
  }

  return distance;
}

// ----------------------------------------------------
// Smooth neck movement
// ----------------------------------------------------
void moveNeckSlow(int from, int to, int steps = 10, int delayMs = 50) {
  float step = (to - from) / (float)steps;
  for (int i = 0; i <= steps; i++) {
    pwm.setPWM(NECK_SERVO, 0, from + step * i);
    delay(delayMs);
  }
}

// ----------------------------------------------------
// Return neck to center
// ----------------------------------------------------
void returnNeckToCenter() {
  pwm.setPWM(NECK_SERVO, 0, CENTER);
  delay(200);
}

// ----------------------------------------------------
// Setup
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);

#if defined(ESP32)
  SerialBT.begin("ESP32_RobotDog");
  Serial.println("Bluetooth started. Device name: ESP32_RobotDog");
#endif

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // Initialize servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPos[i] = CENTER;
    targetPos[i]  = CENTER;
    pwm.setPWM(i, 0, CENTER);
  }

  pwm.setPWM(NECK_SERVO, 0, CENTER);

  Serial.println("Robot dog autonomous mode started.");
#if defined(ESP32)
  SerialBT.println("Robot dog autonomous mode started.");
#endif
}

// ----------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------
void loop() {

  switch(state) {

    // ====================================================================
    //                           FORWARD STATE
    // ====================================================================
    case STATE_FORWARD:
      runWalkSequence(pwm);

      // Misura distanza ogni 1 secondo
      if (millis() - lastCheck >= 1000) {
        lastCheck = millis();

        distFwd = readDistance();
        Serial.print("FORWARD - Distance: ");
        Serial.println(distFwd);

#if defined(ESP32)
        SerialBT.print("FORWARD - Distance: ");
        SerialBT.println(distFwd);
#endif

        // Se distanza < 30, inizia la scansione
        if (distFwd < 30) {
          Serial.println("Obstacle detected! Scanning...");
          
          // Guarda a DESTRA
          moveNeckSlow(CENTER, CENTER + 80, 10, 30);
          delay(300);
          distRight = readDistance();
          Serial.print("Right distance: ");
          Serial.println(distRight);

          if (distRight > 30) {
            // Vai a DESTRA
            Serial.println("Going RIGHT");
            returnNeckToCenter();
            state = STATE_RIGHT;
            break;
          }

          // Altrimenti guarda a SINISTRA
          moveNeckSlow(CENTER + 80, CENTER - 80, 16, 30);
          delay(300);
          distLeft = readDistance();
          Serial.print("Left distance: ");
          Serial.println(distLeft);

          if (distLeft > 30) {
            // Vai a SINISTRA
            Serial.println("Going LEFT");
            returnNeckToCenter();
            state = STATE_LEFT;
            break;
          }

          // Altrimenti vai INDIETRO
          Serial.println("Both blocked! Going BACKWARD");
          returnNeckToCenter();
          state = STATE_BACKWARD;
          lastBackwardScan = millis();
        }
      }
    break;

    // ====================================================================
    //                           RIGHT STATE
    // ====================================================================
    case STATE_RIGHT:
      runWalkSequenceRight(pwm);

      // Continua a misurare la distanza frontale
      if (millis() - lastCheck >= 200) {
        lastCheck = millis();
        distFwd = readDistance();

        Serial.print("RIGHT - Forward distance: ");
        Serial.println(distFwd);

        // Quando diventa > 40, torna dritto
        if (distFwd > 60) {
          Serial.println("Path clear -> Going FORWARD");
          returnNeckToCenter();
          state = STATE_FORWARD;
        }
      }
    break;

    // ====================================================================
    //                           LEFT STATE
    // ====================================================================
    case STATE_LEFT:
      runWalkSequenceLeft(pwm);

      // Continua a misurare la distanza frontale
      if (millis() - lastCheck >= 200) {
        lastCheck = millis();
        distFwd = readDistance();

        Serial.print("LEFT - Forward distance: ");
        Serial.println(distFwd);

        // Quando diventa > 40, torna dritto
        if (distFwd > 60) {
          Serial.println("Path clear -> Going FORWARD");
          returnNeckToCenter();
          state = STATE_FORWARD;
        }
      }
    break;

    // ====================================================================
    //                           BACKWARD STATE
    // ====================================================================
    case STATE_BACKWARD:
      runWalkSequenceBackward(pwm);

      // Ogni tanto prova a guardare a destra e sinistra
      if (millis() - lastBackwardScan >= 2000) {
        lastBackwardScan = millis();

        Serial.println("BACKWARD - Scanning for exit...");

        // Prova a DESTRA
        moveNeckSlow(CENTER, CENTER + 80, 10, 30);
        delay(300);
        distRight = readDistance();
        Serial.print("Right distance: ");
        Serial.println(distRight);

        if (distRight > 30) {
          Serial.println("Right path found! Going RIGHT");
          returnNeckToCenter();
          state = STATE_RIGHT;
          break;
        }

        // Prova a SINISTRA
        moveNeckSlow(CENTER + 80, CENTER - 80, 16, 30);
        delay(300);
        distLeft = readDistance();
        Serial.print("Left distance: ");
        Serial.println(distLeft);

        if (distLeft > 30) {
          Serial.println("Left path found! Going LEFT");
          returnNeckToCenter();
          state = STATE_LEFT;
          break;
        }

        // Torna al centro e continua indietro
        returnNeckToCenter();
        Serial.println("No exit found, continuing BACKWARD");
      }
    break;
  }
}