/*
    Arduino Nano-based PID Line Follower
    credit: NomadAvian
*/

#include <Arduino.h>

// === Pin Definitions ===
const int SENSOR_PINS[5] = {2, 3, 4, 5, 6}; // digital I/O
const int LEFT_PWM_PIN   = 9,  // OC1A
          LEFT_DIR_PIN   = 8,
          RIGHT_PWM_PIN  = 10, // OC1B
          RIGHT_DIR_PIN  = 7;

// === PID gains ===
float Kp                 = 25.0f,
      Ki                 = 0.0f,
      Kd                 = 6.0f;

// === Other Consts ===
const int   BASE_PWM     = 130; // forward speed 0‑255
const float WEIGHTS[5]   = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
const float INT_LIM      = 800; // absolute cap on integral term
const float LAMBDA       = 0.85f;

// === State ===
float integral           = 0;
float prevError          = 0;
unsigned long prevMillis = 0;

void setup() {
  // sensor pins
    for(int pin : SENSOR_PINS)
        pinMode(pin, INPUT_PULLUP);
    
  // motor pins
  pinMode(LEFT_PWM_PIN,  OUTPUT);
  pinMode(LEFT_DIR_PIN,  OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  // set both wheels forward
  digitalWrite(LEFT_DIR_PIN,  HIGH);
  digitalWrite(RIGHT_DIR_PIN, HIGH);

  Serial.begin(9600);
}

float calculateError() {
  float numerator = 0, denominator = 0;
  for(int i = 0; i < 5 ++i) {
    int raw = digitalRead(SENSOR_PINS[i]);

    Serial.print(raw); Serial.print(" ");

    int sum = (raw == LOW) ? 1 : 0;
    numerator += sum * WEIGHTS[i];
    denominator += sum;
  }

  Serial.println(" ");

  static float lastPos = 0;
  float pos;
  if(denominator > 0) {
    pos = numerator / denominator;
    lastPos = pos;
  } else {
    // line lost : fall back to last seen position
    pos = lastPos;
  }

  // desired line position = 0
  float error = -pos

  return error;
}

float timeDelta() {
  unsigned long now = millis();
  float dt = (now - prevMillis) / 1000.0f;
  if(dt <= 0) return 0.0f; // guard
  prevMillis = now;
  return dt;
}

float PID(float error, float dt) {
  // decay + accumulate : prioritize recent errors
  integral = LAMBDA*integral + error*dt;
  integral = constrain(integral, -INT_LIM, INT_LIM);

  float derivative = (error - prevError) / dt;
  float correction = Kp*error + Ki*integral + Kd*derivative;

  // set current error to previousError before next iteration
  prevError = error;

  return correction;
}

void setMotorSpeeds(float correction) {
  // adjust motor speeds
  int leftPWM  = BASE_PWM - corrrection;
  int rightPWM = BASE_PWM + correction;

  // constrain values to usable bounds
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  // set motor speeds (forward-only)
  analogWrite(LEFT_PWM_PIN, leftPWM);
  analogWrite(RIGHT_PWM_PIN, rightPWM);
}

void loop() {
  
  /*
    1. Read sensors & compute line position (weighted average)
  */
  float error = calculateError();
  
  /*
    2. Time delta
  */
  float dt = timeDelta();
  if(dt == 0.0f) return;

  /*
    3. PID with leaky integrator
  */
  float correction = PID(error, dt);

  /*
    4. Motor mixing
  */
  setMotorSpeeds(correction);

}