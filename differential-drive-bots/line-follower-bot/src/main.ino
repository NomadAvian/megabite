/*
  Arduino Nano-based PID Line Follower
  credit: NomadAvian
  FINAL — debug prints wrapped in #if DEBUG
*/

#define DEBUG 0

#include <Arduino.h>

#if DEBUG
    #include <Wire.h>
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h>
    #define SCREEN_WIDTH 128
    #define SCREEN_HEIGHT 64
    #define DISPLAY_UPDATE_INTERVAL 100  // ms
    unsigned long lastDisplayUpdate = 0;
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif

// === Pin Definitions ===
const uint8_t SENSOR_PINS[5] = {2, 3, 4, 5, 6};     // digital I/O
const uint8_t LEFT_PWM_PIN  = 9,   // OC1A
              LEFT_DIR_PIN  = 8,
              RIGHT_PWM_PIN = 10,  // OC1B
              RIGHT_DIR_PIN = 7;

// === PID gains ===
float Kp = 90.0f,  Ki = 3.0f,  Kd = 40.0f;
// === Other constants ===
const int   BASE_PWM = 140;                      // 0-255
const float WEIGHTS[5] = {-4, -2, 0, 2, 4};      // sensor positions
const float INT_LIM    = 800;                    // integral clamp
const float LAMBDA     = 0.85f;                  // leaky-I factor
const float D_ALPHA    = 0.7f;                   // derivative LPF

// === State ===
float integral = 0, prevError = 0, dFilt = 0;
unsigned long prevMicros = 0;
float lastPos = 0;

inline float calculateError()
{
    float num = 0, den = 0;
    for (uint8_t i = 0; i < 5; ++i)
    {
        int raw = digitalRead(SENSOR_PINS[i]);   // LOW = black, HIGH = white
        int hit = (raw == LOW);
        num += hit * WEIGHTS[i];
        den += hit;
    }
    
    if (den > 0) {
        lastPos = num / den;
    } else {                // line lost
        integral = 0;       // clear wind-up
        return -lastPos*2.0;
    }
    return -lastPos;        // target position is 0
}

inline float timeDelta()
{
    unsigned long now = micros();
    float dt = (now - prevMicros) * 1e-5f; // second
    if (dt <= 0) return 0;
    prevMicros = now;
    return dt;
}

inline float PID(float error, float dt)
{
    // leaky integral
    integral = LAMBDA * integral + error * dt;
    integral = constrain(integral, -INT_LIM, INT_LIM);

    // derivative (low-pass filtered)
    float rawD = (error - prevError) / dt;
    dFilt = D_ALPHA * dFilt + (1.0f - D_ALPHA) * rawD;

    prevError = error;
    return Kp * error + Ki * integral + Kd * dFilt;
}

inline void setMotorSpeeds(float correction)
{
    int left  = BASE_PWM - correction;
    int right = BASE_PWM + correction;

    // symmetrical scaling if either side clips
    int maxPWM = max(abs(left), abs(right));
    if (maxPWM > 255) {
        float scale = 255.0f / maxPWM;
        left  *= scale;
        right *= scale;
    }

    left  = constrain(left,  0, 255);
    right = constrain(right, 0, 255);

    analogWrite(LEFT_PWM_PIN,  left);
    analogWrite(RIGHT_PWM_PIN, right);
}

void setup()
{
    for (uint8_t pin : SENSOR_PINS)
        pinMode(pin, INPUT_PULLUP);

    pinMode(LEFT_PWM_PIN,  OUTPUT);
    pinMode(LEFT_DIR_PIN,  OUTPUT);
    pinMode(RIGHT_PWM_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);

    digitalWrite(LEFT_DIR_PIN,  HIGH);   // both wheels forward
    digitalWrite(RIGHT_DIR_PIN, HIGH);

#if DEBUG
    Serial.begin(9600);
    // 0x3C is default i2c adress in some cases MAY be different
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(10, 10);
    display.display();
#endif
}

void loop()
{
    /* ----- 1. sense ----- */
    float error = calculateError();

    /* ----- 2. timing ----- */
    float dt = timeDelta();
    if (dt == 0) return;

    /* ----- 3. control ----- */
    float correction = PID(error, dt);

#if DEBUG
    unsigned long now = millis();
    if (now - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = now;
    display.clearDisplay();
    display.setCursor(10, 10);
    for (int i = 0; i < 5; ++i) {
        display.print(digitalRead(SENSOR_PINS[i]) == LOW ? " 0 " : " 1 ");
    }
    display.display();
}
#endif

    /* ----- 4. actuate ----- */
    setMotorSpeeds(correction);
}