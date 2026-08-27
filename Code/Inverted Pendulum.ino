/* Single Inverted Pendulum using an LQR controller
 *
 * Hardware:
 *   - Cart driven by a brushed DC motor through an BTS7960 H-bridge driver
 *   - Cart position from a quadrature encoder
 *   - Pendulum angle from an AS5600 magnetic encoder behind a TCA9548A I2C mux
 *
 * Control:
 *   - Full-state feedback (LQR gains K) on [x, theta, x_dot, theta_dot]
 *   - Runs on a fixed timestep via micros() in loop()
 *   - Auto-arms once the pendulum is held near upright and still
 *   - Auto-disarms (and cuts the motor) if the pendulum falls past FALL_LIMIT
 *
 * Serial commands:
 *   k -> kill / disarm
 *   space -> manual disarm
 *   p -> print current cart encoder count
 *   l -> start a fixed-length angle log, dumped to Serial when full
 */

#include <Wire.h>

// I2C / angle sensor
#define MUX_ADDR 0x70   // TCA9548A I2C multiplexer
#define AS5600_ADDR 0x36
#define AS5600_ANGLE_H 0x0E   // raw angle, high byte

const int PENDULUM_MUX_CHANNEL = 2;      // mux channel the pendulum's AS5600 is on
const int PENDULUM_ZERO_RAW    = 258;    // raw AS5600 count corresponding to "hanging down"
const float PENDULUM_SIGN        = 1.0;    // flip if angle increases the wrong way

// Motor driver (IBT-2 style H-bridge) + cart encoder
const int RPWM = 25, LPWM = 26, R_EN = 27, L_EN = 14;
const int ENC_A = 18, ENC_B = 19;

volatile long cartCount = 0;

// Cart geometry / unit conversion
const float CART_SIGN            = -1.0;               // flip if +x is the wrong direction
const float CART_TRAVEL_METERS   = 0.40;                // full rail travel
const float CART_TRAVEL_COUNTS   = 21028.0;              // encoder counts over that travel
const float METERS_PER_COUNT     = CART_TRAVEL_METERS / CART_TRAVEL_COUNTS;
const float RAD_PER_COUNT        = (2.0 * PI) / 4096.0;   // AS5600 is a 12-bit sensor (4096 counts/rev)

// Soft travel limits (set relative to power-on position)
long CART_MIN_COUNT, CART_MAX_COUNT;
const long  CART_SAFE_MARGIN = 8000;

// Arming / safety thresholds
const float ARM_ANGLE  = 0.10;  // rad — must be within this of upright to auto-arm
const float ARM_VEL    = 0.5;   // rad/s — must be this slow to auto-arm
const float FALL_LIMIT = 0.6;   // rad — auto-disarm past this

// LQR gains: force = -(K0*x + K1*theta + K2*x_dot + K3*theta_dot)
const float K[4] = {-6, -9.0507, -10, -1.2490}; // [x, theta, x_dot, theta_dot]

// Force -> PWM mapping
const int MAX_PWM      = 230;
const int DEADBAND_PWM = 50;    // minimum PWM needed to actually move the cart
const float FORCE_EPS    = 0.01;  // below this, just stop rather than dither
const float FORCE_TO_PWM = 150.0;

// Control loop timing
const unsigned long CONTROL_PERIOD_US = 4000; // 250 Hz control loop

// Velocity estimation (windowed finite difference on x and theta)
const int VEL_WINDOW = 7;
float xBuf[VEL_WINDOW], thBuf[VEL_WINDOW], tBuf[VEL_WINDOW];
int velIdx = 0;
bool velBufFull = false;

// Angle logger (for tuning / plotting)
#define LOG_SIZE 2000
float logBuf[LOG_SIZE];
unsigned long logTime[LOG_SIZE];
int logIndex = 0;
bool logging = false;

// State
bool  armed = false;
float x, theta;
float dx = 0, dtheta = 0;
float x_prev = 0, theta_prev = 0;
unsigned long t_prev_ctrl;

// Diagnostics reported once per second
unsigned long loopCounter = 0, lastRateReport = 0;
float measuredHz = 0;
unsigned long badReads = 0, muxFails = 0;

// Encoder ISR
void IRAM_ATTR handleCartEncoderA() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) cartCount++;
  else cartCount--;
}

// Motor helpers
void stopMotor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

// Push the cart with a signed force, respecting soft limits and deadband.
void applyForce(float force) {
  if (cartCount <= CART_MIN_COUNT && force > 0) { stopMotor(); return; }
  if (cartCount >= CART_MAX_COUNT && force < 0) { stopMotor(); return; }
  if (fabs(force) < FORCE_EPS) { stopMotor(); return; }

  int pwm = DEADBAND_PWM + (int)(fabs(force) * FORCE_TO_PWM);
  if (pwm > MAX_PWM) pwm = MAX_PWM;

  if (force > 0) { analogWrite(LPWM, 0);   analogWrite(RPWM, pwm); }
  else { analogWrite(RPWM, 0);   analogWrite(LPWM, pwm); }
}

// AS5600 helpers
bool muxSelect(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  uint8_t result = Wire.endTransmission();
  if (result != 0) { muxFails++; return false; }
  return true;
}

uint16_t readRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_ANGLE_H);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF; // sentinel: read failed
  uint16_t hi = Wire.read(), lo = Wire.read();
  return ((hi << 8) | lo) & 0x0FFF;
}

// Convert a raw AS5600 count to a signed angle in (-pi, pi], relative to zeroRaw.
float rawToAngle(uint16_t raw, int zeroRaw, float sign) {
  float a = (float)(raw - zeroRaw) * RAD_PER_COUNT + PI;
  while (a > PI)  a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return sign * a;
}

// Control loop — called once per CONTROL_PERIOD_US (4000/250Hz)
void controlStep(float dt) {
  // Read state
  x = CART_SIGN * (float)cartCount * METERS_PER_COUNT;

  bool pendulumOk = muxSelect(PENDULUM_MUX_CHANNEL);
  uint16_t raw = pendulumOk ? readRawAngle() : 0xFFFF;
  if (raw == 0xFFFF) badReads++;
  else theta = rawToAngle(raw, PENDULUM_ZERO_RAW, PENDULUM_SIGN);

  //  Estimate velocities over a sliding window
  static float tAccum = 0;
  tAccum += dt;
  xBuf[velIdx]  = x;
  thBuf[velIdx] = theta;
  tBuf[velIdx]  = tAccum;
  int newest = velIdx;
  velIdx = (velIdx + 1) % VEL_WINDOW;
  if (velIdx == 0) velBufFull = true;

  if (velBufFull) {
    int oldest = velIdx;
    float windowDt = tBuf[newest] - tBuf[oldest];
    if (windowDt > 1e-4) {
      dx     = (xBuf[newest]  - xBuf[oldest])  / windowDt;
      dtheta = (thBuf[newest] - thBuf[oldest]) / windowDt;
    }
  } else {
    // Not enough samples yet
    dx     = (x     - x_prev)     / dt;
    dtheta = (theta - theta_prev) / dt;
  }
  x_prev = x;
  theta_prev = theta;

  //Disarm on fall
  if (armed && fabs(theta) > FALL_LIMIT) {
    armed = false;
    stopMotor();
    Serial.println("FELL - disarmed");
    return;
  }

  // --- LQR control ---
  if (armed) {
    float force = -(K[0] * x + K[1] * theta + K[2] * dx + K[3] * dtheta);
    applyForce(force);
  } else {
    stopMotor();
  }
}

// Logging
void dumpLog() {
  Serial.println("---LOG START---");
  for (int i = 0; i < logIndex; i++) {
    Serial.print(logTime[i]);
    Serial.print(",");
    Serial.println(logBuf[i], 4);
  }
  Serial.println("---LOG END---");
}

// Serial command handling
void handleSerialCommands() {
  if (!Serial.available()) return;

  char c = Serial.read();
  switch (c) {
    case 'k':
      armed = false;
      stopMotor();
      Serial.println("KILLED");
      break;
    case ' ':
      armed = false;
      stopMotor();
      Serial.println("manual disarm");
      break;
    case 'p':
      Serial.print("cartCount = ");
      Serial.println(cartCount);
      break;
    case 'l':
      logIndex = 0;
      logging = true;
      Serial.println("LOGGING STARTED");
      break;
  }
}

// Setup / loop
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  stopMotor();

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), handleCartEncoderA, CHANGE);
  delay(500);

  // NOTE: this build skips automatic homing —> center the cart by hand
  // before power-on, and treat that position as the zero reference.
  Serial.println("Manual mode: no homing. Center the cart by hand before powering on.");
  cartCount = 0;
  CART_MIN_COUNT = cartCount - CART_SAFE_MARGIN;
  CART_MAX_COUNT = cartCount + CART_SAFE_MARGIN;
  Serial.print("Soft limits set: ");
  Serial.print(CART_MIN_COUNT);
  Serial.print(" to ");
  Serial.println(CART_MAX_COUNT);
  delay(500);

  t_prev_ctrl = micros();
  lastRateReport = millis();
  Serial.println("Hold pendulum UP. Space=ARM, k=KILL.");
}

void loop() {
  handleSerialCommands();

  unsigned long now = micros();
  if (now - t_prev_ctrl >= CONTROL_PERIOD_US) {
    float dt = (now - t_prev_ctrl) * 1e-6;
    t_prev_ctrl = now;
    controlStep(dt);
    loopCounter++;

    if (!armed && fabs(theta) < ARM_ANGLE && fabs(dtheta) < ARM_VEL) {
      armed = true;
      Serial.println(">>> AUTO-ARMED <<<");
    }

    if (logging && logIndex < LOG_SIZE) {
      logTime[logIndex] = now;
      logBuf[logIndex]  = theta;
      logIndex++;
    } else if (logging && logIndex >= LOG_SIZE) {
      logging = false;
      dumpLog();
    }
  }

  // Once-per-second diagnostics
  if (millis() - lastRateReport >= 1000) {
    measuredHz = loopCounter;
    loopCounter = 0;
    lastRateReport = millis();

    Serial.print("rate=");     Serial.print(measuredHz, 0);
    Serial.print(" theta=");   Serial.print(theta, 2);
    Serial.print(" badReads="); Serial.print(badReads);
    Serial.print(" muxFails="); Serial.print(muxFails);
    Serial.println(armed ? " [ARM]" : " [off]");
  }
}
