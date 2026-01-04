// ---------------- JOYSTICK PINS ----------------
#define VRX_PIN 34   // Joystick X axis
#define VRY_PIN 35   // Joystick Y axis
#define SW_PIN  32   // Joystick button

// ---------------- LEFT MOTOR (BTS7960 #1) ----------------
#define L_RPWM 14
#define L_LPWM 13

// ---------------- RIGHT MOTOR (BTS7960 #2) ----------------
#define R_RPWM 26
#define R_LPWM 25

void setup() {
  Serial.begin(115200);

  pinMode(SW_PIN, INPUT_PULLUP);

  // Motor pins
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);

  stopMotors();
}

void loop() {
  int xValue = analogRead(VRX_PIN);   // 0–4095
  int yValue = analogRead(VRY_PIN);   // 0–4095

  Serial.print("X: "); Serial.print(xValue);
  Serial.print("  Y: "); Serial.println(yValue);

  // ---- DEAD ZONE ----
  if (xValue > 1600 && xValue < 2400 && yValue > 1600 && yValue < 2400) {
    stopMotors();
    return;
  }

  // ---- FORWARD ----
  if (yValue > 2600 && xValue > 1600 && xValue < 2400) {
    forward();
    return;
  }

  // ---- BACKWARD ----
  if (yValue < 1400 && xValue > 1600 && xValue < 2400) {
    backward();
    return;
  }

  // ---- RIGHT ----
  if (xValue > 2600) {
    turnRight();
    return;
  }

  // ---- LEFT ----
  if (xValue < 1400) {
    turnLeft();
    return;
  }
}

// ---------------- MOTOR FUNCTIONS ----------------

void forward() {
  analogWrite(L_RPWM, 200);
  analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, 200);
  analogWrite(R_LPWM, 0);
}

void backward() {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 200);
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 200);
}

void turnRight() {
  analogWrite(L_RPWM, 200);
  analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 200);
}// ---------------- JOYSTICK PINS ----------------
#define VRX_PIN 34   // Joystick X axis
#define VRY_PIN 35   // Joystick Y axis
#define SW_PIN  32   // Joystick button

// ---------------- LEFT MOTOR (BTS7960 #1) ----------------
#define L_RPWM 14
#define L_LPWM 13

// ---------------- RIGHT MOTOR (BTS7960 #2) ----------------
#define R_RPWM 26
#define R_LPWM 25

// PWM channels (ESP32 needs channels)
#define CH_L_RPWM 0
#define CH_L_LPWM 1
#define CH_R_RPWM 2
#define CH_R_LPWM 3

// PWM frequency & resolution
#define PWM_FREQ 20000      // 20 kHz (silent for motors)
#define PWM_RESOLUTION 8    // 0–255

void setup() {
  Serial.begin(115200);

  pinMode(SW_PIN, INPUT_PULLUP);

  // Attach PWM channels to pins
  ledcAttachPin(L_RPWM, CH_L_RPWM);
  ledcAttachPin(L_LPWM, CH_L_LPWM);
  ledcAttachPin(R_RPWM, CH_R_RPWM);
  ledcAttachPin(R_LPWM, CH_R_LPWM);

  ledcSetup(CH_L_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_L_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_R_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_R_LPWM, PWM_FREQ, PWM_RESOLUTION);

  stopMotors();
}

void loop() {
  int xValue = analogRead(VRX_PIN);   
  int yValue = analogRead(VRY_PIN);

  Serial.print("X: "); Serial.print(xValue);
  Serial.print("  Y: "); Serial.println(yValue);

  // DEAD ZONE
  if (xValue > 1600 && xValue < 2400 && yValue > 1600 && yValue < 2400) {
    stopMotors();
    return;
  }

  // FORWARD
  if (yValue > 2600 && xValue > 1600 && xValue < 2400) {
    forward();
    return;
  }

  // BACKWARD
  if (yValue < 1400 && xValue > 1600 && xValue < 2400) {
    backward();
    return;
  }

  // RIGHT
  if (xValue > 2600) {
    turnRight();
    return;
  }

  // LEFT
  if (xValue < 1400) {
    turnLeft();
    return;
  }
}

// ---------------- MOTOR FUNCTIONS ----------------

void forward() {
  setMotor(L_RPWM, CH_L_RPWM, L_LPWM, CH_L_LPWM, 180);
  setMotor(R_RPWM, CH_R_RPWM, R_LPWM, CH_R_LPWM, 180);
}

void backward() {
  setMotor(L_LPWM, CH_L_LPWM, L_RPWM, CH_L_RPWM, 180);
  setMotor(R_LPWM, CH_R_LPWM, R_RPWM, CH_R_RPWM, 180);
}

void turnRight() {
  setMotor(L_RPWM, CH_L_RPWM, L_LPWM, CH_L_LPWM, 180);
  setMotor(R_LPWM, CH_R_LPWM, R_RPWM, CH_R_RPWM, 180);
}

void turnLeft() {
  setMotor(L_LPWM, CH_L_LPWM, L_RPWM, CH_L_RPWM, 180);
  setMotor(R_RPWM, CH_R_RPWM, R_LPWM, CH_R_LPWM, 180);
}

void stopMotors() {
  ledcWrite(CH_L_RPWM, 0);
  ledcWrite(CH_L_LPWM, 0);
  ledcWrite(CH_R_RPWM, 0);
  ledcWrite(CH_R_LPWM, 0);
}

void setMotor(int forwardPin, int forwardCh, int backwardPin, int backwardCh, int speedValue) {
  ledcWrite(forwardCh, speedValue);
  ledcWrite(backwardCh, 0);
}


void turnLeft() {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 200);
  analogWrite(R_RPWM, 200);
  analogWrite(R_LPWM, 0);
}

void stopMotors() {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);
}
