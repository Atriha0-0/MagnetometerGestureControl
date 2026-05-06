/*
  ============================================================
  ARDUINO UNO — Robot Receiver
  ============================================================
  Bluetooth : HC-05 SLAVE on SoftwareSerial
                pin 11 = RX (connect to HC-05 TX)
                pin 12 = TX (connect to HC-05 RX via voltage divider)
  Motors    : L298N — ENA=5, IN1=2, IN2=3, ENB=6, IN3=4, IN4=7
  Servos    : Tilt up/down = pin 9 | Pan left/right = pin 10
  IMU       : MPU6050 on I2C (SDA=A4, SCL=A5)

  Commands received from Nano:
    "F"  → move forward  (MOVE_DURATION ms, yaw-corrected)
    "B"  → move backward (MOVE_DURATION ms, yaw-corrected)
    "L"  → snap turn left  90° using MPU6050
    "R"  → snap turn right 90° using MPU6050
    "SU" → tilt servo UP
    "SD" → tilt servo DOWN
    "SL" → pan servo LEFT
    "SR" → pan servo RIGHT
    "X"  → stop motors + center servos

  WIRING NOTE — voltage divider on HC-05 RX line:
    Uno pin 12 → 1kΩ → HC-05 RX pin
                        2kΩ → GND
    (brings 5V down to ~3.3V to protect HC-05)
  ============================================================
*/

#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ──────────────────────────────────────────
//  TEST MODE
//  Set to 1 to test via Serial Monitor (no Bluetooth needed)
//  Set to 0 for real Bluetooth operation
// ──────────────────────────────────────────
#define TEST_MODE 0

// ──────────────────────────────────────────
//  Bluetooth
// ──────────────────────────────────────────
SoftwareSerial BT(11, 12);  // RX=11, TX=12

// ──────────────────────────────────────────
//  MPU6050
// ──────────────────────────────────────────
MPU6050 mpu(Wire);

// ──────────────────────────────────────────
//  Motor pins  (exactly as your original)
// ──────────────────────────────────────────
#define ENA 5
#define IN1 2
#define IN2 3
#define ENB 6
#define IN3 4
#define IN4 7

// ──────────────────────────────────────────
//  Servos
// ──────────────────────────────────────────
Servo tiltServo;  // up/down  on pin 9
Servo panServo;   // left/right on pin 10

const int TILT_UP     = 60;
const int TILT_DOWN   = 120;
const int TILT_CENTER = 90;
const int PAN_LEFT    = 135;
const int PAN_RIGHT   = 45;
const int PAN_CENTER  = 90;

// ──────────────────────────────────────────
//  Speed — your tuned values, kept exactly
// ──────────────────────────────────────────
int baseA      = 132;   // RIGHT motor (ENA)
int baseB      = 140;   // LEFT  motor (ENB)
int baseA_back = 135;
int baseB_back = 140;

// How long F and B commands run before stopping
const unsigned long MOVE_DURATION = 800;  // ms — tune as needed

// ──────────────────────────────────────────
//  PID / yaw — your tuned values, kept exactly
// ──────────────────────────────────────────
float Kp      = 0.42;
float Kp_back = 0.50;

float targetYaw        = 0;
float filteredYaw      = 0;
float filteredYaw_back = 0;

// Snap turn parameters
const float TURN_ANGLE    = 90.0;  // degrees per L/R gesture
const float TURN_SPEED    = 120.0; // max PWM during snap turn (reduced)
const float TURN_DEADBAND = 3.0;   // stop when within this many degrees
const float TURN_MIN_PWM  = 60.0;  // minimum PWM (lower = less overshoot)

// ──────────────────────────────────────────
//  Angle wrap  (your original function)
// ──────────────────────────────────────────
float wrapAngle(float angle) {
  while (angle >  180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// ──────────────────────────────────────────
//  setMotors  (your original — untouched)
// ──────────────────────────────────────────
void setMotors(int right, int left) {
  if (right > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (right < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    right = -right;
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  if (left > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (left < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    left = -left;
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, constrain(left,  0, 255));
  analogWrite(ENB, constrain(right, 0, 255));
}

void stopMotors() {
  setMotors(0, 0);
}

// ──────────────────────────────────────────
//  GoForwardP  (your logic + fixed-duration
//               internal loop added)
// ──────────────────────────────────────────
void GoForwardP() {
  Serial.println(">> FORWARD");
  unsigned long start = millis();

  while (millis() - start < MOVE_DURATION) {
    mpu.update();
    float currentYaw = mpu.getAngleZ();

    filteredYaw = 0.9 * filteredYaw + 0.1 * currentYaw;

    float error      = wrapAngle(targetYaw - filteredYaw);
    float correction = constrain(Kp * error, -30, 30);

    int leftSpeed  = baseA + correction;
    int rightSpeed = baseB - correction;

    setMotors(rightSpeed, leftSpeed);

    Serial.print("Yaw: ");    Serial.print(filteredYaw);
    Serial.print(" | Err: "); Serial.print(error);
    Serial.print(" | Cor: "); Serial.println(correction);

    delay(10);
  }

  stopMotors();
}

// ──────────────────────────────────────────
//  GoBackwardP  (your logic + fixed-duration
//                internal loop added)
// ──────────────────────────────────────────
void GoBackwardP() {
  Serial.println(">> BACKWARD");
  unsigned long start = millis();

  while (millis() - start < MOVE_DURATION) {
    mpu.update();
    float currentYaw = mpu.getAngleZ();

    filteredYaw_back = 0.9 * filteredYaw_back + 0.1 * currentYaw;

    float error      = wrapAngle(targetYaw - filteredYaw_back);
    float correction = constrain(Kp_back * error, -30, 30);

    int leftSpeed  = -baseA_back + (int)correction;
    int rightSpeed = -baseB_back - (int)correction;

    setMotors(rightSpeed, leftSpeed);

    Serial.print("Back Yaw: "); Serial.print(filteredYaw_back);
    Serial.print(" | Err: ");   Serial.print(error);
    Serial.print(" | Cor: ");   Serial.println(correction);

    delay(10);
  }

  stopMotors();
}

// ──────────────────────────────────────────
//  Snap turn using MPU6050
//  direction: +1 = right, -1 = left
//  Rotates exactly TURN_ANGLE degrees then
//  locks the new heading as targetYaw so that
//  GoForwardP/GoBackwardP stay straight on the
//  new direction automatically
// ──────────────────────────────────────────
void snapTurn(int direction) {
  Serial.println(direction > 0 ? ">> TURN RIGHT" : ">> TURN LEFT");

  mpu.update();
  float goalYaw = wrapAngle(mpu.getAngleZ() + direction * TURN_ANGLE);

  unsigned long timeout = millis() + 3000;  // 3s safety cutoff

  while (millis() < timeout) {
    mpu.update();
    float currentYaw = mpu.getAngleZ();
    float error      = wrapAngle(goalYaw - currentYaw);

    Serial.print("TurnErr="); Serial.println(error, 1);

    if (abs(error) <= TURN_DEADBAND) break;

    // Scale PWM with error — slows as it approaches target
    float turnPWM = constrain(abs(error) * 2.0, TURN_MIN_PWM, TURN_SPEED);

    if (error > 0) {
      setMotors(-turnPWM,  turnPWM);  // turn right
    } else {
      setMotors( turnPWM, -turnPWM);  // turn left
    }
    delay(10);
  }

  stopMotors();
  delay(100);  // brief settle before reading final yaw

  // Update target so F/B now lock onto new heading
  mpu.update();
  targetYaw        = mpu.getAngleZ();
  filteredYaw      = targetYaw;
  filteredYaw_back = targetYaw;

  Serial.print("Heading locked: "); Serial.println(targetYaw, 1);
}

// ──────────────────────────────────────────
//  Servo functions  (your originals, kept exactly)
// ──────────────────────────────────────────
void servoup()    { tiltServo.write(TILT_UP);    Serial.println("Servo: UP");    }
void servodown()  { tiltServo.write(TILT_DOWN);  Serial.println("Servo: DOWN");  }
void servoleft()  { panServo.write(PAN_LEFT);    Serial.println("Servo: LEFT");  }
void servoright() { panServo.write(PAN_RIGHT);   Serial.println("Servo: RIGHT"); }
void servocenter(){ tiltServo.write(TILT_CENTER);
                    panServo.write(PAN_CENTER); }

// ──────────────────────────────────────────
//  Command dispatcher
// ──────────────────────────────────────────
void handleCommand(String cmd) {
  // Strip ALL whitespace and control characters aggressively
  cmd.trim();
  cmd.replace("\r", "");
  cmd.replace("\n", "");
  cmd.replace(" ", "");
  if (cmd.length() == 0) return;

  // Print raw bytes for debugging so we can see hidden characters
  Serial.print("CMD [");
  for (int i = 0; i < cmd.length(); i++) {
    Serial.print((int)cmd[i]);
    Serial.print(" ");
  }
  Serial.print("] = ");
  Serial.println(cmd);

  // Use startsWith instead of == for robustness against trailing chars
  if      (cmd.startsWith("SU")) servoup();
  else if (cmd.startsWith("SD")) servodown();
  else if (cmd.startsWith("SL")) servoleft();
  else if (cmd.startsWith("SR")) servoright();
  else if (cmd.startsWith("F"))  GoForwardP();
  else if (cmd.startsWith("B"))  GoBackwardP();
  else if (cmd.startsWith("L"))  snapTurn(-1);
  else if (cmd.startsWith("R"))  snapTurn(+1);
  else if (cmd.startsWith("X"))  { stopMotors(); servocenter(); }
  else {
    Serial.print("Unknown cmd: "); Serial.println(cmd);
  }
}

// ──────────────────────────────────────────
//  Setup
// ──────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  BT.begin(9600);
  Wire.begin();

  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  // Servos
  tiltServo.attach(9);
  panServo.attach(10);
  servocenter();

  // MPU6050
  mpu.begin();
  Serial.println("Calibrating MPU6050... do not move robot.");
  mpu.calcOffsets(true, true);

  Serial.println("Place robot still for yaw calibration...");
  delay(3000);

  float sum = 0;
  for (int i = 0; i < 100; i++) {
    mpu.update();
    sum += mpu.getAngleZ();
    delay(10);
  }
  targetYaw        = sum / 100.0;
  filteredYaw      = targetYaw;
  filteredYaw_back = targetYaw;

  Serial.print("Target yaw locked: "); Serial.println(targetYaw);
  Serial.println("Ready. Waiting for Bluetooth commands.");
}

// ──────────────────────────────────────────
//  Loop
// ──────────────────────────────────────────
void loop() {
#if TEST_MODE
  // TEST MODE: read commands from Serial Monitor
  // Type F, B, L, R, SU, SD, SL, SR, X and press Enter
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
#else
  // NORMAL MODE: read commands from Bluetooth
  if (BT.available()) {
    String cmd = BT.readStringUntil('\n');
    handleCommand(cmd);
  }
#endif
}