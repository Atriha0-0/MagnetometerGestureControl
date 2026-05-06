/*
  ============================================================
  ARDUINO NANO — Gesture Transmitter
  Sensor   : HMC5883L magnetometer
  Bluetooth: HC-05 Master on SoftwareSerial pins 2(RX) 3(TX)

  Commands sent to Uno over Bluetooth:
    "F"  = Forward    "B"  = Backward
    "L"  = Turn Left  "R"  = Turn Right
    "SU" = Servo Up   "SD" = Servo Down
    "SL" = Servo Left "SR" = Servo Right
    "X"  = Stop
  ============================================================
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include <math.h>

SoftwareSerial BT(2, 3);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

const float STOP_FAR  = 15.0;  // too far — stop
const float ROVER_MAX = 67.0;  // rover zone: 15-50
// No gap zone — servo starts immediately after rover
const float SERVO_MIN = 70.0;  // servo zone: 50+
const float ALPHA_FAST = 0.35;

// ══════════════════════════════════════════
//  ROVER PARAMETERS — DO NOT TOUCH
// ══════════════════════════════════════════
const float          R_GESTURE_THRESH   = 6.0;
const float          R_COMMIT_THRESH    = 12.0;
const unsigned long  R_STABLE_MS        = 300;
const unsigned long  R_MAX_WINDOW       = 600;
const float          R_DOMINANCE_RATIO  = 1.3;
const unsigned long  R_COOLDOWN_MS      = 500;
const float          R_NEUTRAL_THRESH   = 3.0;

// ══════════════════════════════════════════
//  SERVO PARAMETERS — completely separate
// ══════════════════════════════════════════
const float          S_GESTURE_THRESH   = 8.0;   // calmer field at medium distance
const float          S_COMMIT_THRESH    = 15.0;  // easier to trigger now
const unsigned long  S_STABLE_MS        = 400;
const unsigned long  S_PEAK_STABLE_MS   = 200;
const unsigned long  S_MAX_WINDOW       = 700;
const float          S_DOMINANCE_RATIO  = 1.4;
const unsigned long  S_COOLDOWN_MS      = 600;
const float          S_NEUTRAL_THRESH   = 4.0;

float fx = 0, fy = 0, fz = 0;
float baseX = 0, baseY = 0, baseZ = 0;

// ══════════════════════════════════════════
//  ROVER STATE
// ══════════════════════════════════════════
enum GestureState { IDLE, WAITING_COMMIT, COOLDOWN_STATE };
GestureState rState = IDLE;
float rRefX = 0, rRefY = 0;
float rPeakDX = 0, rPeakDY = 0;
float rLastPeakMag = 0;
unsigned long rStableStart = 0;
unsigned long rWindowStart = 0;
unsigned long rCooldownStart = 0;
bool rPeakStable = false;
bool rIsStable = false;

// ══════════════════════════════════════════
//  SERVO STATE
// ══════════════════════════════════════════
GestureState sState = IDLE;
float sRefX = 0, sRefY = 0;
float sPeakDX = 0, sPeakDY = 0;
float sLastPeakMag = 0;
unsigned long sNeutralStart   = 0;
unsigned long sPeakStableStart = 0;
unsigned long sWindowStart    = 0;
unsigned long sCooldownStart  = 0;
bool sPeakStable = false;
bool sIsStable   = false;

String directionFromPeak(float pdx, float pdy, float domRatio) {
  float ax = fabsf(pdx);
  float ay = fabsf(pdy);
  bool xWins = (ax > ay * domRatio);
  bool yWins = (ay > ax * domRatio);
  if (!xWins && !yWins) {
    if (ax >= ay) xWins = true;
    else          yWins = true;
  }
  if (xWins) return (pdx < 0) ? "RIGHT" : "LEFT";
  else       return (pdy < 0) ? "FORWARD" : "BACKWARD";
}

void sendCommand(const String &cmd) {
  BT.println(cmd);
  Serial.print("BT SENT: "); Serial.println(cmd);
}

void setup() {
  Serial.begin(115200);
  BT.begin(9600);
  Wire.begin();
  if (!mag.begin()) {
    Serial.println("ERROR: HMC5883L not found.");
    while (1);
  }
  mag.setMagGain(HMC5883_MAGGAIN_8_1);
  Serial.println("Warming up... hold magnet still.");
  for (int i = 0; i < 100; i++) {
    sensors_event_t e; mag.getEvent(&e);
    fx = ALPHA_FAST * e.magnetic.x + (1.0f - ALPHA_FAST) * fx;
    fy = ALPHA_FAST * e.magnetic.y + (1.0f - ALPHA_FAST) * fy;
    fz = ALPHA_FAST * e.magnetic.z + (1.0f - ALPHA_FAST) * fz;
    delay(12);
  }
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 100; i++) {
    sensors_event_t e; mag.getEvent(&e);
    sx += e.magnetic.x; sy += e.magnetic.y; sz += e.magnetic.z;
    delay(10);
  }
  baseX = sx / 100.0f;
  baseY = sy / 100.0f;
  baseZ = sz / 100.0f;
  Serial.println("Calibrated. Ready.");
  Serial.println("STOP<15 | ROVER 15-50 | SERVO>50");
}

// ══════════════════════════════════════════
//  ROVER HANDLER — DO NOT TOUCH
// ══════════════════════════════════════════
void handleRover(float dx, float dy) {
  float gdx  = fx - (baseX + rRefX);
  float gdy  = fy - (baseY + rRefY);
  float gMag = sqrtf(gdx*gdx + gdy*gdy);

  if (gMag < R_NEUTRAL_THRESH) {
    if (rStableStart == 0) rStableStart = millis();
    if (millis() - rStableStart > R_STABLE_MS) rIsStable = true;
  } else {
    rStableStart = 0;
    rIsStable = false;
  }

  switch (rState) {
    case IDLE: {
      if (gMag < R_GESTURE_THRESH) {
        if (rIsStable) { rRefX = dx; rRefY = dy; }
      } else {
        rPeakDX = gdx; rPeakDY = gdy;
        rLastPeakMag = sqrtf(gdx*gdx + gdy*gdy);
        rStableStart = millis();
        rPeakStable  = false;
        rWindowStart = millis();
        rState = WAITING_COMMIT;
      }
      break;
    }
    case WAITING_COMMIT: {
      if (fabsf(gdx) > fabsf(rPeakDX)) rPeakDX = gdx;
      if (fabsf(gdy) > fabsf(rPeakDY)) rPeakDY = gdy;
      float peakMag = sqrtf(rPeakDX*rPeakDX + rPeakDY*rPeakDY);
      if (peakMag > rLastPeakMag + 0.5f) {
        rLastPeakMag = peakMag; rStableStart = millis(); rPeakStable = false;
      } else if (!rPeakStable) {
        if (millis() - rStableStart >= R_STABLE_MS) rPeakStable = true;
      }
      bool timedOut = (millis() - rWindowStart >= R_MAX_WINDOW);
      if (rPeakStable || timedOut) {
        if (peakMag >= R_COMMIT_THRESH) {
          String dir = directionFromPeak(rPeakDX, rPeakDY, R_DOMINANCE_RATIO);
          if      (dir == "FORWARD")  sendCommand("F");
          else if (dir == "BACKWARD") sendCommand("B");
          else if (dir == "LEFT")     sendCommand("L");
          else if (dir == "RIGHT")    sendCommand("R");
          Serial.println();
          Serial.println("==============================");
          Serial.print("  >>  ROVER: "); Serial.println(dir);
          Serial.println("==============================");
          Serial.println();
        }
        rRefX = dx; rRefY = dy;
        rLastPeakMag = 0; rPeakStable = false;
        rIsStable = false; rStableStart = 0;
        rCooldownStart = millis();
        rState = COOLDOWN_STATE;
      }
      break;
    }
    case COOLDOWN_STATE:
      if (millis() - rCooldownStart >= R_COOLDOWN_MS) {
        rRefX = dx; rRefY = dy;
        rIsStable = false; rStableStart = 0;
        rState = IDLE;
      }
      break;
  }
}

// ══════════════════════════════════════════
//  SERVO HANDLER
// ══════════════════════════════════════════
void handleServo(float dx, float dy) {
  float gdx  = fx - (baseX + sRefX);
  float gdy  = fy - (baseY + sRefY);
  float gMag = sqrtf(gdx*gdx + gdy*gdy);

  // Neutral detection — completely separate timers from rover
  if (gMag < S_NEUTRAL_THRESH) {
    if (sNeutralStart == 0) sNeutralStart = millis();
    if (millis() - sNeutralStart > S_STABLE_MS) sIsStable = true;
  } else {
    sNeutralStart = 0;
    sIsStable = false;
  }

  switch (sState) {
    case IDLE: {
      if (gMag < S_GESTURE_THRESH) {
        // Only update reference when genuinely still
        if (sIsStable) { sRefX = dx; sRefY = dy; }
      } else {
        sPeakDX = gdx; sPeakDY = gdy;
        sLastPeakMag     = sqrtf(gdx*gdx + gdy*gdy);
        sPeakStableStart = millis();
        sPeakStable      = false;
        sWindowStart     = millis();
        sState           = WAITING_COMMIT;
      }
      break;
    }
    case WAITING_COMMIT: {
      if (fabsf(gdx) > fabsf(sPeakDX)) sPeakDX = gdx;
      if (fabsf(gdy) > fabsf(sPeakDY)) sPeakDY = gdy;
      float peakMag = sqrtf(sPeakDX*sPeakDX + sPeakDY*sPeakDY);

      // Peak stable timer completely separate from neutral timer
      if (peakMag > sLastPeakMag + 0.5f) {
        sLastPeakMag = peakMag; sPeakStableStart = millis(); sPeakStable = false;
      } else if (!sPeakStable) {
        if (millis() - sPeakStableStart >= S_PEAK_STABLE_MS) sPeakStable = true;
      }

      bool timedOut = (millis() - sWindowStart >= S_MAX_WINDOW);
      if (sPeakStable || timedOut) {
        if (peakMag >= S_COMMIT_THRESH) {
          String dir = directionFromPeak(sPeakDX, sPeakDY, S_DOMINANCE_RATIO);
          // Forward  = SD (tilt down)
          // Backward = SU (tilt up)
          // Left     = SL
          // Right    = SR
          if      (dir == "FORWARD")  sendCommand("SD");
          else if (dir == "BACKWARD") sendCommand("SU");
          else if (dir == "LEFT")     sendCommand("SL");
          else if (dir == "RIGHT")    sendCommand("SR");
          Serial.println();
          Serial.println("==============================");
          Serial.print("  >>  SERVO: "); Serial.println(dir);
          Serial.println("==============================");
          Serial.println();
        }
        sRefX = dx; sRefY = dy;
        sLastPeakMag = 0; sPeakStable = false;
        sIsStable = false; sNeutralStart = 0; sPeakStableStart = 0;
        sCooldownStart = millis();
        sState = COOLDOWN_STATE;
      }
      break;
    }
    case COOLDOWN_STATE:
      if (millis() - sCooldownStart >= S_COOLDOWN_MS) {
        sRefX = dx; sRefY = dy;
        sIsStable = false; sNeutralStart = 0;
        sState = IDLE;
      }
      break;
  }
}

// ══════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════
void loop() {
  sensors_event_t e;
  mag.getEvent(&e);
  fx = ALPHA_FAST * e.magnetic.x + (1.0f - ALPHA_FAST) * fx;
  fy = ALPHA_FAST * e.magnetic.y + (1.0f - ALPHA_FAST) * fy;
  fz = ALPHA_FAST * e.magnetic.z + (1.0f - ALPHA_FAST) * fz;

  float dx = fx - baseX;
  float dy = fy - baseY;
  float dz = fz - baseZ;
  float totalMag = sqrtf(dx*dx + dy*dy + dz*dz);

  int zone;
  if      (totalMag < STOP_FAR)  zone = 0;  // too far
  else if (totalMag < ROVER_MAX) zone = 1;  // rover
  else                           zone = 3;  // servo (no gap)

  // Debug print every 200ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    float gdx = (zone==1) ? fx-(baseX+rRefX) : fx-(baseX+sRefX);
    float gdy = (zone==1) ? fy-(baseY+rRefY) : fy-(baseY+sRefY);
    Serial.print("[ ");
    if      (zone==0) Serial.print("STOP_FAR");
    else if (zone==1) Serial.print("ROVER   ");
    else              Serial.print("SERVO   ");
    Serial.print(" ] mag="); Serial.print(totalMag,0);
    Serial.print(" gdx="); if(gdx>=0) Serial.print(" ");
    Serial.print(gdx,1);
    Serial.print(" gdy="); if(gdy>=0) Serial.print(" ");
    Serial.println(gdy,1);
  }

  // Zone change handler
  static int lastZone = -1;
  if (zone != lastZone) {
    if (zone == 1) {
      rState = IDLE; rRefX = dx; rRefY = dy;
      rIsStable = false; rStableStart = 0;
      rPeakDX = 0; rPeakDY = 0;
      rLastPeakMag = 0; rPeakStable = false;
      Serial.println("--- Rover zone ---");
    } else if (zone == 3) {
      sState = IDLE; sRefX = dx; sRefY = dy;
      sIsStable = false; sNeutralStart = 0;
      sPeakDX = 0; sPeakDY = 0;
      sLastPeakMag = 0; sPeakStable = false;
      Serial.println("--- Servo zone ---");
    } else {
      // zone 0 — too far
      sendCommand("X");
      Serial.println("--- Stop zone ---");
    }
    lastZone = zone;
  }

  if      (zone == 1) handleRover(dx, dy);
  else if (zone == 3) handleServo(dx, dy);

  delay(12);
}