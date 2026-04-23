// controller_uno_r4/main.cpp
// ----------------------------------------------------------------------
// Runs on the Arduino Uno R4 Minima (the handheld controller).
//
// Responsibilities:
//   1. Read the HW-504 joystick (VRx, VRy, SW).
//   2. Integrate joystick deflection into 4 commanded servo angles
//      (rate control — joystick = velocity, not absolute position).
//   3. Transmit a 7-byte framed packet over Serial1 to the HC-05
//      (paired as MASTER) at 9600 baud, 50 Hz.
//
// Control scheme (one 2-axis joystick, 4 DOF):
//   - Short press on SW  -> toggle axis pair
//       mode A: X = base,  Y = shoulder
//       mode B: X = elbow, Y = claw
//   - Long press (>= 1s) -> snap back to the home pose.
//
// Wiring (Uno R4 Minima):
//   Joystick  VRx -> A0    VRy -> A1    SW -> D2 (INPUT_PULLUP)
//   Joystick  VCC -> 5V    GND -> GND
//   HC-05     VCC -> 5V    GND -> GND
//   HC-05     TXD -> D0 (RX1)
//   HC-05     RXD -> D1 (TX1) via divider (see wiring guide)
//
// Note on the Uno R4 Minima: Serial is the USB CDC port (handy for
// debug printf), Serial1 is the hardware UART on D0/D1. We use Serial1
// exclusively for the HC-05 — so opening the Serial Monitor on USB
// does NOT interfere with the Bluetooth link.
// ----------------------------------------------------------------------

#include <Arduino.h>
#include <ArmProtocol.h>

// ---- Pins ----------------------------------------------------------
static constexpr uint8_t PIN_VRX = A0;
static constexpr uint8_t PIN_VRY = A1;
static constexpr uint8_t PIN_SW  = 2;

// ---- Control feel --------------------------------------------------
static constexpr int16_t JOY_CENTER   = 512;   // 10-bit ADC idle value
static constexpr int16_t JOY_DEADZONE = 60;    // +/- counts treated as 0
static constexpr float   MAX_RATE     = 90.0f; // deg/s at full deflection

// Home pose (matches LIM_*.initial on the arm side — keep in sync).
struct Pose { uint8_t base, shoulder, elbow, claw; };
static constexpr Pose HOME { 90, 90, 90, 90 };

// Per-axis travel limits (also clamped by the arm side, but clamping
// locally keeps the commanded value from drifting into saturation).
struct Lim { uint8_t lo, hi; };
static constexpr Lim L_BASE     {   0, 180 };
static constexpr Lim L_SHOULDER {  30, 150 };
static constexpr Lim L_ELBOW    {  30, 150 };
static constexpr Lim L_CLAW     {  20, 160 };

// Tx cadence
static constexpr uint32_t TICK_MS          = 20;    // 50 Hz tx
static constexpr uint32_t LONG_PRESS_MS    = 1000;
static constexpr uint32_t DEBOUNCE_MS      = 25;

// ---- State ---------------------------------------------------------
float angBase     = HOME.base;
float angShoulder = HOME.shoulder;
float angElbow    = HOME.elbow;
float angClaw     = HOME.claw;

bool     modeB        = false;   // false = A (base/shoulder), true = B (elbow/claw)
bool     buttonState  = HIGH;    // current debounced state (pullup => HIGH idle)
bool     buttonRaw    = HIGH;
uint32_t buttonEdgeMs = 0;
uint32_t buttonDownMs = 0;
bool     longHandled  = false;

uint32_t lastTickMs = 0;

// -------------------------------------------------------------------
static inline float applyDeadzone(int16_t raw) {
    int16_t d = raw - JOY_CENTER;
    if (d >  JOY_DEADZONE) return float(d - JOY_DEADZONE) / (1023 - JOY_CENTER - JOY_DEADZONE);
    if (d < -JOY_DEADZONE) return float(d + JOY_DEADZONE) / (JOY_CENTER - JOY_DEADZONE);
    return 0.0f;
}

static inline float clampF(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void snapHome() {
    angBase     = HOME.base;
    angShoulder = HOME.shoulder;
    angElbow    = HOME.elbow;
    angClaw     = HOME.claw;
}

static void pollButton(uint32_t now) {
    bool raw = digitalRead(PIN_SW);
    if (raw != buttonRaw) {
        buttonRaw    = raw;
        buttonEdgeMs = now;
    }
    if ((now - buttonEdgeMs) >= DEBOUNCE_MS && raw != buttonState) {
        buttonState = raw;
        if (buttonState == LOW) {             // pressed
            buttonDownMs = now;
            longHandled  = false;
        } else {                               // released
            if (!longHandled) {
                // short press -> toggle mode
                modeB = !modeB;
                Serial.print(F("[ctl] mode "));
                Serial.println(modeB ? 'B' : 'A');
            }
        }
    }
    // long-press fires while still held
    if (buttonState == LOW && !longHandled &&
        (now - buttonDownMs) >= LONG_PRESS_MS) {
        longHandled = true;
        snapHome();
        Serial.println(F("[ctl] home"));
    }
}

// -------------------------------------------------------------------
void setup() {
    pinMode(PIN_SW, INPUT_PULLUP);
    Serial.begin(115200);
    Serial1.begin(9600);
    Serial.println(F("[ctl] boot OK"));
    lastTickMs = millis();
}

void loop() {
    uint32_t now = millis();
    pollButton(now);

    if (now - lastTickMs < TICK_MS) return;
    float dt = (now - lastTickMs) / 1000.0f;
    lastTickMs = now;

    // Joystick -> normalised [-1, +1]
    float jx = applyDeadzone(analogRead(PIN_VRX));
    float jy = applyDeadzone(analogRead(PIN_VRY));

    // Rate control: integrate deflection over time.
    if (!modeB) {
        angBase     = clampF(angBase     + jx * MAX_RATE * dt, L_BASE.lo,     L_BASE.hi);
        angShoulder = clampF(angShoulder - jy * MAX_RATE * dt, L_SHOULDER.lo, L_SHOULDER.hi);
    } else {
        angElbow    = clampF(angElbow    - jy * MAX_RATE * dt, L_ELBOW.lo,    L_ELBOW.hi);
        angClaw     = clampF(angClaw     + jx * MAX_RATE * dt, L_CLAW.lo,     L_CLAW.hi);
    }

    // Build + send frame.
    ArmProto::Frame f;
    f.base     = (uint8_t)(angBase     + 0.5f);
    f.shoulder = (uint8_t)(angShoulder + 0.5f);
    f.elbow    = (uint8_t)(angElbow    + 0.5f);
    f.claw     = (uint8_t)(angClaw     + 0.5f);
    f.flags    = (buttonState == LOW) ? ArmProto::FLAG_BUTTON : 0;

    uint8_t buf[ArmProto::FRAME_SIZE];
    ArmProto::encode(f, buf);
    Serial1.write(buf, ArmProto::FRAME_SIZE);
}
