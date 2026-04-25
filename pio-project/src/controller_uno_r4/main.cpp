// controller_uno_r4/main.cpp
// ----------------------------------------------------------------------
// Runs on the Arduino Uno R4 Minima (the handheld controller).
//
// Two KY-023 / HW-504 joysticks, four MG996R servos. Each axis maps to
// one motor (rate control — joystick = velocity, not position):
//
//   Joystick 1  (left)   X  ->  base
//                        Y  ->  shoulder
//   Joystick 2  (right)  X  ->  elbow
//                        Y  ->  claw
//
//   Long-press EITHER SW (>= 1 s)  ->  snap all axes back to home pose.
//   Short-press SW                 ->  reserved (no-op for now).
//   Type 'h' or 'H' on USB Serial  ->  snap to home (laptop hotkey path,
//                                      see scripts/home.py).
//
// Wiring (Uno R4 Minima):
//   Joystick 1   VRx -> A0    VRy -> A1    SW -> D2  (INPUT_PULLUP)
//   Joystick 2   VRx -> A2    VRy -> A3    SW -> D3  (INPUT_PULLUP)
//   HC-05        VCC -> 5V    GND -> GND
//                TXD -> D0 (RX1)
//                RXD -> D1 (TX1) via 1k/2k divider
//
// Notes:
//   - Serial   = USB CDC port (debug log, doesn't touch the BT link).
//   - Serial1  = hardware UART on D0/D1 (HC-05, 9600 baud).
//   - DIAGONAL_GATE: set to 1 to force one-axis-at-a-time motion per stick
//     (whichever axis has the larger deflection wins, the other is zeroed).
//     Set to 0 to allow simultaneous X+Y motion. Toggle and compare feel.
// ----------------------------------------------------------------------

#include <Arduino.h>
#include <ArmProtocol.h>

// ---- Feature toggles -----------------------------------------------
// 0 = both axes of a stick can move at once (smoother, default)
// 1 = only the dominant axis moves; the other is forced to zero
#define DIAGONAL_GATE 0

// ---- Pins ----------------------------------------------------------
static constexpr uint8_t PIN_J1_VRX = A0;
static constexpr uint8_t PIN_J1_VRY = A1;
static constexpr uint8_t PIN_J1_SW  = 2;

static constexpr uint8_t PIN_J2_VRX = A2;
static constexpr uint8_t PIN_J2_VRY = A3;
static constexpr uint8_t PIN_J2_SW  = 3;

// ---- Control feel --------------------------------------------------
static constexpr int16_t JOY_CENTER   = 512;   // 10-bit ADC idle value
static constexpr int16_t JOY_DEADZONE = 60;    // +/- counts treated as 0
static constexpr float   MAX_RATE     = 90.0f; // deg/s at full deflection

// Home pose (must match LIM_*.initial on the arm side — keep in sync).
struct Pose { uint8_t base, shoulder, elbow, claw; };
static constexpr Pose HOME { 90, 90, 90, 90 };

// Per-axis travel limits (also clamped on the arm side).
struct Lim { uint8_t lo, hi; };
static constexpr Lim L_BASE     {   0, 180 };
static constexpr Lim L_SHOULDER {  30, 150 };
static constexpr Lim L_ELBOW    {  30, 150 };
static constexpr Lim L_CLAW     {  20, 160 };

// Tx cadence
static constexpr uint32_t TICK_MS       = 20;    // 50 Hz tx
static constexpr uint32_t LONG_PRESS_MS = 1000;
static constexpr uint32_t DEBOUNCE_MS   = 25;

// ---- Joystick & button state --------------------------------------
struct Stick {
    uint8_t pinVrx, pinVry, pinSw;
    bool     swState;     // debounced (HIGH = released, LOW = pressed)
    bool     swRaw;
    uint32_t edgeMs;
    uint32_t downMs;
    bool     longHandled;
};

Stick stick1 { PIN_J1_VRX, PIN_J1_VRY, PIN_J1_SW, HIGH, HIGH, 0, 0, false };
Stick stick2 { PIN_J2_VRX, PIN_J2_VRY, PIN_J2_SW, HIGH, HIGH, 0, 0, false };

float angBase     = HOME.base;
float angShoulder = HOME.shoulder;
float angElbow    = HOME.elbow;
float angClaw     = HOME.claw;

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

static void readStick(const Stick& s, float& outX, float& outY) {
    float x = applyDeadzone(analogRead(s.pinVrx));
    float y = applyDeadzone(analogRead(s.pinVry));
#if DIAGONAL_GATE
    // Only the dominant axis is allowed through.
    if (fabsf(x) >= fabsf(y)) y = 0.0f;
    else                      x = 0.0f;
#endif
    outX = x;
    outY = y;
}

static void snapHome() {
    angBase     = HOME.base;
    angShoulder = HOME.shoulder;
    angElbow    = HOME.elbow;
    angClaw     = HOME.claw;
    Serial.println(F("[ctl] home"));
}

// Returns true if a long-press fired on this stick this tick.
static bool pollButton(Stick& s, uint32_t now) {
    bool fired = false;
    bool raw   = digitalRead(s.pinSw);
    if (raw != s.swRaw) {
        s.swRaw  = raw;
        s.edgeMs = now;
    }
    if ((now - s.edgeMs) >= DEBOUNCE_MS && raw != s.swState) {
        s.swState = raw;
        if (s.swState == LOW) {       // pressed
            s.downMs      = now;
            s.longHandled = false;
        }
        // Release: short press is reserved — intentionally does nothing.
    }
    if (s.swState == LOW && !s.longHandled &&
        (now - s.downMs) >= LONG_PRESS_MS) {
        s.longHandled = true;
        fired = true;
    }
    return fired;
}

// -------------------------------------------------------------------
void setup() {
    pinMode(PIN_J1_SW, INPUT_PULLUP);
    pinMode(PIN_J2_SW, INPUT_PULLUP);
    Serial.begin(115200);
    Serial1.begin(9600);
    Serial.println(F("[ctl] boot OK — 2 joysticks, 4 DOF"));
    lastTickMs = millis();
}

void loop() {
    uint32_t now = millis();

    bool home1 = pollButton(stick1, now);
    bool home2 = pollButton(stick2, now);

    // Laptop hotkey: 'h' / 'H' on USB Serial snaps home. Drain anything
    // else so we don't backlog the rx FIFO.
    bool homeKey = false;
    while (Serial.available()) {
        int c = Serial.read();
        if (c == 'h' || c == 'H') homeKey = true;
    }

    if (home1 || home2 || homeKey) snapHome();

    if (now - lastTickMs < TICK_MS) return;
    float dt = (now - lastTickMs) / 1000.0f;
    lastTickMs = now;

    float j1x, j1y, j2x, j2y;
    readStick(stick1, j1x, j1y);
    readStick(stick2, j2x, j2y);

    // Joystick 1 -> base (X) + shoulder (Y, inverted so up = forward).
    angBase     = clampF(angBase     + j1x * MAX_RATE * dt, L_BASE.lo,     L_BASE.hi);
    angShoulder = clampF(angShoulder - j1y * MAX_RATE * dt, L_SHOULDER.lo, L_SHOULDER.hi);
    // Joystick 2 -> elbow (Y) + claw (X).
    angElbow    = clampF(angElbow    - j2y * MAX_RATE * dt, L_ELBOW.lo,    L_ELBOW.hi);
    angClaw     = clampF(angClaw     + j2x * MAX_RATE * dt, L_CLAW.lo,     L_CLAW.hi);

    ArmProto::Frame f;
    f.base     = (uint8_t)(angBase     + 0.5f);
    f.shoulder = (uint8_t)(angShoulder + 0.5f);
    f.elbow    = (uint8_t)(angElbow    + 0.5f);
    f.claw     = (uint8_t)(angClaw     + 0.5f);
    f.flags    = ((stick1.swState == LOW) || (stick2.swState == LOW)) ? ArmProto::FLAG_BUTTON : 0;

    uint8_t buf[ArmProto::FRAME_SIZE];
    ArmProto::encode(f, buf);
    Serial1.write(buf, ArmProto::FRAME_SIZE);
}
