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

#include <Arduino.h>      // Core Arduino API: pinMode, analogRead, millis, Serial, ...
#include <ArmProtocol.h>  // Shared header (lib/ArmProtocol/) — defines the 7-byte
                          // packet format and an encoder. The arm-side firmware
                          // includes the same header so both sides agree on the
                          // wire format.

// ---- Feature toggles -----------------------------------------------
// 0 = both axes of a stick can move at once (smoother, default)
// 1 = only the dominant axis moves; the other is forced to zero
//
// `#define` is a preprocessor switch — the compiler literally substitutes
// the value before compiling. We use it (instead of a constexpr) so the
// `#if DIAGONAL_GATE` block below can be turned on/off at compile time.
#define DIAGONAL_GATE 0

// ---- Pins ----------------------------------------------------------
// Which Uno R4 pins each joystick is wired to. A0..A5 = analog-capable
// inputs (read with analogRead()); 2/3 are plain digital pins for the
// joystick push-switch. Change here only if you re-wire the controller.
static constexpr uint8_t PIN_J1_VRX = A0;   // joystick 1 X potentiometer
static constexpr uint8_t PIN_J1_VRY = A1;   // joystick 1 Y potentiometer
static constexpr uint8_t PIN_J1_SW  = 2;    // joystick 1 push-switch (active LOW)

static constexpr uint8_t PIN_J2_VRX = A2;
static constexpr uint8_t PIN_J2_VRY = A3;
static constexpr uint8_t PIN_J2_SW  = 3;

// ---- Control feel --------------------------------------------------
// JOY_CENTER: with the stick at rest, analogRead() returns roughly this.
//   The Uno R4's ADC defaults to 10-bit (0..1023), so centre ≈ 512.
// JOY_DEADZONE: the stick's springs aren't perfect — it rarely returns to
//   exactly 512. Anything within ±JOY_DEADZONE counts as "no input", so
//   the arm doesn't drift when the user lets go. Bigger deadzone = more
//   tolerant of sloppy sticks but less sensitive at small deflections.
// MAX_RATE: how fast a joint moves when the stick is fully deflected, in
//   degrees per second. The math below scales this by the stick reading.
static constexpr int16_t JOY_CENTER   = 512;   // 10-bit ADC idle value
static constexpr int16_t JOY_DEADZONE = 60;    // +/- counts treated as 0
static constexpr float   MAX_RATE     = 90.0f; // deg/s at full deflection

// Home pose (must match LIM_*.initial on the arm side — keep in sync).
// `Pose` just bundles the four target angles into one named thing so we
// can write `HOME.base` instead of remembering which array index is which.
struct Pose { uint8_t base, shoulder, elbow, claw; };
static constexpr Pose HOME { 90, 90, 90, 90 };

// Per-axis travel limits (also clamped on the arm side — defence in depth).
// These prevent the controller from ever asking the arm to drive past a
// safe range, even if the user holds the stick all the way over.
struct Lim { uint8_t lo, hi; };
static constexpr Lim L_BASE     {   0, 180 };
static constexpr Lim L_SHOULDER {  0, 150 };
static constexpr Lim L_ELBOW    {  30, 150 };
static constexpr Lim L_CLAW     {  20, 160 };

// Tx cadence
// TICK_MS:       send a packet every 20 ms = 50 packets/sec.
// LONG_PRESS_MS: hold the joystick switch this long to trigger "home".
// DEBOUNCE_MS:   ignore button transitions shorter than this — mechanical
//                switches "bounce" (chatter) for a few ms when pressed,
//                producing 5-10 fake edges. We wait for the line to settle.
static constexpr uint32_t TICK_MS       = 20;    // 50 Hz tx
static constexpr uint32_t LONG_PRESS_MS = 1000;
static constexpr uint32_t DEBOUNCE_MS   = 25;

// ---- Joystick & button state --------------------------------------
// One Stick struct per joystick. Holds wiring (which pins) AND the live
// debounce/long-press state machine for that stick's button.
//   swState     = the *debounced* logical state we trust (HIGH = released,
//                 LOW = pressed; LOW because the switch is wired to GND
//                 and uses the MCU's INPUT_PULLUP, so closing it pulls
//                 the line low).
//   swRaw       = the most recent unfiltered read of the pin.
//   edgeMs      = millis() time of the last raw transition.
//   downMs      = millis() when swState last went LOW (start of a press).
//   longHandled = once we fire the long-press action, latch this so we
//                 don't fire again every tick while the user keeps holding.
struct Stick {
    uint8_t pinVrx, pinVry, pinSw;
    bool     swState;     // debounced (HIGH = released, LOW = pressed)
    bool     swRaw;
    uint32_t edgeMs;
    uint32_t downMs;
    bool     longHandled;
};

// Initialise both sticks: pins as configured above, both buttons assumed
// released at boot, all timers at zero.
Stick stick1 { PIN_J1_VRX, PIN_J1_VRY, PIN_J1_SW, HIGH, HIGH, 0, 0, false };
Stick stick2 { PIN_J2_VRX, PIN_J2_VRY, PIN_J2_SW, HIGH, HIGH, 0, 0, false };

// The four angles we send to the arm, kept as floats so we can integrate
// small fractional changes per tick (rate control). Initialised to the
// home pose so the very first packet sent is sensible.
float angBase     = HOME.base;
float angShoulder = HOME.shoulder;
float angElbow    = HOME.elbow;
float angClaw     = HOME.claw;

uint32_t lastTickMs = 0;   // when we last transmitted, in millis()

// -------------------------------------------------------------------
// Convert a raw 0..1023 ADC reading into a normalised -1.0..+1.0 value
// with a flat zero region around centre.
//
//   raw < centre - deadzone  ->  negative output, scaled so full = -1.0
//   raw within deadzone      ->  exactly 0.0
//   raw > centre + deadzone  ->  positive output, scaled so full = +1.0
//
// This is what makes "stick at rest" mean "joint doesn't move", and full
// deflection mean "move at MAX_RATE".
static inline float applyDeadzone(int16_t raw) {
    int16_t d = raw - JOY_CENTER;
    if (d >  JOY_DEADZONE) return float(d - JOY_DEADZONE) / (1023 - JOY_CENTER - JOY_DEADZONE);
    if (d < -JOY_DEADZONE) return float(d + JOY_DEADZONE) / (JOY_CENTER - JOY_DEADZONE);
    return 0.0f;
}

// Tiny helper: squash a float into [lo, hi]. The `?:` is a ternary
// expression — equivalent to: if (v < lo) return lo; else if (v > hi) ...
static inline float clampF(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Read both axes of a stick into outX, outY (each -1.0..+1.0). The `&`
// in the parameters means we modify the caller's variables directly,
// which is how a C/C++ function returns more than one value.
static void readStick(const Stick& s, float& outX, float& outY) {
    float x = applyDeadzone(analogRead(s.pinVrx));
    float y = applyDeadzone(analogRead(s.pinVry));
#if DIAGONAL_GATE
    // Optional "no diagonals" mode: whichever axis is being pushed harder
    // wins, the other is forced to zero. Helps when you only want to move
    // one joint at a time but the joystick keeps drifting on the other axis.
    if (fabsf(x) >= fabsf(y)) y = 0.0f;
    else                      x = 0.0f;
#endif
    outX = x;
    outY = y;
}

// Reset all four angles to the home pose. Called when a long-press or
// the laptop 'h' hotkey fires.
static void snapHome() {
    angBase     = HOME.base;
    angShoulder = HOME.shoulder;
    angElbow    = HOME.elbow;
    angClaw     = HOME.claw;
    Serial.println(F("[ctl] home"));
}

// Polls one joystick switch and returns true on the tick a long-press
// is detected. Implements:
//   1. Debounce — only accept a logical change after the line has held
//      its new level for at least DEBOUNCE_MS.
//   2. Long-press detect — once the (debounced) state is LOW for >=
//      LONG_PRESS_MS, fire exactly once (latched by `longHandled`).
//
// `Stick&` (reference, not const) because we modify the state in-place.
static bool pollButton(Stick& s, uint32_t now) {
    bool fired = false;
    bool raw   = digitalRead(s.pinSw);
    if (raw != s.swRaw) {
        // Raw line just changed — restart the debounce timer.
        s.swRaw  = raw;
        s.edgeMs = now;
    }
    // Line has been stable for DEBOUNCE_MS in its new state — accept it.
    if ((now - s.edgeMs) >= DEBOUNCE_MS && raw != s.swState) {
        s.swState = raw;
        if (s.swState == LOW) {       // pressed (active-LOW with INPUT_PULLUP)
            s.downMs      = now;
            s.longHandled = false;
        }
        // Release: short press is reserved — intentionally does nothing.
        // (If you ever want a short-press action, hook it in here on the
        //  HIGH transition, gated by !s.longHandled.)
    }
    // Fire the long-press action once, the moment we cross the threshold.
    if (s.swState == LOW && !s.longHandled &&
        (now - s.downMs) >= LONG_PRESS_MS) {
        s.longHandled = true;
        fired = true;
    }
    return fired;
}

// -------------------------------------------------------------------
// One-time boot setup. Arduino calls this once after reset.
void setup() {
    // INPUT_PULLUP: enable the MCU's internal pull-up resistor on these
    // pins. Joystick switches are wired between the pin and GND, so the
    // pin reads HIGH at rest and LOW when pressed.
    pinMode(PIN_J1_SW, INPUT_PULLUP);
    pinMode(PIN_J2_SW, INPUT_PULLUP);
    Serial.begin(115200);    // USB CDC — debug log, also accepts the 'h' hotkey
    Serial1.begin(9600);     // hardware UART on D0/D1 — to the HC-05 master
    Serial.println(F("[ctl] boot OK — 2 joysticks, 4 DOF"));
    lastTickMs = millis();
}

// Main loop — Arduino runs this forever, as fast as it can.
void loop() {
    uint32_t now = millis();

    // 1. Service the buttons every iteration (so we don't miss a fast
    //    press). Each call returns true ONLY on the tick a long-press
    //    fires, so we don't re-snap home repeatedly while held.
    bool home1 = pollButton(stick1, now);
    bool home2 = pollButton(stick2, now);

    // 2. Laptop hotkey: 'h' / 'H' typed on USB Serial also snaps home.
    //    We drain everything in the rx buffer so other characters don't
    //    accumulate forever, but only react to 'h'/'H'.
    bool homeKey = false;
    while (Serial.available()) {
        int c = Serial.read();
        if (c == 'h' || c == 'H') homeKey = true;
    }

    if (home1 || home2 || homeKey) snapHome();

    // 3. Rate-limit the rest of the loop to TICK_MS (= 50 Hz). Reading
    //    the ADC and sending a packet every full loop iteration would be
    //    wasteful and could overrun the 9600-baud BT link.
    if (now - lastTickMs < TICK_MS) return;
    float dt = (now - lastTickMs) / 1000.0f;   // elapsed time in SECONDS for the integration below
    lastTickMs = now;

    float j1x, j1y, j2x, j2y;
    readStick(stick1, j1x, j1y);
    readStick(stick2, j2x, j2y);

    // Rate control: stick deflection sets *velocity*, not position. Each
    // tick we add (deflection x MAX_RATE x dt) to the current angle, then
    // clamp to the joint's safe range. Y axes are subtracted so pushing
    // the stick UP makes the angle DECREASE — feels right with the arm
    // mounted in front of the operator.
    //
    // Joystick 1 -> base (X) + shoulder (Y, inverted so up = forward).
    angBase     = clampF(angBase     + j1x * MAX_RATE * dt, L_BASE.lo,     L_BASE.hi);
    angShoulder = clampF(angShoulder - j1y * MAX_RATE * dt, L_SHOULDER.lo, L_SHOULDER.hi);
    // Joystick 2 -> elbow (Y) + claw (X).
    angElbow    = clampF(angElbow    - j2y * MAX_RATE * dt, L_ELBOW.lo,    L_ELBOW.hi);
    angClaw     = clampF(angClaw     + j2x * MAX_RATE * dt, L_CLAW.lo,     L_CLAW.hi);

    // 4. Build the outgoing packet. Cast each float to uint8_t with +0.5
    //    for round-to-nearest. flags carries one bit telling the arm a
    //    button is currently pressed (currently unused on the arm side
    //    but reserved — it's "free" since the byte is in the frame anyway).
    ArmProto::Frame f;
    f.base     = (uint8_t)(angBase     + 0.5f);
    f.shoulder = (uint8_t)(angShoulder + 0.5f);
    f.elbow    = (uint8_t)(angElbow    + 0.5f);
    f.claw     = (uint8_t)(angClaw     + 0.5f);
    f.flags    = ((stick1.swState == LOW) || (stick2.swState == LOW)) ? ArmProto::FLAG_BUTTON : 0;

    // 5. Serialise into the 7-byte wire format and shove it down Serial1.
    //    The HC-05 transparently relays it to the paired slave, which is
    //    listening on Serial1 on the Mega.
    uint8_t buf[ArmProto::FRAME_SIZE];
    ArmProto::encode(f, buf);
    Serial1.write(buf, ArmProto::FRAME_SIZE);
}
