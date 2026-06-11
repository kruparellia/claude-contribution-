// controller_uno_r4/main.cpp
// ----------------------------------------------------------------------
// Runs on the Arduino Uno R4 Minima (the handheld controller).
//
// Two KY-023 / HW-504 joysticks + one rotary potentiometer (10k linear),
// four MG996R servos. Control mapping:
//
//   Joystick 1  (left)   X axis   ->  SHOULDER  (rate control)
//                        Y axis   ->  unused (idle)
//                        SW held  ->  BASE rotates clockwise (CW)
//
//   Joystick 2  (right)  X axis   ->  ELBOW  (rate control)
//                        Y axis   ->  unused (idle)
//                        SW held  ->  BASE rotates anti-clockwise (CCW)
//
//   Potentiometer (A4)   knob position  ->  CLAW angle  (absolute control,
//                        but "only commit while turning" — see note below)
//
//   Type 'h' or 'H' on USB Serial            -> snap to home pose.
//
// Note on the claw potentiometer:
//   A pot is *absolute*: turn the knob to position X, and we'd normally
//   set the claw to X every tick. Problem: typing 'h' to home the arm
//   would yank the claw to 90°, then on the next tick the pot reading
//   (which we never told to move) would yank it right back to wherever
//   the knob is sitting. Homing the claw would feel broken.
//
//   Fix: we only OVERWRITE angClaw when the pot reading is actively
//   changing. After the knob has been still for POT_STILL_MS, we stop
//   driving the claw from the pot, so 'h' can home the claw and it
//   stays homed until the user next twists the knob.
//
// Wiring (Uno R4 Minima):
//   Joystick 1   VRx -> A0    VRy -> A1    SW -> D2  (INPUT_PULLUP)
//   Joystick 2   VRx -> A2    VRy -> A3    SW -> D3  (INPUT_PULLUP)
//   Claw pot     CCW leg -> GND     CW leg -> 5V     wiper -> A4
//   HC-05        VCC -> 5V    GND -> GND
//                TXD -> D0 (RX1)
//                RXD -> D1 (TX1) via 1k/2k divider
//
// Build: pio run -e controller_uno_r4 -t upload
// ----------------------------------------------------------------------

#include <Arduino.h>
#include <ArmProtocol.h>  // shared 7-byte frame definition (lib/ArmProtocol/)

// ---- Pins ----------------------------------------------------------
static constexpr uint8_t PIN_J1_VRX  = A0;
static constexpr uint8_t PIN_J1_VRY  = A1;
static constexpr uint8_t PIN_J1_SW   = 2;    // hold -> base CW

static constexpr uint8_t PIN_J2_VRX  = A2;
static constexpr uint8_t PIN_J2_VRY  = A3;
static constexpr uint8_t PIN_J2_SW   = 3;    // hold -> base CCW

static constexpr uint8_t PIN_CLAW_POT = A4;  // claw position dial (10k linear)

// ---- Control feel --------------------------------------------------
// JOY_CENTER / JOY_DEADZONE / MAX_RATE — see controller_uno_r4 for the
// rationale; same values reused here so the joystick feel matches.
//
// BASE_RATE: degrees/sec the base rotates while a stick button is held.
//   Slower than MAX_RATE because the base swings the whole arm — a fast
//   slew here is the easiest way to brown the PSU. Tune later.
//
// POT_NOISE: ADC readings on an analog pin jitter by a few counts even
//   when the knob is perfectly still. We consider the knob "moving" only
//   if the reading has changed by more than POT_NOISE counts since the
//   last stored sample. Measured noise floor on this rig is ~20 counts
//   peak-to-peak; 15 keeps intentional turns responsive while ignoring
//   the bulk of that jitter. (Originally 6, which let pure noise keep
//   the pot pinned ACTIVE forever and yanked the claw away from HOME.)
//
// POT_STILL_MS: once the knob has been still (no change > POT_NOISE) for
//   this long, we STOP overwriting angClaw from the pot. That's what
//   lets the 'h' hotkey home the claw without the pot immediately
//   stomping the homed value.
static constexpr int16_t  JOY_CENTER   = 512;
static constexpr int16_t  JOY_DEADZONE = 60;
// MAX_RATE matches arm_mega's MAX_DEG_PER_SEC so the controller never
// integrates commanded angle faster than the arm can physically follow.
// If they were mismatched (controller faster), letting go of the stick
// would leave a queue of "commanded angle ahead of actual" that the arm
// would keep chasing for a moment — feels like stop-overshoot.
static constexpr float    MAX_RATE     = 60.0f;   // deg/s — joystick joints
static constexpr float    BASE_RATE    = 60.0f;   // deg/s — base spin

// Joystick expo curve strength for the shoulder/elbow sticks. 0.0 = linear
// (full deflection scales straight into MAX_RATE), 1.0 = pure cubic (very
// soft middle, hard top end). 0.6 gives a noticeably gentler feel for
// small/fine corrections while still reaching MAX_RATE at full deflection.
// Base (button-spin) and claw (absolute pot) bypass this — they aren't
// proportional inputs.
static constexpr float    JOY_EXPO     = 0.6f;
static constexpr int16_t  POT_NOISE    = 15;      // ADC counts of dead-band jitter
static constexpr uint32_t POT_STILL_MS = 500;     // knob still this long -> stop driving claw
// POT_ARM_DELTA: the pot is "armed" only after the user deliberately turns
// the knob this far from where it was sitting at boot. Until then, the
// claw is held at HOME regardless of pot readings. Stops the claw from
// being yanked to wherever the knob happens to be parked when the
// controller powers up. 50 counts ≈ 5 % of the pot's travel — easy to
// exceed with a deliberate twist, impossible to hit from noise.
static constexpr int16_t  POT_ARM_DELTA = 50;

// Home pose — must match L_*.initial on the arm side.
struct Pose { uint8_t base, shoulder, elbow, claw; };
static constexpr Pose HOME { 90, 36, 55, 5 };

// Per-axis travel limits (defence in depth — arm side clamps too).
struct Lim { uint8_t lo, hi; };
static constexpr Lim L_BASE     {   0, 180 };
static constexpr Lim L_SHOULDER {  30, 155 };
static constexpr Lim L_ELBOW    {   0, 180 };
static constexpr Lim L_CLAW     {   5, 160 };  // claw OPENS toward lo (5=open, 160=closed); matches arm LIM_CLAW

// Shoulder lift interlock (mirrors arm_mega's SHOULDER_LIFT_GATE_DEG /
// ELBOW_LIFT_SAFE_DEG — keep them in sync). While the shoulder is down past
// SHOULDER_LIFT_GATE_DEG, we refuse to integrate it back UP until the
// commanded elbow is folded back to ELBOW_LIFT_SAFE_DEG. The up-stick input
// is dropped rather than banked, so nothing lurches when the elbow tucks.
static constexpr uint8_t SHOULDER_LIFT_GATE_DEG = 80;
static constexpr uint8_t ELBOW_LIFT_SAFE_DEG    = 165;

// How long after a home request we keep asserting FLAG_HOME in outgoing
// frames, so the arm reliably catches it (over a lossy RF link) and runs
// its staged home. Comfortably longer than a few frame periods.
static constexpr uint32_t HOME_FLAG_HOLD_MS = 600;

// Tx cadence (50 Hz to the arm, identical to original firmware).
static constexpr uint32_t TICK_MS     = 20;
static constexpr uint32_t DEBOUNCE_MS = 25;
// Max ms between two J2-SW press edges to count as a double-tap (home).
// Long enough to be comfortable, short enough that two unrelated quick
// presses while spinning the base won't trigger an accidental home.
static constexpr uint32_t DOUBLE_TAP_MS = 350;

// How often we print the live angle line to USB Serial during calibration.
// 200 ms = 5 Hz is fast enough to feel responsive when you wiggle the stick
// but slow enough that the monitor stays readable.
static constexpr uint32_t PRINT_MS    = 200;

// ---- Joystick & button state --------------------------------------
struct Stick {
    uint8_t pinVrx, pinVry;
    int16_t cenX, cenY;     // measured neutral readings, set by calibrateSticks()
};
Stick stick1 { PIN_J1_VRX, PIN_J1_VRY, JOY_CENTER, JOY_CENTER };
Stick stick2 { PIN_J2_VRX, PIN_J2_VRY, JOY_CENTER, JOY_CENTER };

// Debounced push-button (only used here for the joystick SWs).
struct Button {
    uint8_t  pin;
    bool     state;     // debounced (LOW = pressed)
    bool     raw;
    uint32_t edgeMs;
};
Button btnJ1Sw { PIN_J1_SW, HIGH, HIGH, 0 };  // hold = base CW
Button btnJ2Sw { PIN_J2_SW, HIGH, HIGH, 0 };  // hold = base CCW

// Live joint angles — kept as floats to integrate sub-degree changes.
float angBase     = HOME.base;
float angShoulder = HOME.shoulder;
float angElbow    = HOME.elbow;
float angClaw     = HOME.claw;

// Reverse link from the arm: it reports its live pose (see FLAG_AUTO) so we can
// mirror it for a bumpless handoff. armAuto = the arm is driving itself now.
ArmProto::Decoder armDecoder;
ArmProto::Frame   armPose { HOME.base, HOME.shoulder, HOME.elbow, HOME.claw, 0 };
bool     armAuto     = false;
uint32_t lastArmRxMs = 0;

uint32_t lastTickMs = 0;

// millis() of the previous J2-SW press edge — used to detect double-tap-home.
uint32_t lastJ2PressMs = 0;

// Claw pot state — used to detect "is the knob being actively turned?".
//   potLastRaw    : last raw ADC reading we considered the user's intent
//   potLastMoveMs : millis() of the most recent reading that differed
//                   from potLastRaw by more than POT_NOISE
// If (now - potLastMoveMs) < POT_STILL_MS we believe the user is still
// twisting (or just stopped), and we let the pot dictate angClaw.
// Otherwise the knob is "parked" and we leave angClaw alone, so homing
// has a chance to stick.
int16_t  potLastRaw    = 0;
uint32_t potLastMoveMs = 0;
// Where the pot was sitting at boot, and whether the user has since
// turned the knob far enough to "arm" pot->claw driving. Sticky once
// true — after the first deliberate twist, normal POT_NOISE logic
// takes over for the rest of the session.
int16_t  potBootRaw    = 0;
bool     potArmed      = false;

// When did we last print the [ang] line? Separate timer from the tx tick
// because we want angle output even when the user isn't moving anything.
uint32_t lastPrintMs = 0;

// ---- Serial line buffer (calibration commands) --------------------
// During calibration you can drive each joint to a specific angle by
// typing a command on the USB Serial monitor and pressing Enter:
//
//   h         -> snap all joints to the home pose
//   b<deg>    -> set base     to <deg> (clamped to L_BASE)
//   s<deg>    -> set shoulder to <deg> (clamped to L_SHOULDER)
//   e<deg>    -> set elbow    to <deg> (clamped to L_ELBOW)
//   c<deg>    -> set claw     to <deg> (clamped to L_CLAW)
//
// Whitespace and case are ignored; e.g. "B 175" works the same as "b175".
// The buffer is intentionally small — the longest valid command is 5
// chars ("s 150" or similar) plus the newline, so 16 bytes is plenty.
static char    cmdBuf[16];
static uint8_t cmdLen = 0;

// -------------------------------------------------------------------
// Deadzone helper. Maps 0..1023 ADC to -1..+1 with a flat zero zone around
// the axis's MEASURED neutral `center` (set at boot by calibrateSticks), so a
// stick that rests a few tens of counts off 512 still reads exactly 0 when
// untouched — no creep. Scaled per-side so full deflection still reaches ±1.
static inline float applyDeadzone(int16_t raw, int16_t center) {
    int16_t d = raw - center;
    if (d >  JOY_DEADZONE) return float(d - JOY_DEADZONE) / float(1023 - center - JOY_DEADZONE);
    if (d < -JOY_DEADZONE) return float(d + JOY_DEADZONE) / float(center - JOY_DEADZONE);
    return 0.0f;
}

static inline float clampF(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Cubic expo curve. Input and output are both in [-1..+1]. Smooth and
// monotonic so there are no flat spots or kinks. With k=0 this returns v
// unchanged; with k=1 it returns v^3 (very soft middle). At v=±1 the
// output is ±1 regardless of k, so full-stick still hits MAX_RATE.
static inline float applyExpo(float v, float k) {
    float a = v < 0 ? -v : v;
    float curved = a * (1.0f - k + k * a * a);
    return v < 0 ? -curved : curved;
}

// In this variant we only care about each stick's Y axis. We still read
// X so it can be logged for debugging, but it doesn't feed the angles.
static void readStick(const Stick& s, float& outX, float& outY) {
    outX = applyDeadzone(analogRead(s.pinVrx), s.cenX);
    outY = applyDeadzone(analogRead(s.pinVry), s.cenY);
}

// Poll a debounced push-button. Returns true ONCE on the press edge,
// but we don't use that return here — we only need btn.state (LOW = held).
static bool pollButton(Button& btn, uint32_t now) {
    bool pressedEdge = false;
    bool raw = digitalRead(btn.pin);
    if (raw != btn.raw) {
        btn.raw    = raw;
        btn.edgeMs = now;
    }
    if ((now - btn.edgeMs) >= DEBOUNCE_MS && raw != btn.state) {
        bool wasReleased = (btn.state == HIGH);
        btn.state = raw;
        if (wasReleased && btn.state == LOW) pressedEdge = true;
    }
    return pressedEdge;
}

// millis() until which outgoing frames assert FLAG_HOME (set by snapHome()).
static uint32_t homeFlagUntilMs = 0;

static void snapHome() {
    angBase     = HOME.base;
    angShoulder = HOME.shoulder;
    angElbow    = HOME.elbow;
    angClaw     = HOME.claw;
    // Tell the arm to run its staged home (fold elbow -> lift shoulder ->
    // place elbow) in case it's down in the stall zone. Harmless if it's
    // already up: the arm just snaps straight home.
    homeFlagUntilMs = millis() + HOME_FLAG_HOLD_MS;
    // Re-park the pot: rebase the "boot" reading to wherever the knob
    // is sitting NOW and disarm. The claw will then sit at HOME until
    // the user makes a fresh deliberate twist (>POT_ARM_DELTA from the
    // current knob position). Without this, an already-armed pot would
    // immediately stomp the homed claw angle on the next tick.
    potBootRaw    = analogRead(PIN_CLAW_POT);
    potLastRaw    = potBootRaw;
    potArmed      = false;
    potLastMoveMs = 0;
    Serial.println(F("[ctl] home"));
}

// Parse one complete line of user input (already null-terminated, no \n).
// Handles "h", "b<deg>", "s<deg>", "e<deg>", "c<deg>" with optional spaces
// and any case. Unknown commands are reported and ignored.
//
// For calibration: when you set the claw via 'c<deg>', we also force the
// pot to be considered "parked" — otherwise the next loop iteration
// would notice that potRaw is still close to potLastRaw (knob hasn't
// moved) and... actually that wouldn't overwrite angClaw because the
// "active" window already expired. But if the knob HAS been turned in
// the last 500 ms, the pot would stomp the calibration value. Resetting
// potLastMoveMs to 0 here makes "c<deg>" always stick until you next
// twist the knob.
static void handleCommand(char* line) {
    // Strip leading whitespace.
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '\0') return;          // empty input — ignore

    char  cmd = *line++;
    if (cmd >= 'A' && cmd <= 'Z') cmd = (char)(cmd + 32);   // tolower

    if (cmd == 'h') {
        snapHome();
        return;
    }

    // Everything else expects an integer angle after the letter.
    int deg = atoi(line);          // atoi skips leading whitespace itself
    float v = (float)deg;

    switch (cmd) {
        case 'b':
            angBase = clampF(v, L_BASE.lo, L_BASE.hi);
            Serial.print(F("[cal] base="));     Serial.println((int)(angBase + 0.5f));
            break;
        case 's':
            angShoulder = clampF(v, L_SHOULDER.lo, L_SHOULDER.hi);
            Serial.print(F("[cal] shoulder=")); Serial.println((int)(angShoulder + 0.5f));
            break;
        case 'e':
            angElbow = clampF(v, L_ELBOW.lo, L_ELBOW.hi);
            Serial.print(F("[cal] elbow="));    Serial.println((int)(angElbow + 0.5f));
            break;
        case 'c':
            angClaw       = clampF(v, L_CLAW.lo, L_CLAW.hi);
            // Re-park the pot — see snapHome() for the rationale.
            potBootRaw    = analogRead(PIN_CLAW_POT);
            potLastRaw    = potBootRaw;
            potArmed      = false;
            potLastMoveMs = 0;
            Serial.print(F("[cal] claw="));     Serial.println((int)(angClaw + 0.5f));
            break;
        default:
            Serial.print(F("[cal] unknown cmd '"));
            Serial.print(cmd);
            Serial.println(F("'  (use h, b<deg>, s<deg>, e<deg>, c<deg>)"));
            break;
    }
}

// Pump the USB Serial RX buffer into cmdBuf one byte at a time. On a
// newline (CR or LF), null-terminate and hand off to handleCommand().
// Drops anything past the end of the buffer and resyncs at the next newline.
static void pumpSerialCommands() {
    while (Serial.available()) {
        int ci = Serial.read();
        if (ci < 0) break;
        char c = (char)ci;
        if (c == '\r' || c == '\n') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                handleCommand(cmdBuf);
                cmdLen = 0;
            }
        } else if (cmdLen < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen++] = c;
        } else {
            cmdLen = 0;     // overflow — drop and resync at next newline
        }
    }
}

// Print the live angle line (and a bit of pot diagnostics) at PRINT_MS.
// Use this while calibrating to see what the firmware is currently
// commanding, and whether the pot is being treated as "active" or "parked".
static void printAngles(uint32_t now) {
    if (now - lastPrintMs < PRINT_MS) return;
    lastPrintMs = now;
    bool potActive = (now - potLastMoveMs) < POT_STILL_MS;
    Serial.print(F("[ang] b="));    Serial.print((int)(angBase     + 0.5f));
    Serial.print(F(" s="));         Serial.print((int)(angShoulder + 0.5f));
    Serial.print(F(" e="));         Serial.print((int)(angElbow    + 0.5f));
    Serial.print(F(" c="));         Serial.print((int)(angClaw     + 0.5f));
    Serial.print(F("  pot="));      Serial.print(potLastRaw);
    Serial.print(F(" "));           Serial.println(potActive ? F("ACTIVE") : F("parked"));
}

// Map a pot reading (0..1023) onto the claw's safe travel range.
// Returns a float so the integrator-style loop can mix it with future
// trim adjustments cleanly.
static inline float potToClawDeg(int16_t raw) {
    // Avoid division surprises: 1023 is the ADC's max value, so the
    // range is [0..1023]. Linear map onto [L_CLAW.lo .. L_CLAW.hi].
    float t = float(raw) / 1023.0f;
    return float(L_CLAW.lo) + t * float(L_CLAW.hi - L_CLAW.lo);
}

// ---- Boot-time joystick centre calibration --------------------------
// Sticks rarely rest at exactly 512 — a few tens of counts of bias is normal,
// and with a FIXED centre that bias sits outside the deadzone so the rate
// integrator creeps the commanded angle forever. On the arm that continuous
// "drift" reads as the user driving and kicks it out of auto mode. So at boot
// — sticks assumed untouched — we measure each axis's true neutral and
// deadzone around THAT. A wildly off reading (a held stick or a floating pin)
// is rejected back to the nominal 512 so we never end up worse than before.
static int16_t measureCentre(uint8_t pin) {
    long sum = 0;
    for (uint8_t i = 0; i < 32; ++i) { sum += analogRead(pin); delay(2); }
    int16_t c = (int16_t)(sum / 32);
    if (c < JOY_CENTER - 150 || c > JOY_CENTER + 150) return JOY_CENTER;
    return c;
}

static void calibrateSticks() {
    stick1.cenX = measureCentre(stick1.pinVrx);
    stick1.cenY = measureCentre(stick1.pinVry);
    stick2.cenX = measureCentre(stick2.pinVrx);
    stick2.cenY = measureCentre(stick2.pinVry);
    Serial.print(F("[ctl] stick centres  J1("));
    Serial.print(stick1.cenX); Serial.print(','); Serial.print(stick1.cenY);
    Serial.print(F(")  J2("));
    Serial.print(stick2.cenX); Serial.print(','); Serial.print(stick2.cenY);
    Serial.println(')');
}

// -------------------------------------------------------------------
void setup() {
    pinMode(PIN_J1_SW, INPUT_PULLUP);
    pinMode(PIN_J2_SW, INPUT_PULLUP);
    // A4 (PIN_CLAW_POT) is analog input by default — no pinMode needed.

    Serial.begin(115200);    // USB CDC debug + 'h' hotkey
    Serial1.begin(9600);     // hardware UART -> HC-05 master
    Serial.println(F("[ctl] boot OK — sticks=shoulder/elbow, SW=base spin, pot=claw"));
    Serial.println(F("[ctl] cmds: h=home  b<deg>=base  s<deg>=shoulder  e<deg>=elbow  c<deg>=claw"));

    // Measure each stick's neutral so an off-centre rest position can't creep
    // the commanded angle (which the arm reads as a manual override and uses
    // to drop out of auto mode). Keep the sticks untouched through boot.
    calibrateSticks();

    // Seed the pot state with the current knob position so we don't
    // see a spurious "knob moved" on the very first tick. Also remember
    // it as the "boot" reading — we'll refuse to drive the claw from
    // the pot until the user turns the knob more than POT_ARM_DELTA
    // away from this value, so the claw stays at HOME on power-up.
    potLastRaw    = analogRead(PIN_CLAW_POT);
    potBootRaw    = potLastRaw;
    potArmed      = false;
    potLastMoveMs = 0;
    lastTickMs    = millis();
}

void loop() {
    uint32_t now = millis();

    // 1. Service the joystick switches every iteration.
    pollButton(btnJ1Sw, now);
    if (pollButton(btnJ2Sw, now)) {
        // Press edge on the elbow-stick button. Two presses inside
        // DOUBLE_TAP_MS = home. Single presses still fall through to the
        // base-CCW hold logic below.
        if (now - lastJ2PressMs <= DOUBLE_TAP_MS) {
            snapHome();
            lastJ2PressMs = 0;       // consume the pair — no triple-tap chain
        } else {
            lastJ2PressMs = now;
        }
    }

    // 1b. Drain the arm's reverse pose-report stream (bumpless handoff). Keep
    //     the latest decoded pose + whether the arm is currently autonomous.
    while (Serial1.available()) {
        ArmProto::Frame rf;
        if (armDecoder.feed((uint8_t)Serial1.read(), rf)) {
            armPose     = rf;
            armAuto     = (rf.flags & ArmProto::FLAG_AUTO) != 0;
            lastArmRxMs = now;
        }
    }

    // 2. Drain USB Serial for calibration/homing commands. Recognised:
    //      h           -> snap to home pose (only path to home this firmware)
    //      b/s/e/c <n> -> jump that joint to <n> degrees, clamped to limits
    //    (See handleCommand() for the full list.) Commands must end with
    //    a newline, which the Arduino Serial Monitor adds when you hit
    //    Send if you have "Newline" or "Both NL & CR" selected.
    pumpSerialCommands();

    // 2b. Live angle printout at PRINT_MS — handy during calibration so
    //     you can see exactly what the firmware is commanding right now.
    printAngles(now);

    // 3. Rate-limit to TICK_MS.
    if (now - lastTickMs < TICK_MS) return;
    float dt = (now - lastTickMs) / 1000.0f;
    lastTickMs = now;

    // 4. Read sticks + base buttons. (X inverted so pushing the stick UP
    //    decreases the joint angle, matching arm-mounted-in-front feel.)
    float j1x, j1y, j2x, j2y;
    readStick(stick1, j1x, j1y);
    readStick(stick2, j2x, j2y);
    bool spinCW  = (btnJ1Sw.state == LOW);
    bool spinCCW = (btnJ2Sw.state == LOW);
    (void)j1y; (void)j2y;   // Y axes intentionally unused

    // Bumpless handoff: while the arm is driving itself (armAuto) and we're not
    // touching anything locally, MIRROR its reported pose instead of holding
    // our own stale one — so taking back manual control starts from where the
    // arm actually is, with no jump. Any local input drops us straight back to
    // normal integration (and the arm, seeing movement, leaves auto). The claw
    // stays pot-driven either way. (applyDeadzone returns exactly 0 at rest.)
    bool localDriving = (j1x != 0.0f) || (j2x != 0.0f) || spinCW || spinCCW;
    bool armFresh     = (now - lastArmRxMs) < 500;
    if (armAuto && armFresh && !localDriving) {
        angBase     = armPose.base;
        angShoulder = armPose.shoulder;
        angElbow    = armPose.elbow;
    } else {
        // Normal manual integration. Expo curve first: small stick deflections
        // produce gentle motion, only the last bit of throw gives full MAX_RATE.
        float j1xExpo = applyExpo(j1x, JOY_EXPO);
        float j2xExpo = applyExpo(j2x, JOY_EXPO);
        // Elbow first, so the shoulder interlock below sees this tick's elbow.
        angElbow = clampF(angElbow + j2xExpo * MAX_RATE * dt, L_ELBOW.lo, L_ELBOW.hi);

        // Shoulder, with the lift interlock. While the shoulder is down in the
        // stall zone and the elbow isn't folded back yet, an UP command
        // (decreasing angle) is simply not applied. Lowering, and movement
        // above the threshold, are unaffected.
        float wantShoulder = clampF(angShoulder - j1xExpo * MAX_RATE * dt,
                                    L_SHOULDER.lo, L_SHOULDER.hi);
        bool inStallZone = angShoulder > SHOULDER_LIFT_GATE_DEG;
        bool elbowTucked = angElbow   >= ELBOW_LIFT_SAFE_DEG;
        bool lifting     = wantShoulder < angShoulder;   // smaller angle = arm rising
        if (inStallZone && !elbowTucked && lifting) {
            wantShoulder = angShoulder;                  // veto the lift
        }
        angShoulder = wantShoulder;

        // Base rotation: joystick SWs held = spin (both held = cancel).
        float baseDelta = 0.0f;
        if (spinCCW) baseDelta -= BASE_RATE * dt;
        if (spinCW)  baseDelta += BASE_RATE * dt;
        angBase = clampF(angBase + baseDelta, L_BASE.lo, L_BASE.hi);
    }

    // 6. Claw potentiometer -> claw angle, but only while the knob is
    //    actively being turned.
    //
    //    "Active" = the latest reading is more than POT_NOISE counts
    //    away from the last reading we accepted. Each time that happens
    //    we update potLastRaw and stamp potLastMoveMs. Then we drive
    //    the claw from the pot only while (now - potLastMoveMs) is
    //    still inside the POT_STILL_MS window.
    //
    //    Net effect:
    //      - turn the knob -> claw tracks it immediately
    //      - stop turning -> claw holds whatever angle the knob picked
    //      - type 'h' to home -> claw goes to 90°; with the knob still
    //        parked, this assignment is NOT overwritten on the next
    //        tick because potLastMoveMs is now stale
    //      - twist the knob again -> claw snaps to the new knob position
    //
    //    The "claw snaps" on resume is intentional. If you want a
    //    smooth re-entry instead, slew angClaw toward potToClawDeg(raw)
    //    rather than assigning it directly — this is the spot.
    int16_t potRaw = analogRead(PIN_CLAW_POT);
    // Arm the pot the first time it strays far enough from its boot
    // position to be unambiguous user intent. Once armed, stays armed
    // until reset — normal POT_NOISE logic then handles "is the user
    // still turning it?".
    if (!potArmed && abs(potRaw - potBootRaw) > POT_ARM_DELTA) {
        potArmed   = true;
        potLastRaw = potRaw;       // resync so the noise check below doesn't immediately re-trigger
    }
    if (abs(potRaw - potLastRaw) > POT_NOISE) {
        potLastRaw    = potRaw;
        potLastMoveMs = now;
    }
    bool potActive = potArmed && (now - potLastMoveMs) < POT_STILL_MS;
    if (potActive) {
        angClaw = clampF(potToClawDeg(potLastRaw), L_CLAW.lo, L_CLAW.hi);
    }

    // 7. Build the outgoing packet (same 7-byte frame as the original).
    ArmProto::Frame f;
    f.base     = (uint8_t)(angBase     + 0.5f);
    f.shoulder = (uint8_t)(angShoulder + 0.5f);
    f.elbow    = (uint8_t)(angElbow    + 0.5f);
    f.claw     = (uint8_t)(angClaw     + 0.5f);
    // FLAG_BUTTON: ANY input currently active — joystick SW held OR
    // pot being actively turned. The arm side ignores this today but
    // it's a free debug signal.
    bool anyActive = spinCCW || spinCW || potActive;
    f.flags = (anyActive ? ArmProto::FLAG_BUTTON : 0)
            | ((now < homeFlagUntilMs) ? ArmProto::FLAG_HOME : 0);

    uint8_t buf[ArmProto::FRAME_SIZE];
    ArmProto::encode(f, buf);
    Serial1.write(buf, ArmProto::FRAME_SIZE);
}
