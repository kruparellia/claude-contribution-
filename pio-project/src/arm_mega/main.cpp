// arm_mega/main.cpp
// ----------------------------------------------------------------------
// Runs on the Elegoo Mega 2560 R3 mounted on the arm base.
//
// Responsibilities:
//   1. Receive 7-byte framed packets from the HC-05 (paired as SLAVE)
//      on Serial1 at 9600 baud.
//   2. Rate-limit commanded angles into smooth servo motion (slew).
//   3. Fail-safe: if no valid frame for LINK_TIMEOUT_MS, freeze in place.
//
// Wiring (Mega):
//   HC-05  VCC  -> 5V
//   HC-05  GND  -> GND
//   HC-05  TXD  -> Mega D19  (RX1)           [HC-05 TX is 3.3V-ish, Mega accepts]
//   HC-05  RXD  -> Mega D18  (TX1) via divider (see wiring guide)
//   Servos     : see SERVO_PIN_* below. Power from external 4xAA pack,
//                NOT from the Mega's 5V rail. Share GND with the Mega.
//
// IMPORTANT — pin choice on Mega:
//   The Servo library on Mega supports servos on ANY digital pin (unlike
//   the Uno). D9/10/11 are fine; D6 is used for the claw to keep lines
//   away from the SPI header in case we add sensors later.
// ----------------------------------------------------------------------

#include <Arduino.h>      // Core Arduino types/functions (millis(), Serial, etc.)
#include <Servo.h>        // Standard Arduino Servo library — generates the PWM
                          // pulse train that an MG996R expects (1.0..2.0 ms
                          // pulse every ~20 ms, mapped to 0..180 degrees).
#include <ArmProtocol.h>  // Our shared header in lib/ArmProtocol — defines the
                          // 7-byte packet format and the byte-by-byte decoder.

// ---- Pin assignments ------------------------------------------------
// Which Mega digital pin each servo's signal wire is connected to. Change
// these here ONLY if you re-route the wiring; the rest of the code reads
// these constants instead of hard-coded numbers, so you don't have to hunt
// through the file. (`static constexpr uint8_t` = compile-time constant,
// 1 byte, file-local — the C++ way to say "#define a small integer".)
static constexpr uint8_t SERVO_PIN_BASE     = 9;
static constexpr uint8_t SERVO_PIN_SHOULDER = 10;
static constexpr uint8_t SERVO_PIN_ELBOW    = 11;
static constexpr uint8_t SERVO_PIN_CLAW     = 6;

// ---- Motion limits --------------------------------------------------
// Tune these AFTER mechanical bring-up — set the lower/upper so the arm
// cannot drive itself into its own frame. Start conservative.
//
// `Limits` bundles three numbers per joint:
//   lo      = lowest angle (degrees) we will ever command
//   hi      = highest angle we will ever command
//   initial = where the joint should sit at power-on (the "safe pose")
// Keeping them grouped means we can pass one struct around instead of
// three loose variables, and the compiler can inline the whole thing.
struct Limits { uint8_t lo, hi, initial; };
static constexpr Limits LIM_BASE     {  0, 180,  60 };
static constexpr Limits LIM_SHOULDER { 30, 150,  60 };
static constexpr Limits LIM_ELBOW    { 30, 150,  60 };
static constexpr Limits LIM_CLAW     { 20, 160,  60 };

// Max angular speed per servo. 120 deg/s is comfortable for MG996R and
// keeps current draw well below stall. Lower this if the battery sags.
// This is what makes the arm move SMOOTHLY instead of snapping instantly
// to whatever angle the joystick last asked for — see slewAndWrite().
static constexpr float MAX_DEG_PER_SEC = 120.0f;

// If we haven't seen a valid frame in this long, hold position.
// Acts as a safety net: if the BT link drops or the controller is
// switched off mid-motion, the arm freezes instead of running away.
static constexpr uint32_t LINK_TIMEOUT_MS = 500;

// Update tick — how often we re-compute slewed angles and write servos.
// 20 ms = 50 Hz. The Servo library refreshes pulses at ~50 Hz internally
// anyway, so going faster here would just waste CPU.
static constexpr uint32_t TICK_MS = 20;  // 50 Hz

// ---- State ----------------------------------------------------------
// Four Servo objects, one per joint. `Servo` is a class from <Servo.h>;
// declaring an object here reserves a slot in the library's internal
// table of active servos. We don't .attach() them until setup().
Servo servoBase, servoShoulder, servoElbow, servoClaw;

// One decoder instance that "owns" the 7-byte buffer for in-progress
// frames coming in from Serial1. Stateful: it remembers how many bytes
// of the next frame it has already seen.
ArmProto::Decoder decoder;

// One row per joint. Bundling per-axis info into a struct lets the rest
// of the code loop over `axes[]` instead of repeating the same logic
// four times. References (`Limits&`, `Servo&`) mean the struct points
// at the original objects — no copies, no risk of writing to a copy.
struct AxisState {
    float current;       // what we're actually commanding right now (slewed,
                         // float so we can move by fractional degrees per tick)
    uint8_t target;      // the latest angle the controller asked for
    const Limits& lim;   // motion limits for this joint (read-only)
    Servo& servo;        // the Servo object we write the pulse to
};

// Initialise each axis: start at the safe pose, target the safe pose
// (so we don't immediately move on boot), and bind the lim/servo refs.
AxisState axes[] = {
    { (float)LIM_BASE.initial,     LIM_BASE.initial,     LIM_BASE,     servoBase     },
    { (float)LIM_SHOULDER.initial, LIM_SHOULDER.initial, LIM_SHOULDER, servoShoulder },
    { (float)LIM_ELBOW.initial,    LIM_ELBOW.initial,    LIM_ELBOW,    servoElbow    },
    { (float)LIM_CLAW.initial,     LIM_CLAW.initial,     LIM_CLAW,     servoClaw     },
};

// Timestamps (milliseconds since boot, from millis()).
uint32_t lastFrameMs = 0;   // when we last received a valid frame — fed the watchdog
uint32_t lastTickMs  = 0;   // when we last ran the slew/write step

// ---------------------------------------------------------------------
// Squash a value into the [lo, hi] range. `inline` hints the compiler to
// paste the body at each call site (cheap, the function is tiny).
static inline uint8_t clampTo(uint8_t v, const Limits& l) {
    if (v < l.lo) return l.lo;
    if (v > l.hi) return l.hi;
    return v;
}

// Called every time a fresh, checksum-valid frame arrives. We only update
// the *targets* — the actual servo motion is paced by slewAndWrite().
// Each angle is clamped on the way in so a buggy controller can't drive
// the arm past its mechanical stops.
static void applyFrame(const ArmProto::Frame& f) {
    axes[0].target = clampTo(f.base,     axes[0].lim);
    axes[1].target = clampTo(f.shoulder, axes[1].lim);
    axes[2].target = clampTo(f.elbow,    axes[2].lim);
    axes[3].target = clampTo(f.claw,     axes[3].lim);
    lastFrameMs = millis();   // pet the watchdog — we just heard from the controller
}

// Step every joint one tick closer to its target, never moving more than
// MAX_DEG_PER_SEC * dt degrees in a single tick. This is the "slew rate
// limit" that turns a step input from the controller into a smooth ramp.
//
// dtMs = how long it's actually been since the last tick (usually ~20 ms,
// but using the real value keeps motion uniform if a tick gets delayed).
static void slewAndWrite(uint32_t dtMs) {
    const float stepMax = MAX_DEG_PER_SEC * (dtMs / 1000.0f);  // max degrees we may move this tick
    for (auto& a : axes) {                                     // `auto&` = "reference to whatever's in axes[]"
        float err = (float)a.target - a.current;               // how far we still want to go
        if (err >  stepMax) err =  stepMax;                    // cap forward speed
        if (err < -stepMax) err = -stepMax;                    // cap reverse speed
        a.current += err;                                      // advance our internal angle
        a.servo.write((int)(a.current + 0.5f));                // round-to-nearest, push to servo
    }
}

// ---------------------------------------------------------------------
// Arduino calls setup() once at power-on / after reset.
void setup() {
    Serial.begin(115200);          // USB — debug log (visible in `pio device monitor`)
    Serial1.begin(9600);           // Hardware UART on D18/D19 — talks to the HC-05.
                                   // 9600 must match what we set with AT+UART
                                   // when configuring the modules.

    // Tell the Servo library which pin to drive for each joint. `.attach()`
    // hooks the pin into the library's internal 50 Hz pulse generator.
    servoBase.attach(SERVO_PIN_BASE);
    servoShoulder.attach(SERVO_PIN_SHOULDER);
    servoElbow.attach(SERVO_PIN_ELBOW);
    servoClaw.attach(SERVO_PIN_CLAW);

    // Move to the safe starting pose immediately so the arm doesn't sit
    // at whatever random angle the servos last held.
    for (auto& a : axes) a.servo.write(a.lim.initial);

    // F("...") stores the string in flash (program memory) instead of RAM.
    // RAM is precious on the Mega (8 KB) — every F() saves a few bytes.
    Serial.println(F("[arm] boot OK — waiting for frames on Serial1"));
    lastFrameMs = millis();
    lastTickMs  = millis();
}

// Arduino calls loop() over and over, as fast as it can, forever.
void loop() {
    // 1. Drain the Bluetooth receive buffer. `Serial1.available()` returns
    //    how many unread bytes are sat in the UART's hardware FIFO. We
    //    feed each one to the decoder; it returns true when it has a
    //    complete, checksum-valid 7-byte frame.
    while (Serial1.available()) {
        ArmProto::Frame f;
        if (decoder.feed((uint8_t)Serial1.read(), f)) {
            applyFrame(f);
        }
    }

    // 2. Every TICK_MS, slew the servos one step toward their targets.
    uint32_t now = millis();
    if (now - lastTickMs >= TICK_MS) {
        uint32_t dt = now - lastTickMs;   // actual elapsed time, used by slewAndWrite
        lastTickMs = now;

        // Link watchdog: if we haven't heard a valid frame for too long,
        // pretend the target is wherever we currently are. The slew step
        // then has zero error, so the arm stops mid-motion instead of
        // sailing on toward the last received target.
        if (now - lastFrameMs > LINK_TIMEOUT_MS) {
            for (auto& a : axes) a.target = (uint8_t)(a.current + 0.5f);
        }

        slewAndWrite(dt);
    }
    // Note: nothing else runs in loop(), so this is a tight poll. If you
    // ever add work here, keep it short — anything > a few ms eats into
    // the tick budget and motion gets jerky.
}
