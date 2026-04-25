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

#include <Arduino.h>
#include <Servo.h>
#include <ArmProtocol.h>

// ---- Pin assignments ------------------------------------------------
static constexpr uint8_t SERVO_PIN_BASE     = 9;
static constexpr uint8_t SERVO_PIN_SHOULDER = 10;
static constexpr uint8_t SERVO_PIN_ELBOW    = 11;
static constexpr uint8_t SERVO_PIN_CLAW     = 6;

// ---- Motion limits --------------------------------------------------
// Tune these AFTER mechanical bring-up — set the lower/upper so the arm
// cannot drive itself into its own frame. Start conservative.
struct Limits { uint8_t lo, hi, initial; };
static constexpr Limits LIM_BASE     {  0, 180,  60 };
static constexpr Limits LIM_SHOULDER { 30, 150,  60 };
static constexpr Limits LIM_ELBOW    { 30, 150,  60 };
static constexpr Limits LIM_CLAW     { 20, 160,  60 };

// Max angular speed per servo. 120 deg/s is comfortable for MG996R and
// keeps current draw well below stall. Lower this if the battery sags.
static constexpr float MAX_DEG_PER_SEC = 120.0f;

// If we haven't seen a valid frame in this long, hold position.
static constexpr uint32_t LINK_TIMEOUT_MS = 500;

// Update tick — how often we re-compute slewed angles and write servos.
static constexpr uint32_t TICK_MS = 20;  // 50 Hz

// ---- State ----------------------------------------------------------
Servo servoBase, servoShoulder, servoElbow, servoClaw;
ArmProto::Decoder decoder;

struct AxisState {
    float current;   // what we're actually commanding (slewed)
    uint8_t target;  // what the controller last asked for
    const Limits& lim;
    Servo& servo;
};

AxisState axes[] = {
    { (float)LIM_BASE.initial,     LIM_BASE.initial,     LIM_BASE,     servoBase     },
    { (float)LIM_SHOULDER.initial, LIM_SHOULDER.initial, LIM_SHOULDER, servoShoulder },
    { (float)LIM_ELBOW.initial,    LIM_ELBOW.initial,    LIM_ELBOW,    servoElbow    },
    { (float)LIM_CLAW.initial,     LIM_CLAW.initial,     LIM_CLAW,     servoClaw     },
};

uint32_t lastFrameMs = 0;
uint32_t lastTickMs  = 0;

// ---------------------------------------------------------------------
static inline uint8_t clampTo(uint8_t v, const Limits& l) {
    if (v < l.lo) return l.lo;
    if (v > l.hi) return l.hi;
    return v;
}

static void applyFrame(const ArmProto::Frame& f) {
    axes[0].target = clampTo(f.base,     axes[0].lim);
    axes[1].target = clampTo(f.shoulder, axes[1].lim);
    axes[2].target = clampTo(f.elbow,    axes[2].lim);
    axes[3].target = clampTo(f.claw,     axes[3].lim);
    lastFrameMs = millis();
}

static void slewAndWrite(uint32_t dtMs) {
    const float stepMax = MAX_DEG_PER_SEC * (dtMs / 1000.0f);
    for (auto& a : axes) {
        float err = (float)a.target - a.current;
        if (err >  stepMax) err =  stepMax;
        if (err < -stepMax) err = -stepMax;
        a.current += err;
        a.servo.write((int)(a.current + 0.5f));
    }
}

// ---------------------------------------------------------------------
void setup() {
    Serial.begin(115200);          // USB — debug log
    Serial1.begin(9600);           // HC-05 default baud

    servoBase.attach(SERVO_PIN_BASE);
    servoShoulder.attach(SERVO_PIN_SHOULDER);
    servoElbow.attach(SERVO_PIN_ELBOW);
    servoClaw.attach(SERVO_PIN_CLAW);

    // Move to the safe starting pose immediately.
    for (auto& a : axes) a.servo.write(a.lim.initial);

    Serial.println(F("[arm] boot OK — waiting for frames on Serial1"));
    lastFrameMs = millis();
    lastTickMs  = millis();
}

void loop() {
    // 1. Drain the BT rx FIFO into the decoder.
    while (Serial1.available()) {
        ArmProto::Frame f;
        if (decoder.feed((uint8_t)Serial1.read(), f)) {
            applyFrame(f);
        }
    }

    // 2. Every TICK_MS, slew the servos one step.
    uint32_t now = millis();
    if (now - lastTickMs >= TICK_MS) {
        uint32_t dt = now - lastTickMs;
        lastTickMs = now;

        // Link watchdog: if silent too long, snap target to current so
        // the arm holds its pose instead of completing the last motion.
        if (now - lastFrameMs > LINK_TIMEOUT_MS) {
            for (auto& a : axes) a.target = (uint8_t)(a.current + 0.5f);
        }

        slewAndWrite(dt);
    }
}
