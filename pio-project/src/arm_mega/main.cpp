// arm_mega/main.cpp
// ----------------------------------------------------------------------
// Runs on the Elegoo Mega 2560 R3 mounted on the arm base.
//
// Responsibilities:
//   1. Receive 7-byte framed packets from the HC-05 (paired as SLAVE)
//      on Serial1 at 9600 baud.
//   2. Rate-limit commanded angles into smooth servo motion (slew).
//   3. Fail-safe: if no valid frame for LINK_TIMEOUT_MS, freeze in place.
//   4. Receive colour-detection messages from the ESP32-WROVER CAM on
//      Serial2 at 115200 baud. The latest detection is exposed in `cam`
//      so sort-mode logic can branch on it. The camera path is purely
//      additive — if the ESP32 is unplugged, the arm still works.
//
// Wiring (Mega):
//   HC-05  VCC  -> 5V
//   HC-05  GND  -> GND
//   HC-05  TXD  -> Mega D19  (RX1)           [HC-05 TX is 3.3V-ish, Mega accepts]
//   HC-05  RXD  -> Mega D18  (TX1) via divider (see wiring guide)
//   ESP32  TX2  -> Mega D17  (RX2)           [ESP32 GPIO17, 3.3V, Mega accepts]
//   ESP32  GND  -> Mega GND  (shared ground required)
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

// ---- Camera link constants -----------------------------------------
// Baud must match LINK_BAUD in esp32_color_detect.ino.
static constexpr uint32_t CAM_BAUD       = 115200;
// If we don't hear from the ESP32 for this long, the link is considered
// dead and `cam.linkOk` falls back to false. Independent of the HC-05
// watchdog above — the camera dropping out must not freeze the arm.
static constexpr uint32_t CAM_TIMEOUT_MS = 1000;

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

// ---- Camera state ---------------------------------------------------
// `cam` is the public face of the camera link. Read-only from the rest
// of the sketch's point of view — only cameraPoll() updates it.
//
// Use `cam.linkOk` to know whether the ESP32 is alive at all, then
// `cam.present` and `cam.colour` to drive sort-mode behaviour.
enum CamColour : uint8_t {
    CAM_NONE = 0, CAM_RED, CAM_GREEN, CAM_BLUE, CAM_YELLOW
};

struct CameraState {
    bool      present   = false;     // is a coloured object in frame?
    CamColour colour    = CAM_NONE;  // which colour, if present
    uint8_t   pct       = 0;         // % of frame covered by the colour
    uint32_t  lastMsgMs = 0;         // millis() of the last good message
    bool      linkOk    = false;     // false if no message seen recently
};
CameraState cam;

// Line buffer for the Serial2 parser. 64 bytes is plenty for the
// `CAM,1,YELLOW,42` style messages we expect (longest is ~20 chars).
static char    camRxBuf[64];
static uint8_t camRxLen = 0;

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
// Camera-link helpers
//
// Wire format (one ASCII line per frame, '\n' terminated):
//   CAM,<state>,<colour>,<pct>
//     state  = 0 (no object) | 1 (object present)
//     colour = NONE | RED | GREEN | BLUE | YELLOW
//     pct    = 0..100, percent of frame matching the dominant colour
//
// Parsing is one byte at a time so a half-arrived line never blocks
// the main loop.
// ---------------------------------------------------------------------
static CamColour parseCamColour(const char* s) {
    if (!strcmp(s, "RED"))    return CAM_RED;
    if (!strcmp(s, "GREEN"))  return CAM_GREEN;
    if (!strcmp(s, "BLUE"))   return CAM_BLUE;
    if (!strcmp(s, "YELLOW")) return CAM_YELLOW;
    return CAM_NONE;
}

static const char* camColourName(CamColour c) {
    switch (c) {
        case CAM_RED:    return "RED";
        case CAM_GREEN:  return "GREEN";
        case CAM_BLUE:   return "BLUE";
        case CAM_YELLOW: return "YELLOW";
        default:         return "NONE";
    }
}

// Parse one complete `CAM,...` line and update `cam`. Returns true on
// success. Any malformed line is silently dropped — we'll just resync
// at the next newline.
static bool handleCamLine(char* line) {
    if (strncmp(line, "CAM,", 4) != 0) return false;
    char* p          = line + 4;
    char* stateStr   = strtok(p,    ",");
    char* colourStr  = strtok(NULL, ",");
    char* pctStr     = strtok(NULL, ",");
    if (!stateStr || !colourStr || !pctStr) return false;

    int state = atoi(stateStr);
    int pct   = atoi(pctStr);
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;

    cam.present   = (state != 0);
    cam.colour    = cam.present ? parseCamColour(colourStr) : CAM_NONE;
    cam.pct       = (uint8_t)pct;
    cam.lastMsgMs = millis();
    cam.linkOk    = true;
    return true;
}

static void cameraSetup() {
    Serial2.begin(CAM_BAUD);
    camRxLen = 0;
}

// Drain Serial2's RX buffer non-blockingly. Call once per loop().
static void cameraPoll() {
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '\r') continue;
        if (c == '\n') {
            camRxBuf[camRxLen] = '\0';
            if (camRxLen > 0) handleCamLine(camRxBuf);
            camRxLen = 0;
        } else if (camRxLen < sizeof(camRxBuf) - 1) {
            camRxBuf[camRxLen++] = c;
        } else {
            // Overflow — drop and resync at next newline.
            camRxLen = 0;
        }
    }

    // Camera watchdog. Independent of the HC-05 watchdog: a dead camera
    // must NOT cause the arm to freeze, only to forget the last detection.
    if (cam.linkOk && (millis() - cam.lastMsgMs) > CAM_TIMEOUT_MS) {
        cam.linkOk  = false;
        cam.present = false;
        cam.colour  = CAM_NONE;
        cam.pct     = 0;
    }
}

// Print camera state changes to USB-debug only (not on every frame),
// so the monitor stays readable. Cheap to leave on in production.
static void logCamChanges() {
    static bool      lastPresent = false;
    static CamColour lastColour  = CAM_NONE;
    static bool      lastLink    = false;

    if (cam.linkOk != lastLink) {
        Serial.print(F("[cam] link: "));
        Serial.println(cam.linkOk ? F("OK") : F("LOST"));
        lastLink = cam.linkOk;
    }
    if (cam.present != lastPresent || cam.colour != lastColour) {
        Serial.print(F("[cam] "));
        Serial.print(cam.present ? F("present ") : F("absent  "));
        Serial.print(F("colour="));
        Serial.print(camColourName(cam.colour));
        Serial.print(F(" pct="));
        Serial.println(cam.pct);
        lastPresent = cam.present;
        lastColour  = cam.colour;
    }
}

// ---------------------------------------------------------------------
// Arduino calls setup() once at power-on / after reset.
void setup() {
    Serial.begin(115200);          // USB — debug log (visible in `pio device monitor`)
    Serial1.begin(9600);           // Hardware UART on D18/D19 — talks to the HC-05.
                                   // 9600 must match what we set with AT+UART
                                   // when configuring the modules.
    cameraSetup();                 // Hardware UART2 on D16/D17 — talks to the
                                   // ESP32-WROVER CAM at 115200 (see CAM_BAUD).

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
    Serial.println(F("[arm] boot OK — Serial1=HC-05@9600, Serial2=cam@115200"));
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

    // 1b. Drain the camera receive buffer. Non-blocking; updates `cam`
    //     in place. logCamChanges() prints transitions only, so it
    //     doesn't spam the USB monitor at the camera's frame rate.
    cameraPoll();
    logCamChanges();

    // ---- Sort-mode hook (autonomous mode) -------------------------------
    // When `cam.linkOk && cam.present`, branch on `cam.colour` to choose
    // a destination bin. Leave the manual joystick path untouched — the
    // sort-mode planner can override `axes[].target` between this point
    // and slewAndWrite() below.
    //
    // Example skeleton:
    //   if (sortModeActive && cam.linkOk && cam.present) {
    //       switch (cam.colour) {
    //           case CAM_RED:    moveToBin(BIN_A); break;
    //           case CAM_GREEN:  moveToBin(BIN_B); break;
    //           case CAM_BLUE:   moveToBin(BIN_C); break;
    //           case CAM_YELLOW: moveToBin(BIN_D); break;
    //           default: break;
    //       }
    //   }

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
