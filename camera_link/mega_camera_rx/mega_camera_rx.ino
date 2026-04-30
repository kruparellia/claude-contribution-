// ============================================================================
// Mega-side receiver for the ESP32-WROVER colour-detector link.
//
// Drop these helpers into your existing Mega sketch (the one already driving
// the servos via the joystick from the Uno+HC-05). Call cameraSetup() from
// setup(), and cameraPoll() once per main-loop iteration. Read the latest
// detection from the global `cam` struct.
//
// Wire: ESP32 GPIO17 (TX2) -> Mega pin 17 (RX2), GND -> GND. Serial2 on
// the Mega is on pins 16 (TX2) / 17 (RX2). Serial1 (pins 18/19) is
// reserved for the HC-05 link, so the camera uses Serial2.
//
// Message format from the ESP32 (one ASCII line, '\n' terminated):
//   CAM,<state>,<colour>,<pct>
// ============================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Public state — read this from your servo / sort-mode logic
// ---------------------------------------------------------------------------
enum CamColour : uint8_t {
  CAM_NONE = 0, CAM_RED, CAM_GREEN, CAM_BLUE, CAM_YELLOW
};

struct CameraState {
  bool       present  = false;     // is a coloured object in frame?
  CamColour  colour   = CAM_NONE;  // which colour, if present
  uint8_t    pct      = 0;         // % of frame covered by the colour
  uint32_t   lastMsgMs = 0;        // millis() of the last good message
  bool       linkOk   = false;     // false if no message seen recently
};

CameraState cam;

// If we don't hear from the camera for this long, assume the link dropped.
static const uint32_t CAM_TIMEOUT_MS = 1000;

// ---------------------------------------------------------------------------
// Internal parser state
// ---------------------------------------------------------------------------
static char     rxBuf[64];
static uint8_t  rxLen = 0;

static CamColour parseColour(const char* s) {
  if (!strcmp(s, "RED"))    return CAM_RED;
  if (!strcmp(s, "GREEN"))  return CAM_GREEN;
  if (!strcmp(s, "BLUE"))   return CAM_BLUE;
  if (!strcmp(s, "YELLOW")) return CAM_YELLOW;
  return CAM_NONE;
}

static const char* colourName(CamColour c) {
  switch (c) {
    case CAM_RED:    return "RED";
    case CAM_GREEN:  return "GREEN";
    case CAM_BLUE:   return "BLUE";
    case CAM_YELLOW: return "YELLOW";
    default:         return "NONE";
  }
}

// Parse one complete line. Returns true if the message was well-formed.
static bool handleLine(char* line) {
  // Expecting:  CAM,<state>,<colour>,<pct>
  if (strncmp(line, "CAM,", 4) != 0) return false;

  char* p = line + 4;
  char* stateStr  = strtok(p,    ",");
  char* colourStr = strtok(NULL, ",");
  char* pctStr    = strtok(NULL, ",");
  if (!stateStr || !colourStr || !pctStr) return false;

  int state = atoi(stateStr);
  int pct   = atoi(pctStr);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;

  cam.present   = (state != 0);
  cam.colour    = cam.present ? parseColour(colourStr) : CAM_NONE;
  cam.pct       = (uint8_t)pct;
  cam.lastMsgMs = millis();
  cam.linkOk    = true;
  return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void cameraSetup() {
  Serial2.begin(115200);            // matches LINK_BAUD on the ESP32
  rxLen = 0;
}

// Call frequently from loop(). Non-blocking.
void cameraPoll() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) handleLine(rxBuf);
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      // Overflow — discard and resync at the next newline.
      rxLen = 0;
    }
  }

  if (cam.linkOk && (millis() - cam.lastMsgMs) > CAM_TIMEOUT_MS) {
    cam.linkOk  = false;
    cam.present = false;
    cam.colour  = CAM_NONE;
    cam.pct     = 0;
  }
}

// ---------------------------------------------------------------------------
// Standalone demo. Delete setup()/loop() when merging into your real sketch.
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);             // USB debug to the Arduino IDE
  cameraSetup();
  Serial.println("Mega camera-RX demo ready.");
}

void loop() {
  cameraPoll();

  // Print state changes only, so the monitor isn't spammed.
  static bool      lastPresent = false;
  static CamColour lastColour  = CAM_NONE;
  static bool      lastLink    = false;

  if (cam.linkOk != lastLink) {
    Serial.print("link: "); Serial.println(cam.linkOk ? "OK" : "LOST");
    lastLink = cam.linkOk;
  }
  if (cam.present != lastPresent || cam.colour != lastColour) {
    Serial.print("object: ");
    Serial.print(cam.present ? "present " : "absent  ");
    Serial.print("colour=");
    Serial.print(colourName(cam.colour));
    Serial.print(" pct=");
    Serial.println(cam.pct);
    lastPresent = cam.present;
    lastColour  = cam.colour;
  }

  // ---- example hook for sort-mode --------------------------------------
  // if (cam.linkOk && cam.present) {
  //   switch (cam.colour) {
  //     case CAM_RED:    /* move arm to bin A */ break;
  //     case CAM_GREEN:  /* move arm to bin B */ break;
  //     case CAM_BLUE:   /* move arm to bin C */ break;
  //     case CAM_YELLOW: /* move arm to bin D */ break;
  //     default: break;
  //   }
  // }
}
