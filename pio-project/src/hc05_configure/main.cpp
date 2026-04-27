// hc05_configure/main.cpp
// ----------------------------------------------------------------------
// One-time HC-05 configuration helper. Flash this to the Mega, connect
// ONE HC-05 module at a time, and use the Arduino Serial Monitor to
// type AT commands at the module.
//
// HOW TO ENTER AT MODE (factory firmware, default):
//   1. Unplug the HC-05 from power.
//   2. Press and HOLD the tiny button on the HC-05 module.
//   3. While still holding the button, plug VCC back in.
//   4. Release the button after 2 seconds.
//   5. The red LED blinks slowly (~2 s period) = AT mode confirmed.
//      (Normal mode blinks ~5 Hz.)
//   In AT mode the module talks at 38400 baud, which is what this sketch
//   uses on Serial1. It forwards bytes between the USB Serial Monitor
//   and the HC-05, line-buffered with CR+LF line endings.
//
// SERIAL MONITOR SETUP:
//   - Baud rate:     115200
//   - Line ending:   "Both NL & CR"
//
// RECOMMENDED COMMAND SEQUENCE
// ---------------------------------------------------------------------
// Module 1 -> ARM (slave, lives on the Mega):
//     AT
//     AT+ORGL           // restore factory defaults
//     AT+ROLE=0         // 0 = slave
//     AT+NAME=ARM
//     AT+UART=9600,0,0  // the comm baud the arm firmware expects
//     AT+ADDR?          // WRITE DOWN the address printed here!
//                       // e.g. +ADDR:2016:7:136745  -> use 2016,7,136745
//
// Module 2 -> CTRL (master, lives on the Uno R4 controller):
//     AT
//     AT+ORGL
//     AT+ROLE=1         // 1 = master
//     AT+NAME=CTRL
//     AT+UART=9600,0,0
//     AT+CMODE=0        // connect to a fixed address only (0=fixed,1=any)
//     AT+BIND=2016,7,136745   // <-- slave's address from above
//     AT+INIT           // may respond "ERROR:(17)" = already initialised, OK
//     AT+PAIR=2016,7,136745,9
//     AT+LINK=2016,7,136745   // optional — it'll auto-link on power-up
//
// After this, pull each module off the Mega, wire them to their real
// boards, power up, and the red LED on the master should go solid (or
// double-blink every 2 s) once it's connected to the slave.
// ----------------------------------------------------------------------

#include <Arduino.h>

// Wire the HC-05 to Mega D18/D19 (Serial1) exactly like the arm firmware:
//   HC-05 TXD -> D19 (RX1)
//   HC-05 RXD -> D18 (TX1) via 1k/2k divider (R1=1k top, R2=2k to GND)

static constexpr uint32_t USB_BAUD     = 115200;
static constexpr uint32_t HC05_AT_BAUD = 38400;   // factory AT-mode baud

void setup() {
    Serial.begin(USB_BAUD);
    Serial1.begin(HC05_AT_BAUD);
    delay(50);
    Serial.println();
    Serial.println(F("== HC-05 AT-mode pass-through =="));
    Serial.println(F("Set Serial Monitor line ending to 'Both NL & CR' and 115200 baud."));
    Serial.println(F("Type AT commands; responses from the HC-05 print below."));
    Serial.println(F("If you see nothing back, the module isn't in AT mode — re-enter it."));
}

void loop() {
    while (Serial.available())   Serial1.write((uint8_t)Serial.read());
    while (Serial1.available())  Serial.write((uint8_t)Serial1.read());
}
