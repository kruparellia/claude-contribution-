// ArmProtocol.h
// ----------------------------------------------------------------------
// Wire format shared by the controller (Uno R4) and the arm (Mega 2560)
// over the HC-05 Bluetooth link.
//
// Frame layout (7 bytes, little-endian, no padding):
//
//   byte 0   : START_BYTE  (0xA5) — lets the receiver re-sync after noise
//   byte 1   : base angle  (0..180)
//   byte 2   : shoulder    (0..180)
//   byte 3   : elbow       (0..180)
//   byte 4   : claw        (0..180, but we'll clamp ~20..160 in firmware)
//   byte 5   : flags       (bit0 = joystick button, bit1 = home request, rest reserved)
//   byte 6   : checksum    (XOR of bytes 0..5)
//
// Design notes:
//   - 1 byte per angle is plenty (1-degree resolution >> servo accuracy).
//   - XOR checksum catches single-bit errors on the RF link cheaply.
//   - Fixed start byte means a desynced receiver can recover on the next
//     frame without us needing COBS/HDLC-style framing.
//   - At 50 Hz update rate, 7 bytes * 10 bits/byte * 50 = 3500 bps,
//     well inside the HC-05 default of 9600 baud.
// ----------------------------------------------------------------------

#pragma once

#include <Arduino.h>

namespace ArmProto {

static constexpr uint8_t START_BYTE = 0xA5;
static constexpr uint8_t FRAME_SIZE = 7;

// Bit positions in the flags byte
static constexpr uint8_t FLAG_BUTTON = 0x01;   // a joystick SW / pot is active
static constexpr uint8_t FLAG_HOME   = 0x02;   // home requested — arm runs its staged home

struct Frame {
    uint8_t base;
    uint8_t shoulder;
    uint8_t elbow;
    uint8_t claw;
    uint8_t flags;
};

// XOR checksum over the start byte + 5 payload bytes.
inline uint8_t checksum(const Frame& f) {
    return uint8_t(START_BYTE) ^ f.base ^ f.shoulder ^ f.elbow ^ f.claw ^ f.flags;
}

// Serialise a Frame into a 7-byte buffer. Caller supplies the buffer.
inline void encode(const Frame& f, uint8_t out[FRAME_SIZE]) {
    out[0] = START_BYTE;
    out[1] = f.base;
    out[2] = f.shoulder;
    out[3] = f.elbow;
    out[4] = f.claw;
    out[5] = f.flags;
    out[6] = checksum(f);
}

// Stateful byte-at-a-time decoder. Feed every incoming byte into feed().
// Returns true exactly once per valid frame; the decoded frame is in `out`.
class Decoder {
public:
    bool feed(uint8_t b, Frame& out) {
        buf_[idx_++] = b;

        // Resync: if first byte isn't the start byte, drop it.
        if (idx_ == 1 && buf_[0] != START_BYTE) {
            idx_ = 0;
            return false;
        }

        if (idx_ < FRAME_SIZE) return false;

        // Full frame in buffer — validate checksum.
        idx_ = 0;
        Frame f {buf_[1], buf_[2], buf_[3], buf_[4], buf_[5]};
        if (checksum(f) != buf_[6]) return false;  // corrupted, drop it

        out = f;
        return true;
    }

private:
    uint8_t buf_[FRAME_SIZE];
    uint8_t idx_ = 0;
};

}  // namespace ArmProto
