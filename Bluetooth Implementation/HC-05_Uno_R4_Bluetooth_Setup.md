# HC-05 Bluetooth Communication on Arduino Uno R4

This document summarizes **exactly how the HC‑05 Bluetooth modules were successfully configured and used** with **two Arduino Uno R4 boards**, including pitfalls, macOS workaround, and final working code.

---

## Final Working State

- Two Arduino Uno R4 boards, each with one HC‑05
- One HC‑05 configured as **Master** (with EN/KEY access)
- One HC‑05 acting as **Slave** (no EN pin access)
- Bluetooth automatically connects on power‑up
- Data sent from the Master Arduino appears on the Slave Arduino Serial Monitor

**LED states:**
- Master: double blink (connected, data mode)
- Slave: solid red (connected, data mode)

---

## Key Hardware Rules

- HC‑05 is wireless only **between modules**, not to Arduino
- Each HC‑05 connects to its own Arduino TX/RX
- Arduino Uno R4 is **5 V logic** → HC‑05 RX needs level shifting
- Voltage divider:
  - Arduino TX → HC‑05 RX (1k / 2k)
  - HC‑05 TX → Arduino RX (direct)
- Each HC‑05 powered from its own Arduino

---

## Uno R4 Specific Notes

- No automatic USB–serial bridge
- Must use `Serial1` for Bluetooth
- `SoftwareSerial` not suitable
- AT mode requires a serial bridge sketch

---

## macOS Workaround (Firmware v3.0)

- HC‑05 v3.0 breaks `AT+INQ`
- Slave paired to macOS
- Hold **Option** and click Bluetooth icon
- Read slave MAC address
- Convert format:

```
AA:BB:CC:DD:EE:FF → AABB,CC,DDEEFF
```

- Use address with `AT+BIND`

---

## AT Mode Master Bridge Code

```cpp
void setup() {
  Serial.begin(115200);
  Serial1.begin(38400);
}

void loop() {
  if (Serial.available()) Serial1.write(Serial.read());
  if (Serial1.available()) Serial.write(Serial1.read());
}
```

Used with commands like:
- `AT`
- `AT+ROLE=1`
- `AT+CMODE=0`
- `AT+BIND=xxxx,yy,zzzzzz`

---

## Slave Data Mode Code

```cpp
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.println("Slave Bluetooth receiver ready");
}

void loop() {
  if (Serial1.available()) Serial.write(Serial1.read());
  if (Serial.available()) Serial1.write(Serial.read());
}
```

---

## Master Data Mode Code

```cpp
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.println("MASTER ready - type to send");
}

void loop() {
  if (Serial.available()) Serial1.write(Serial.read());
  if (Serial1.available()) Serial.write(Serial1.read());
}
```

---

## Uploading with Two Boards

Error:
```
dfu-util: More than one DFU capable USB device found
```

Fix:
- Unplug one board while uploading
- Or explicitly select port under **Tools → Port**

---

## Final Mental Model

- HC‑05 = wireless serial cable
- AT mode only for configuration
- Data mode for communication
- Auto‑reconnect after power cycle

---

## Outcome

✅ Stable Bluetooth
✅ Reliable serial data
✅ Survives resets
