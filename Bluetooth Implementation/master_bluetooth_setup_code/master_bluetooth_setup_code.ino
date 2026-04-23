// HC-05 AT mode serial bridge for Uno R4

void setup() {
  Serial.begin(115200);     // USB Serial (PC)
  Serial1.begin(38400);     // HC-05 AT mode baud rate
}

void loop() {
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
