void setup() {
  Serial.begin(115200);    // USB to Mac
  Serial1.begin(9600);     // Bluetooth data mode baud

  Serial.println("MASTER ready - type to send");
}

void loop() {
  // PC → Bluetooth
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }

  // Bluetooth → PC (optional echo)
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}