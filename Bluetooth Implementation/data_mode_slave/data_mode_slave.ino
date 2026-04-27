void setup() {
  Serial.begin(115200);    // USB to Mac
  Serial1.begin(9600);     // Bluetooth data mode baud

  Serial.println("SLAVE ready - receiving data");
}

void loop() {
  // Bluetooth → PC
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
  }
}