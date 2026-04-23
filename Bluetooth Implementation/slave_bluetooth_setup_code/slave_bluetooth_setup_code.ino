
void setup() {
  Serial.begin(115200);    // USB → MacBook (Serial Monitor)
  Serial1.begin(9600);     // HC-05 default data-mode baud rate

  Serial.println("Slave Bluetooth receiver ready");
}

void loop() {
  // If data comes from Bluetooth, send it to the PC
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
  }

  // Optional: allow typing in Serial Monitor and send back
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
}
