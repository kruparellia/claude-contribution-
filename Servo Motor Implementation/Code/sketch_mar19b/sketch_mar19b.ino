// #include <Servo.h>

// Servo myServo;
// const int SERVO_PIN = 10;
// const int pot = A0;
// float potValue;
// float potValueRatio;
// int servoMin = 1100;
// int servoMax = 1900;
// void setup() {
//   Serial.begin(9600);
//   pinMode(pot, INPUT);
//   myServo.attach(SERVO_PIN, 900, 2100);
//   myServo.writeMicroseconds(1500);
//   delay(1000);
// }

// void loop() {
//   potValue = analogRead(pot);
//   potValueRatio = (potValue / 692);
//   Serial.println(potValueRatio);

//   int pulse = (potValueRatio * (servoMax - servoMin)) + servoMin;
//   pulse = constrain(pulse, servoMin, servoMax);

//   myServo.writeMicroseconds(pulse);

//   delay(20);
// }
#include <Servo.h>

Servo myServo;

const int joystickPin = A0;  // VRx from joystick
const int servoPin = 9;      // orange wire from MG996R

const int servoMin = 1000;
const int servoMax = 2000;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);

  // Start centered
  myServo.writeMicroseconds(1500);
}

void loop() {
  // Read joystick (0 to 1023)
  int joyValue = analogRead(joystickPin);

  // Map joystick full range to servo safe range
  int servoCommand = map(joyValue, 0, 1023, servoMin, servoMax);

  // Constrain for safety
  servoCommand = constrain(servoCommand, servoMin, servoMax);

  // Send command to servo
  myServo.writeMicroseconds(servoCommand);

  // Debug output
  Serial.print("Joystick: ");
  Serial.print(joyValue);
  Serial.print("   Servo: ");
  Serial.println(servoCommand);

  delay(10);
}
