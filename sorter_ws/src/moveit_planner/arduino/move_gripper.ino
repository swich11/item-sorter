#include <Servo.h>

Servo myServo;

#define controlPin 9
#define openAngle 0
#define closeAngle 60

#define baudRate 9600

void setup() {
  // set baud rate, pwm pin, open as default
  Serial.begin(baudRate);
  myServo.attach(controlPin);
  myServo.write(openAngle);
}

void loop() {
  // check reading serial, if there is check command, otherwise print error
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "open") {
      myServo.write(openAngle);
      Serial.println("Gripper opened");
    } 
    else if (cmd == "close") {
      myServo.write(closeAngle);
      Serial.println("Gripper closed");
    } 
    else {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
    }
  } else {
    // No command received
    Serial.print("no messesage picked up"); // small delay to avoid busy-waiting
    delay(100);
  }
}
