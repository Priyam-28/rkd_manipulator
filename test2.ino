#include <Servo.h>

Servo servo1;  // Left servo
Servo servo2;  // Right servo

void setup() {
  Serial.begin(9600);
  servo1.attach(10);  // Pin 9 for servo 1
  servo2.attach(11); // Pin 10 for servo 2
}

void loop() {
  if (Serial.available() > 0) {
    // Read the angles sent from the Python script
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    
    if (commaIndex > 0) {
      // Extract the two angles
      int left_servo_angle = data.substring(0, commaIndex).toInt();
      int right_servo_angle = data.substring(commaIndex + 1).toInt();
      
      // Move both servos to the synchronized angles
      servo1.write(left_servo_angle);
      servo2.write(right_servo_angle);
    }
  }
}
