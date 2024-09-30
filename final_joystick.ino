#include <Servo.h>

Servo baseMotor;
Servo rightServo;
Servo leftServo;
Servo endEffector;  // End effector servo

int baseAngle = 90;  // Initial positions
int rightServoAngle = 90;
int leftServoAngle = 90;
int endEffectorAngle = 90;  // End effector starts in a neutral position

void setup() {
  Serial.begin(9600);
  
  // Attach servos to corresponding pins
  baseMotor.attach(9);    // Base motor to pin 9
  rightServo.attach(10);  // Right servo to pin 10
  leftServo.attach(11);   // Left servo to pin 11
  endEffector.attach(12); // End effector to pin 12

  // Set initial positions
  baseMotor.write(baseAngle);
  rightServo.write(rightServoAngle);
  leftServo.write(leftServoAngle);
  endEffector.write(endEffectorAngle);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    
    // Parse incoming data
    int basePosIndex = data.indexOf('B');
    int rightPosIndex = data.indexOf('R');
    int leftPosIndex = data.indexOf('L');
    int endEffectorPosIndex = data.indexOf('E');

    // Extract angles
    baseAngle = data.substring(basePosIndex + 1, rightPosIndex).toInt();
    rightServoAngle = data.substring(rightPosIndex + 1, leftPosIndex).toInt();
    leftServoAngle = data.substring(leftPosIndex + 1, endEffectorPosIndex).toInt();
    endEffectorAngle = data.substring(endEffectorPosIndex + 1).toInt();
    
    // Write values to motors and servos
    baseMotor.write(baseAngle);
    rightServo.write(rightServoAngle);
    leftServo.write(leftServoAngle);
    endEffector.write(endEffectorAngle);
  }
}
