#include <Servo.h>

Servo baseServo;  // Servo for the base (bottom)

int servoPin = 9;  // Pin for the base servo motor
int currentAngle = 90;  // Initial angle for the servo (middle position)
String inputString = "";  // String to hold the incoming data
boolean stringComplete = false;

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud rate
  baseServo.attach(servoPin);  // Attach the base servo to pin 9
  baseServo.write(currentAngle);  // Set initial servo position to 90 degrees
  inputString.reserve(10);  // Reserve memory for the input string
}

void loop() {
  if (stringComplete) {
    int angle = inputString.toInt();  // Convert input string to integer
    if (angle >= 0 && angle <= 180) {
      currentAngle = angle;  // Update the servo angle if within range
      baseServo.write(currentAngle);  // Move the servo to the specified angle
      Serial.println("Base Servo moved to: " + String(currentAngle));  // Send feedback
    }
    inputString = "";  // Clear the input string
    stringComplete = false;  // Reset the flag
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;  // When newline is received, flag the input as complete
    } else {
      inputString += inChar;  // Add incoming characters to the input string
    }
  }
}
