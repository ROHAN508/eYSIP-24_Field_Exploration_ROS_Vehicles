#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

int throttle = 0;
int servo_angle = 1800;
const int dirPin = 19;   // DIR pin
const int pwmPin = 18;  // PWM pin
const int potPin = 34;
int diff =0;
const int servoPin1 = 27;
const int servoPin2 = 12;
Servo myServo;

void setup() {
  Serial.begin(9600);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(potPin, INPUT); // Set the potentiometer pin as input
  myServo.attach(servoPin1);
  myServo.attach(servoPin2); 
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Arduino is ready to receive data");
}

void loop() {
  int potValue = analogRead(potPin);
  if (Serial.available()) {
    // int potValue = analogRead(potPin);
    String input = Serial.readStringUntil('\n'); // Read until newline character
    int commaIndex = input.indexOf(',');         // Find the comma separating the values
    if (commaIndex > 0) {
      String throttleStr = input.substring(0, commaIndex);
      String servoAngleStr = input.substring(commaIndex + 1);
      throttle = throttleStr.toInt();
      servo_angle = servoAngleStr.toInt();

      Serial.print("Received throttle: ");
      Serial.println(throttle);
      Serial.print("Received servo_angle: ");
      Serial.println(servo_angle);
    }
    // int potValue = analogRead(potPin);
    
    
    runMotor(throttle);

    
  }
  diff = servo_angle - potValue;
  Serial.print("diff: ");
  Serial.println(diff);
  if(servo_angle < potValue and abs(diff) > 85){
      analogWrite(servoPin1, pwmVal(abs(diff)));
    // analogWrite(servoPin1, 20);
      analogWrite(servoPin2, 0);
    }
  else if(servo_angle > potValue and abs(diff > 85)){
      analogWrite(servoPin1, 0);
      analogWrite(servoPin2, pwmVal(abs(diff)));
      // analogWrite(servoPin2, 20);

    }
  else{
      analogWrite(servoPin1, 0);
      analogWrite(servoPin2, 0);
    }
}

void runMotor(int speed) {
  if (speed>0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }
  analogWrite(pwmPin, abs(speed)); // Set motor speed (0-255)
}

int pwmVal(int x){
  int pVal = 0.25*x;
  if (pVal > 10){
    return pVal;
  }
  else{
    return 5;
  }
}