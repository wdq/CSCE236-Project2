#include <Servo.h>

#define LEFT_SERVO_PIN 5
#define RIGHT_SERVO_PIN 6
#define LEFT_IR_PIN 3
#define RIGHT_IR_PIN 7
// 1 is off the line and 0 is on the line

Servo leftServo;
Servo rightServo;

void setup() {
  Serial.begin(9600);
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  servoStop();
}

void servoStraight() {
  leftServo.write(180);
  rightServo.write(1);
}

void servoStop() {
  leftServo.write(90);
  rightServo.write(91);
}

void servoLeft() {
  leftServo.write(90);
  rightServo.write(46);
}

void servoRight() {
  leftServo.write(135);
  rightServo.write(91);    
}

/*
 * 0 = stopped
 * 1 = straight
 * 2 = left
 * 3 = right
 */
uint8_t previousServoDirection = 0; 

void loop() {
  uint8_t leftIrValue = digitalRead(LEFT_IR_PIN);
  uint8_t rightIrValue = digitalRead(RIGHT_IR_PIN);
  Serial.print(leftIrValue);
  Serial.print(", ");
  Serial.println(rightIrValue);
  

  if((leftIrValue == 0) && (rightIrValue == 0)) { // on the line, go straight
    previousServoDirection = 1;
    servoStraight();
  } else if((leftIrValue == 0) && (rightIrValue == 1)) {
    previousServoDirection = 2;
    servoLeft();    
  } else if((leftIrValue == 1) && (rightIrValue == 0)) {
    previousServoDirection = 3;
    servoRight();
  } else { 
    if(previousServoDirection == 2) {
      servoRight();
    } else if(previousServoDirection == 3) {
      servoLeft();
    }
  }
}
