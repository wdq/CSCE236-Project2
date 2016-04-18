#include <Servo.h>

#define RIGHT_BUMPER_TRIGGER 2
#define LEFT_BUMPER_TRIGGER 3
#define LEFT_IR A0
#define RIGHT_IR A1
#define CALIBRATE_BUTTON_PIN 4
#define MOTOR_SWITCH_PIN 7
#define LEFT_SERVO_PIN 6
#define RIGHT_SERVO_PIN 5
#define IR_SERVO_PIN 11
#define LEFT_LED_PIN 12
#define RIGHT_LED_PIN 13
#define VCNL4000_ADDRESS 0x13  // 0x26 write, 0x27 read

Servo leftServo, rightServo;

/*
 * -1 = line is on the right
 * 0  = robot is on the line
 * 1  = line is on the left
 */
int linePos = 0;
int alreadyTurned = 0;


/*
 * Servo details:
 * 
 *          LEFT    RIGHT
 * stop:    90      92
 * straight:100     82
 * backup:  80      102
 * right:   101     103
 * left:    80      82
 */

 /*
  * IR Sensors edge values:
  *       LEFT    RIGHT
  *       1000    930
  */

void setup() {
  Serial.begin(9600);
  
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN );
  servoStop();

  pinMode(MOTOR_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LEFT_BUMPER_TRIGGER, INPUT_PULLUP);
  pinMode(RIGHT_BUMPER_TRIGGER, INPUT_PULLUP);
  
  digitalWrite(MOTOR_SWITCH_PIN, 1);
  digitalWrite(LEFT_BUMPER_TRIGGER, 1);
  digitalWrite(RIGHT_BUMPER_TRIGGER, 1);  

  while(digitalRead(MOTOR_SWITCH_PIN) == 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  int motorSwitchPin = digitalRead(MOTOR_SWITCH_PIN);
  int leftBumper = digitalRead(LEFT_BUMPER_TRIGGER);
  int rightBumper = digitalRead(RIGHT_BUMPER_TRIGGER);
  int leftIR = analogRead(LEFT_IR);
  int rightIR = analogRead(RIGHT_IR);

  
  Serial.print(leftIR);
  Serial.print("\t");
  Serial.print(rightIR);
  Serial.print("\t");
  Serial.print(leftBumper);
  Serial.print("\t");
  Serial.println(rightBumper);
  
  //checking for hits - checking twice - solved weird behaviour
  if(leftBumper == 0 && digitalRead(LEFT_BUMPER_TRIGGER) == 0) {
    Serial.println("left");
    if(linePos == 1) {
      goAroundLeft();
    } else {
      goAroundRight();  
    }
    //alreadyTurned = 1;
  } else if(rightBumper == 0 && digitalRead(RIGHT_BUMPER_TRIGGER) == 0) {
    Serial.println("right");
    if(linePos == -1) {
      goAroundRight();
    } else {
      goAroundLeft();
    }
    //alreadyTurned = 1;
  }
  
  //line following  
  if(leftIR < 1000 && rightIR < 930) {
    //neither sensor is on the line
    servoStraight();
    /*
    if(linePos == 0) {
      servoStraight();
    } else if(alreadyTurned == 0) {
      if(linePos == -1) {
        servoLeft();
        delay(500);
        alreadyTurned = 1;
      } else {
        servoRight();
        delay(500);
        alreadyTurned = 1;
      }
    } else {
      servoStraight();
    }
    */
  } else if(leftIR > 1000) {
    //left sensor is on the line
    servoLeft();
  } else if(rightIR > 800) {
    //right sensor is on the line
    servoRight();
  } else {
    //neither left nor right sensor is on the line
    servoStraight();
  }
  
}

void servoStop() {
  leftServo.write(90);
  rightServo.write(92);
}

void servoStraight() {
  leftServo.write(100);
  rightServo.write(82);
}

void servoBack() {
  leftServo.write(80);
  rightServo.write(102);
}

void servoRight() {
  leftServo.write(102);
  rightServo.write(104);
}

void servoLeft() {
  leftServo.write(80);
  rightServo.write(82);
}

//steps to go around an obstacle from left side
void goAroundLeft() {
  servoBack();
  delay(2000);
  servoLeft();
  delay(700);
  servoStraight();
  delay(2500);
  servoRight();
  delay(700);
  servoStraight();
  delay(2500);
  servoRight();
  delay(700);
  servoStraight();
}

//steps to go around an obstale from right side
void goAroundRight() {
  servoBack();
  delay(2000);
  servoRight();
  delay(700);
  servoStraight();
  delay(2500);
  servoLeft();
  delay(700);
  servoStraight();
  delay(2500);
  servoLeft();
  delay(700);
  servoStraight();
}

