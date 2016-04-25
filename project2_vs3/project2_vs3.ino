#include <Servo.h>

#define RIGHT_BUMPER_TRIGGER 2
#define LEFT_BUMPER_TRIGGER 3
#define LEFT_IR A1
#define RIGHT_IR A0
#define CALIBRATE_BUTTON_PIN 4
#define MOTOR_SWITCH_PIN 7
#define LEFT_SERVO_PIN 5
#define RIGHT_SERVO_PIN 6
#define IR_SERVO_PIN 11
#define LEFT_LED_PIN 13
#define RIGHT_LED_PIN 12
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
  rightServo.attach(RIGHT_SERVO_PIN);
  servoStop();

  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
  
  pinMode(MOTOR_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LEFT_BUMPER_TRIGGER, INPUT_PULLUP);
  pinMode(RIGHT_BUMPER_TRIGGER, INPUT_PULLUP);
  
  digitalWrite(MOTOR_SWITCH_PIN, 1);
  digitalWrite(LEFT_BUMPER_TRIGGER, 1);
  digitalWrite(RIGHT_BUMPER_TRIGGER, 1);  

  /*while(digitalRead(MOTOR_SWITCH_PIN) == 1){
    int leftIR = analogRead(LEFT_IR);
    int rightIR = analogRead(RIGHT_IR);
    Serial.print(leftIR);
    Serial.print("\t");
    Serial.print(rightIR);
    Serial.println();
  }*/
}

void loop() {
    driveRobot();
}

uint16_t lowCutCal = 200;
uint16_t highCutCal = 200;
uint8_t printDrive = 1;

#define GOINGRIGHT 1
#define GOINGSTRAIGHT 2
#define GOINGLEFT 3
uint8_t lastState = GOINGSTRAIGHT;

void driveRobot(){
  int rightVal = analogRead(RIGHT_IR);
  int leftVal = analogRead(LEFT_IR);
  
  if((rightVal < lowCutCal) && (leftVal > highCutCal)){
    servoStraight();
    lastState = GOINGSTRAIGHT;
    if(printDrive == 1) { Serial.println("Going straight A"); }
  }else if((rightVal < lowCutCal) && (leftVal < lowCutCal)){
    if(lastState == GOINGSTRAIGHT){
      servoLeft();
      lastState = GOINGLEFT;
      if(printDrive == 1) { Serial.println("Going left B"); }
    }else if(lastState == GOINGRIGHT){
      servoRight();
      lastState = GOINGRIGHT;
      if(printDrive == 1) { Serial.println("Going right C"); }
    }else if(lastState == GOINGLEFT){
      servoLeft();
      lastState = GOINGLEFT;
      if(printDrive == 1) { Serial.println("Going left D"); }
    }
  }else if((rightVal > highCutCal) && (leftVal < lowCutCal)){
    //drive right
    servoRight();
    lastState = GOINGRIGHT;
    if(printDrive == 1) { Serial.println("Going right E"); }
  }else if((rightVal > highCutCal) && (leftVal > highCutCal)){
    //this else statement should never happen unless the line is thick enough
    //to hold both of the reflectors
    servoRight();
    lastState = GOINGRIGHT;
    if(printDrive == 1) { Serial.println("Going right F"); }
  }else{
    if(printDrive == 1) { Serial.println("driving neutral zone, keep doing same as before G"); }
  }
}

#define LEFTMAX 180
#define RIGHTMIN 0
int servoRightVal = 90;
int servoLeftVal = 90;

void servoStop() {
  leftServo.write(90);
  rightServo.write(92);
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, LOW);    
}

void servoStraight() {
  digitalWrite(LEFT_LED_PIN, HIGH);
  digitalWrite(RIGHT_LED_PIN, HIGH);    
  if(servoLeftVal < LEFTMAX){
    servoLeftVal = servoLeftVal + 1;
    leftServo.write(servoLeftVal);
  }
  if(servoRightVal > RIGHTMIN){
    servoRightVal = servoRightVal - 1;
    rightServo.write(servoRightVal);
  }
}

void servoBack() {
  //leftServo.write(80);
  //rightServo.write(102);
}

void servoRight() {
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, HIGH);    
  if(servoRightVal < 89){ // 90 down to 0 are foward
    servoRightVal = servoRightVal + 1;
    rightServo.write(servoRightVal);
  }
  if(servoLeftVal < LEFTMAX){
    servoLeftVal = servoLeftVal + 1;
    leftServo.write(servoLeftVal);
  }
}

void servoLeft() {
  digitalWrite(LEFT_LED_PIN, HIGH);
  digitalWrite(RIGHT_LED_PIN, LOW);     
  if(servoLeftVal > 91){ //90 up to 180 are forward
    servoLeftVal = servoLeftVal - 1;
    leftServo.write(servoLeftVal);
  }
  if(servoRightVal > RIGHTMIN){
    servoRightVal = servoRightVal - 1;
    rightServo.write(servoRightVal);
  }
}

