#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>

#define ONBOARD_BUTTON_PIN 4
#define LEFT_SERVO_PIN 5
#define RIGHT_SERVO_PIN 6
#define IR_SERVO_PIN 11
#define LEFT_LED_PIN 13
#define RIGHT_LED_PIN 12
#define VCNL4000_ADDRESS 0x13  // 0x26 write, 0x27 read
#define LEFT_IR_PIN 3
#define RIGHT_IR_PIN 2
// 1 is on the line and 0 is off the line

// VCNL4000 Register Map
#define COMMAND_0 0x80  // starts measurments, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x11
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define AMBIENT_RESULT_MSB 0x85  // high byte of ambient light measure
#define AMBIENT_RESULT_LSB 0x86  // low byte of ambient light measure
#define PROXIMITY_RESULT_MSB 0x87  // High byte of proximity measure
#define PROXIMITY_RESULT_LSB 0x88  // low byte of proximity measure
#define PROXIMITY_FREQ 0x89  // Proximity IR test signal freq, 0-3
#define PROXIMITY_MOD 0x8A  // proximity modulator timing




Servo leftServo;
Servo rightServo;

//ambient is not really used, concentrating on proximity
long proximityCalibrationValue;
long ambientCalibrationValue;
long ambientValue;
long proximityValue;

/*
 * global time variables - used when turning back on the line
 *  - the robot can find the line while turning - main loop gets that
 */
uint32_t t0;
uint32_t t1;

//saves the state - if the robot is going back to the line from avoiding the obstacle == 1
uint8_t goingAround = 0;

void setup() {
  pinMode(ONBOARD_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(1, ONBOARD_BUTTON_PIN);

  
  Serial.begin(9600);
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  servoStop();

  Wire.begin();
  readCalibrationFromEeprom();
  proximityInit();

  //calibrate the sensor
  Serial.println("Setting new calibration values...");
  ambientCalibrationValue = readAmbient();
  proximityCalibrationValue = readProximity();
  writeCalibrationToEeprom(ambientCalibrationValue, proximityCalibrationValue);

  //print current calibration values
  Serial.print("Calibration values are: ");
  Serial.print(ambientCalibrationValue);
  Serial.print(", ");
  Serial.println(proximityCalibrationValue);
}

void servoStraight() {
  leftServo.write(180);
  rightServo.write(1);
}

void servoStop() {
  leftServo.write(92);
  rightServo.write(90);
}

void servoLeft() {
  leftServo.write(90);
  rightServo.write(46);
}

void servoRight() {
  leftServo.write(135);
  rightServo.write(91);    
}

void servoSlightRight() {
  leftServo.write(131);
  rightServo.write(75);
}

//steps to go around an obstacle from left side
void goAroundLeft() {
  Serial.println("Start of goAroundLeft");
  servoLeft();
  t0 = millis();
  t1 = 0;

  //keep turning left until the obstacle is out of the way (+200ms to make space for the rest of the robot to go around)
  while(readProximity() > proximityCalibrationValue);
  delay(400);
  t0 = millis() - t0;
  servoStraight();
  delay(1000);
  t1 = millis();
  servoRight();
  delay(t0);
  servoStraight();
  delay(500);

  //turn to be facing the line again
  servoRight();
  //don't want a sharp angle to find the line again - turning right 100ms shorter time
  delay(t0 - 100);
  goingAround = 1;
  Serial.println(goingAround);
  Serial.println("End of goAroundLeft");
}

//steps to go around an obstale from right side
void goAroundRight() {
  Serial.println("Start of goAroundRight");
  servoRight();
  t0 = millis();
  t1 = 0;

  //keep turning left until the obstacle is out of the way (+200ms to make space for the rest of the robot to go around)
  while(readProximity() > proximityCalibrationValue);
  delay(400);
  t0 = millis() - t0;
  servoStraight();
  delay(1000);
  t1 = millis();
  servoLeft();
  delay(t0);
  servoStraight();
  delay(500);

  //turn to be facing the line again
  servoLeft();
  //don't want a sharp angle to find the line again - turning left 100ms shorter time
  delay(t0 - 100);
  goingAround = 1;
  Serial.println(goingAround);
  Serial.println("End of goAroundRight");
}

// A function that reads the value from the proximity sensor and outputs a number based on if it should move away from the wall or towards it
// A 1 means it should turn away, and a 2 means it should turn towards
uint8_t checkProximity() {
    proximityValue = readProximity();
    if((proximityCalibrationValue) <= proximityValue){// && ambientValue < ambientCalibrationValue) {
      // Turn away from wall
      return 1;
    } else if((proximityCalibrationValue) > proximityValue){// && ambientValue > ambientCalibrationValue) {
      // Turn towards wall
      return 2;
    }
}

// Write the four bytes to the EEPROM for calibration values.
void writeCalibrationToEeprom(long ambientValue, long proximityValue) {
  uint8_t address = 0;
  
  proximityCalibrationValue = proximityValue;
  ambientCalibrationValue = ambientValue;
  
  byte proximityFour = (proximityCalibrationValue & 0xFF);
  byte proximityThree = ((proximityCalibrationValue >> 8) & 0xFF);
  byte proximityTwo = ((proximityCalibrationValue >> 16) & 0xFF);
  byte proximityOne = ((proximityCalibrationValue >> 24) & 0xFF);

  byte ambientFour = (ambientCalibrationValue & 0xFF);
  byte ambientThree = ((ambientCalibrationValue >> 8) & 0xFF);
  byte ambientTwo = ((ambientCalibrationValue >> 16) & 0xFF);
  byte ambientOne = ((ambientCalibrationValue >> 24) & 0xFF);

  EEPROM.write(address, proximityFour);
  EEPROM.write(address + 1, proximityThree);
  EEPROM.write(address + 2, proximityTwo);
  EEPROM.write(address + 3, proximityOne);

  EEPROM.write(address + 4, ambientFour);
  EEPROM.write(address + 5, ambientThree);
  EEPROM.write(address + 6, ambientTwo);
  EEPROM.write(address + 7, ambientOne);    
  
}

// Read the four bytes from the EEPROM for the calibration values. 
void readCalibrationFromEeprom() {
  int address = 0;
  
  long proximityFour = EEPROM.read(address);
  long proximityThree = EEPROM.read(address + 1);
  long proximityTwo = EEPROM.read(address + 2);
  long proximityOne = EEPROM.read(address + 3);

  proximityCalibrationValue = ((proximityFour << 0) & 0xFF) + ((proximityThree << 8) & 0xFFFF) + ((proximityTwo << 16) & 0xFFFFFF) + ((proximityOne << 24) & 0xFFFFFFFF);

  long ambientFour = EEPROM.read(address + 4);
  long ambientThree = EEPROM.read(address + 5);
  long ambientTwo = EEPROM.read(address + 6);
  long ambientOne = EEPROM.read(address + 7);

  ambientCalibrationValue = ((ambientFour << 0) & 0xFF) + ((ambientThree << 8) & 0xFFFF) + ((ambientTwo << 16) & 0xFFFFFF) + ((ambientOne << 24) & 0xFFFFFFFF);  
  
}

void proximityInit() {
  /* Test that the device is working correctly */
  byte temp = readByte(PRODUCT_ID);
  if (temp != 0x11)  // Product ID Should be 0x11
  {
    Serial.print("Something's wrong. Not reading correct ID: 0x");
    Serial.println(temp, HEX);
  }
  else
    Serial.println("VNCL4000 Online...");
  
  /* Now some VNCL400 initialization stuff
     Feel free to play with any of these values, but check the datasheet first!*/
  writeByte(AMBIENT_PARAMETER, 0x0F);  // Single conversion mode, 128 averages
  writeByte(IR_CURRENT, 20);  // Set IR current to 200mA
  writeByte(PROXIMITY_FREQ, 2);  // 781.25 kHz
  writeByte(PROXIMITY_MOD, 0x81);  // 129, recommended by Vishay
}

// Everything after this was taken from the example on the course website. 

// readProximity() returns a 16-bit value from the VCNL4000's proximity data registers
unsigned int readProximity()
{
  unsigned int data;
  byte temp;
  
  temp = readByte(COMMAND_0);
  writeByte(COMMAND_0, temp | 0x08);  // command the sensor to perform a proximity measure
  
  while(!(readByte(COMMAND_0)&0x20)) 
    ;  // Wait for the proximity data ready bit to be set
  data = readByte(PROXIMITY_RESULT_MSB) << 8;
  data |= readByte(PROXIMITY_RESULT_LSB);
  
  return data;
}

// readAmbient() returns a 16-bit value from the VCNL4000's ambient light data registers
unsigned int readAmbient()
{
  unsigned int data;
  byte temp;
  
  temp = readByte(COMMAND_0);
  writeByte(COMMAND_0, temp | 0x10);  // command the sensor to perform ambient measure
  
  while(!(readByte(COMMAND_0)&0x40)) 
    ;  // wait for the proximity data ready bit to be set
  data = readByte(AMBIENT_RESULT_MSB) << 8;
  data |= readByte(AMBIENT_RESULT_LSB);
  
  return data;
}


// writeByte(address, data) writes a single byte of data to address
void writeByte(byte address, byte data)
{
  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

// readByte(address) reads a single byte of data from address
byte readByte(byte address)
{
  byte data;
  
  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(VCNL4000_ADDRESS, 1);
  while(!Wire.available())
    ;
  data = Wire.read();

  return data;
}

/*
 * 0 = stopped
 * 1 = straight
 * 2 = left
 * 3 = right
 */
uint8_t previousServoDirection = 0; 
uint8_t mode = 0;
uint8_t button = 0;
//0 = goAroundLeft
//1 = goAroundRight
uint8_t goAround = 0;

void loop() {
  button = digitalRead(ONBOARD_BUTTON_PIN);

  //button was pressed, can now switch mode
  //mode 0 = dont do anything
  //mode 1 = goAroundLeft()
  //mode 2 = goAroundRight()
  if(button == 0){
    mode++;
    delay(500);
    if(mode==0){
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
    }else if(mode==1){
      digitalWrite(LEFT_LED_PIN, HIGH);
      digitalWrite(RIGHT_LED_PIN, LOW);
    }else if(mode==2){
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, HIGH);
    }else{
      mode = 0;
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
    }
  }
  
  if(mode==0){
    //dont drive
    servoStop();
  }else if(mode==1){
    //drive, goAroundLeft
    drive();
    goAround = 0;
  }else if(mode==2){
    //drive, goAroundRight
    drive();
    goAround = 1;
  }
  
}

void drive(){
  proximityValue = readProximity();
  
  uint8_t leftIrValue = digitalRead(LEFT_IR_PIN);
  uint8_t rightIrValue = digitalRead(RIGHT_IR_PIN);
  
  if(proximityValue > proximityCalibrationValue) {
    int proximityCheck1 = readProximity();
    int proximityCheck2 = readProximity();
    if(proximityCheck1 >= proximityValue && proximityCheck2 >= proximityCheck1) {
      Serial.println("Obstacle"); 
      if(goAround == 0){
        goAroundLeft();  
      }else if(goAround == 1){
        goAroundRight();
      }
             
    } else {
      Serial.println("Free");
    }
    Serial.println(goingAround);
  }
    

  if((leftIrValue == 1) && (rightIrValue == 1)) { // on the line, go straight
    previousServoDirection = 1;
    servoStraight();
    goingAround = 0;
  } else if((leftIrValue == 1) && (rightIrValue == 0)) {
    previousServoDirection = 2;
    servoLeft();    
    goingAround = 0;
  } else if((leftIrValue == 0) && (rightIrValue == 1)) {
    previousServoDirection = 3;
    servoRight();
    goingAround = 0;
  } else {
    //neither one of the sensors is on the line
    servoStraight();
  }
}

