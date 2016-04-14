#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>

#define CALIBRATE_BUTTON_PIN 4
#define MOTOR_SWITCH_PIN 7
#define LEFT_SERVO_PIN 6
#define RIGHT_SERVO_PIN 5
#define IR_SERVO_PIN 11
#define LEFT_LED_PIN 12
#define RIGHT_LED_PIN 13
#define VCNL4000_ADDRESS 0x13  // 0x26 write, 0x27 read


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

#define ONECM_PROXIMITY_VALUE 2625
#define ONECM_AMBIENT_VALUE 33
#define TWOCM_PROXIMITY_VALUE 2560
#define TWOCM_AMBIENT_VALUE 35
#define FOURCM_PROXIMITY_VALUE 2490
#define FOURCM_AMBIENT_VALUE 64
//#define SIXCM_PROXIMITY_VALUE 2465
//#define SIXCM_AMBIENT_VALUE 169
#define SIXCM_PROXIMITY_VALUE 2440
#define SIXCM_AMBIENT_VALUE 70
#define EIGHTCM_PROXIMITY_VALUE 2450
#define EIGHTCM_AMBIENT_VALUE 233 // seems to lower over time for some reason
#define TENCM_PROXIMITY_VALUE 2445
#define TENCM_AMBIENT_VALUE 280

long proximityCalibrationValue;
long ambientCalibrationValue;

long ambientValue;
long proximityValue;
Servo leftServo;
Servo rightServo;
Servo irServo;


void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  Wire.begin();
  readCalibrationFromEeprom();
  proximityInit();
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  irServo.attach(IR_SERVO_PIN);
  servoStop();
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_SWITCH_PIN, INPUT_PULLUP);
  digitalWrite(1, CALIBRATE_BUTTON_PIN);
  digitalWrite(1, MOTOR_SWITCH_PIN);

  if(digitalRead(CALIBRATE_BUTTON_PIN) == 0) {
    Serial.println("Setting new calibration values...");
    ambientCalibrationValue = readAmbient();
    proximityCalibrationValue = readProximity();
    writeCalibrationToEeprom(ambientCalibrationValue, proximityCalibrationValue);
  }

  Serial.print("Calibration values are: ");
  Serial.print(ambientCalibrationValue);
  Serial.print(", ");
  Serial.println(proximityCalibrationValue);
  
  irServo.write(45);
  delay(500);
  irServo.detach();



  }

void loop() {
  int motorSwitchPin = digitalRead(MOTOR_SWITCH_PIN);
    ambientValue = readAmbient();
    proximityValue = readProximity();
    Serial.print(ambientValue, DEC);
    Serial.print("\t");
    Serial.println(proximityValue, DEC);  
    
  if(motorSwitchPin == 0) {
    int proximityCheck1 = checkProximity();
    //delay(10);
    int proximityCheck2 = checkProximity();
    if(proximityCheck1 == proximityCheck2) {
      if(proximityCheck1 == 1) {
        servoSlightRight();        
      } else if(proximityCheck1 == 2) {
        servoSlightLeft();
      }
    }

  } else {
    servoStop();
  }
}

uint8_t checkProximity() {
    int allowedDifference = 10;
    ambientValue = readAmbient();
    proximityValue = readProximity();
    if((proximityCalibrationValue) <= proximityValue){// && ambientValue < ambientCalibrationValue) {
      // Turn away from wall
      //servoSlightRight();
      return 1;
    } else if((proximityCalibrationValue) > proximityValue){// && ambientValue > ambientCalibrationValue) {
      // Turn towards wall
      //servoSlightLeft();
      return 2;
    }
}

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

void servoStraight() {
  Serial.println("Straight");
  leftServo.write(100); // increase for faster
  rightServo.write(83);// decrease for faster
  digitalWrite(LEFT_LED_PIN, HIGH);
  digitalWrite(RIGHT_LED_PIN, HIGH);
}

void servoSlightLeft() {
  Serial.println("Left");
  //leftServo.write(100); // increase for faster
  //rightServo.write(75); // decrease for faster
  //leftServo.write(98);
  //rightServo.write(82);
  leftServo.write(95);
  rightServo.write(82);
  digitalWrite(LEFT_LED_PIN, HIGH);
  digitalWrite(RIGHT_LED_PIN, LOW);  
}

void servoSlightRight() {
  Serial.println("Right");
  //leftServo.write(105); // increase for faster
  //rightServo.write(82); // decrease for faster
  //leftServo.write(100);
  //rightServo.write(84);
  leftServo.write(100);
  rightServo.write(87);
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, HIGH);
}

void servoStop() {
  //Serial.println("Stop");
  leftServo.write(90);
  rightServo.write(92);
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, LOW);
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
