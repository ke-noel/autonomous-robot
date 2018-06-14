#include "Servo.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
/* Program for self-driving robot
 * Senses objects using liDAR and ultrasonic and avoids them,
 * prioritizing ultrasonic. A bubble rebound algorithm is used
 * with the ultrasonic sensor data to choose the path with the
 * fewest obstacles. The liDAR (TFMini) rangefinder is used to
 * determine speed; the further away the object detected, the 
 * faster the speed.
 *
 * The ultrasonic detection zone will be referred to as the near zone.
 * The liDAR detection zone will be referred to as the far zone.
 */

/* LiDAR pins
  black - GND (to Arduino GND)
  red --- 5V (to Arduino 5V)
  white - liDAR RX (to Arduino TX)
  green - liDAR TX (to Arduino RX)  
*/

volatile int liDARdist; // the reading from the liDAR

// Ultrasonic pin definitions
// arranged at 45, 20 and 5 degrees from the left and right
#define ultraTrig44L 24
#define ultraEcho44L 22
#define ultraTrig21L 28
#define ultraEcho21L 26
#define ultraTrig4L 32
#define ultraEcho4L 30
#define ultraTrig7R 36
#define ultraEcho7R 34
#define ultraTrig21R 40
#define ultraEcho21R 38
#define ultraTrig47R 44
#define ultraEcho47R 42

// Compass module pins
#define gyroSDA 20 // must be these ports; arduino default
#define gyroSCL 21 // must be these ports; arduino default

const int MPU_addr=0x68;
const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265; int maxVal=402;
double x; double y; double z;

// Victor888 motor controller pins
#define leftMotorPin 2 // PWM
#define rightMotorPin 3 // PWM
Servo leftMotor;
Servo rightMotor;
float leftMotorSpeed;
float rightMotorSpeed;

// Ultrasonic sensors distances from different angles
// ex. dist45R is the distance measured by the ultrasonic at 45 degrees on the right side
float dist47R;
float dist21R;
float dist7R;
float dist4L;
float dist21L;
float dist44L;

bool isNearZone; // aka did any ultrasonic detect something within 75cm?

void setup() {
  Serial1.begin(115200); // HW serial for liDAR
  Serial.begin(9600); // USB to computer serial
  delay(100);

  // Setup TFMini for standard output mode
  Serial1.write(0x42);
  Serial1.write(0x57);
  Serial1.write(0x02);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x01);
  Serial1.write(0x06);
  
  // Setup ultrasonics
  pinMode(ultraEcho47R, INPUT);
  pinMode(ultraTrig47R, OUTPUT);
  
  pinMode(ultraEcho21R, INPUT);
  pinMode(ultraTrig21R, OUTPUT);
  
  pinMode(ultraEcho7R, INPUT);
  pinMode(ultraTrig7R, OUTPUT);
  
  pinMode(ultraEcho4L, INPUT);
  pinMode(ultraTrig4L, OUTPUT);
  
  pinMode(ultraEcho21L, INPUT);
  pinMode(ultraTrig21L, OUTPUT);
  
  pinMode(ultraEcho44L, INPUT);
  pinMode(ultraTrig44L, OUTPUT);

  // Setup compass module
    Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  
  AcX=Wire.read()<<8|Wire.read();     
  AcZ=Wire.read()<<8|Wire.read();  
  GyX=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
  
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(AcX);
  Serial.print(" | Y = "); Serial.println(AcZ); 
  
  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.println(GyZ);
  Serial.println(" ");
  delay(500);
  AcX=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);

x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI); y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI); z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

Serial.print("AngleX= "); Serial.println(x);
Serial.print("AngleY= "); Serial.println(z);
Serial.println("-----------------------------------------");
delay(400);
  
  //Setup motor controllers
  leftMotorSpeed = 1500;
  rightMotorSpeed = 1500;
  leftMotor.attach(leftMotorPin);
  leftMotor.writeMicroseconds(leftMotorSpeed); //stopped
  rightMotor.attach(rightMotorPin);
  rightMotor.writeMicroseconds(rightMotorSpeed); //stopped
}

double readLiDAR() {
  // Returns the current distance read by the liDAR
  /* Input format for TFMini liDAR
    1) 0x59
    2) 0x59
    3) Dist_L (low 8bit)
    4) Dist_H (high 8bit)
    5) Strength_L (low 8bit)
    6) Strength_H (high 8bit)
    7) Reserved bytes
    8) Original signal quality degree
    9) Checksum parity bit (low 8bit), checksum = Byte1 + Byte2 + ... + Byte8 
  */
  while(true)
  {
    while(Serial1.available()>=9) { // there should be 9 bytes of input
      if((0x59 == Serial1.read()) && (0x59 == Serial1.read())) { // byte 1 and 2
        unsigned int t1 = Serial1.read(); // byte 3 (Dist_L)
        unsigned int t2 = Serial1.read(); // byte 4 (Dist_H)
        t2 <<= 8; // shift one byte left
        t2 += t1;
        liDARdist = t2;
        for(int i=0; i<5; i++)Serial1.read(); // byte 5, 6, 7, 8, 9 are ignored
      }    
    }
    return liDARdist; // in cm
  }
}

float readUltrasonic(int echoPin, int trigPin) {
  // Return the current distance reading from the specified ultrasonic
  // Flag isNearZone if something's within 75cm
  digitalWrite(echoPin + 1, LOW);
  delayMicroseconds(2);
  digitalWrite(echoPin + 1, HIGH);
  delayMicroseconds(10);
  digitalWrite(echoPin + 1, LOW);
  float duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.0345 / 2; // in cm
  if (distance < 3 || distance > 275) { // ultrasonic rated accurate from 3 to 300 cm
    distance = 0;
  } else if (distance < 75) {
    isNearZone = true;
  }
  return distance;
}

void updateUltrasonic() {
  // Grab the new distance readings for all of the ultrasonics
  dist47R = readUltrasonic(ultraEcho47R, ultraEcho47R);
  dist21R = readUltrasonic(ultraEcho21R, ultraEcho21R);
  dist7R = readUltrasonic(ultraEcho7R, ultraEcho7R);
  dist4L = readUltrasonic(ultraEcho4L, ultraEcho4L);
  dist21L = readUltrasonic(ultraEcho21L, ultraEcho21L);
  dist44L = readUltrasonic(ultraEcho44L, ultraEcho44L);
}

float getReboundAngle() { // left is positive, right is negative
  // Bubble rebound algorithm using ultrasonic data to return the angle to 
  // get to the path with the fewest/furthest obstacles
  float angle_dist = (44.32 * dist44L) - (46.55 * dist47R) + (21 * (dist21L - dist21R)) + (4.19 * dist4L) - (7.45 * dist7R);
  float sum = dist44L + dist21L + dist4L + dist7R + dist21R + dist47R;
  return angle_dist / sum;
}

void setMotorSpeed(double leftSpeed, double rightSpeed) {
  /* Control the motor speed using the Victor888's
  *  1300   - full speed backwards
  *  1500 - stopped
  *  1700 - full speed forwards
  */
  leftMotor.writeMicroseconds(leftSpeed);
  rightMotor.writeMicroseconds(rightSpeed);
}

void stopMotors() {
  setMotorSpeed(1500, 1500);
}

void updateSpeed(int distance) { // distance is in cm
  /* If nothing is detected in near zone,
     use the liDAR-detected distance to determine speed.
     The further the nearest detected obstacle, the faster
     the speed up to a max of ___. <-- check with motor controller
  */
  if (distance > 1000 && (leftMotorSpeed + 5) < 1700) {
    leftMotorSpeed+=5;
    rightMotorSpeed+=5;
  } else if (distance < 500 && (leftMotorSpeed - 5) > 1300) {
    leftMotorSpeed-=5;
    rightMotorSpeed-=5;
  }
}

void updateCompassModule() {
 
  
float getHeading() {
  updateCompassModule();
  // rando mathy stuff LUUUUUUUCCCCCCCCCCCC
}

void loop() {
  updateUltrasonic();
  if (isNearZone) { // an object is detected within 75cm
    float reboundAngle = getReboundAngle();
    float startingHeading = getHeading();
    float currentHeading = startingHeading;
    
      if (reboundAngle > 0) { // must turn left
        leftMotorSpeed = 1600;
        rightMotorSpeed = 1300;
      } else { //turn right
        leftMotorSpeed = 1300;
        rightMotorSpeed = 1600;
      }
    
      while (abs(currentHeading - startingHeading) > reboundAngle) {
          setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
          delay(50);
          stopMotors();
          currentHeading = getHeading();
      }
  } else { // There is nothing within 75cm
    // Increase or decrease speed based on the closest object detected by the liDAR
    updateSpeed(readLiDAR());
    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  }
  delay(100);
}
