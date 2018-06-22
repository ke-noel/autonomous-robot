/* Program for self-driving robot
 * Senses objects using liDAR and ultrasonic and avoids them,
 * prioritizing ultrasonic. A bubble rebound algorithm is used
 * with the ultrasonic sensor data to choose the path with the
 * fewest obstacles. The liDAR (TFMini) rangefinder is used to
 * determine speed; the further away the object detected, the 
 * faster the speed.
 *
 * Up to 75 cm away from the robot will be referred to as the near zone.
 */

#include "Servo.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

/* LiDAR pins
  GND -- Arduino GND
  5V  -- Arduino 5V
  RX  -- Arduino TX1 18
  TX  -- Arduino RX1 19
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

/* Compass module pins
  GND -- Arduino GND
  5V  -- Arduino 5V)
  SDA -- Arduino SDA 20
  SCL -- Arduino SCL 21
*/

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
// ex. dist47R is the distance measured by the ultrasonic at 47 degrees on the right side
float dist47R;
float dist21R;
float dist7R;
float dist4L;
float dist21L;
float dist44L;

bool isNearZone; // aka did any ultrasonic detect something within 75cm?

void setup() {
  Serial.begin(9600); // USB to computer serial
  Serial1.begin(115200); // HW serial for liDAR
  Serial.println("-----------STARTING UP-----------");
  
  delay(100);
  
  Serial.println("Setting up TFMini for standard output mode...");
  Serial1.write(0x42);
  Serial1.write(0x57);
  Serial1.write(0x02);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x01);
  Serial1.write(0x06);
  
  Serial.print("Setting up ultrasonic pins...");
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

  Serial.println("Setting up compass module...");
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  
  Serial.println("Setting up motor controllers...");
  leftMotor.attach(leftMotorPin);
  leftMotor.writeMicroseconds(1500); //stopped
  rightMotor.attach(rightMotorPin);
  rightMotor.writeMicroseconds(1500); //stopped
  
  Serial.println("Program ready.");
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
  while(true) // stay in loop until measurement is taken
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
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
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
  
  Serial.print("Ultrasonic distance: ");
  Serial.print(dist44L); Serial.print("cm | "); 
  Serial.print(dist21L); Serial.print("cm | "); 
  Serial.print(dist4L); Serial.print("cm | ");
  Serial.print(dist7R); Serial.print("cm | ");
  Serial.print(dist21R); Serial.print("cm | ");
  Serial.print(dist47R); Serial.println("cm");
}

float getReboundAngle() { // left is positive, right is negative
  // Bubble rebound algorithm using ultrasonic data to return the angle to 
  // get to the path with the fewest/furthest obstacles
  float angle_dist = (44.32 * dist44L) - (46.55 * dist47R) + (21 * (dist21L - dist21R)) + (4.19 * dist4L) - (7.45 * dist7R);
  float sum = dist44L + dist21L + dist4L + dist7R + dist21R + dist47R;
  return angle_dist / sum;
}

void setMotorSpeed(double left, double right) {
  /* Control the motor speed using the Victor888's
  *  1300   - full speed backwards
  *  1500 - stopped
  *  1700 - full speed forwards
  */
  leftMotor.writeMicroseconds(left);
  rightMotor.writeMicroseconds(right);
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
  // needs to be moved here
}

float getHeading() {
    Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
   
  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.println(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
  Serial.println(" ");
  delay(500);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);

x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI); y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI); z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

Serial.print("AngleX= "); Serial.println(x);
Serial.print("AngleY= "); Serial.println(y);
Serial.print("AngleZ= "); Serial.println(z);
Serial.println("-----------------------------------------");
delay(400);
}

void loop() {


  updateUltrasonic();
  if (isNearZone) { // an object is detected within 75cm
    float reboundAngle = getReboundAngle();
    float startingHeading = getHeading();
    float currentHeading = startingHeading;
    
    Serial.print("Rebound angle: "); Serial.println(reboundAngle);
    
    if (reboundAngle > 0) { // must turn left
      leftMotorSpeed = 1600;
      rightMotorSpeed = 1300;
    } else { //turn right
      leftMotorSpeed = 1300;
      rightMotorSpeed = 1600;
    }
    while (abs(currentHeading - startingHeading) < reboundAngle) {
      // turn until the rebound angle is reached
      Serial.print("Current angle: "); Serial.println(currentHeading - startingHeading);
      setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
      delay(50);
      stopMotors();
      currentHeading = getHeading();
    }
  } else { // there's nothing detected within 75cm
    // Increase or decrease speed based on the closest object detected by the liDAR
    updateSpeed(readLiDAR());
    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  }
  delay(100);
}
