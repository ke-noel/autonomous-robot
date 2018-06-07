#include "Servo.h"

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

// Ultrasonic pin definitions
// arranged at 45, 20 and 5 degrees from the left and right
#define ultraTrig45R 24
#define ultraEcho45R 22
#define ultraTrig20R 28
#define ultraEcho20R 26
#define ultraTrig5R 32
#define ultraEcho5R 30
#define ultraTrig5L 36
#define ultraEcho5L 34
#define ultraTrig20L 40
#define ultraEcho20L 38
#define ultraTrig45L 44
#define ultraEcho45L 42

// Compass module pins
#define gyroSDA 20 // must be these ports; arduino default
#define gyroSCL 21 // must be these ports; arduino default

// Victor888 motor controller pins
#define leftMotorPin 2 // PWM
#define rightMotorPin 3 // PWM
Servo leftMotor;
Servo rightMotor;
float speed;

// Ultrasonic sensors distances from different angles
// ex. dist45R is the distance measured by the ultrasonic at 45 degrees on the right side
float dist45R;
float dist20R;
float dist5R;
float dist5L;
float dist20L;
float dist45L;

bool isNearZone; // aka did any ultrasonic detect something within 75cm?

volatile int liDARdist; // the reading from the liDAR

int16_t GyX,GyY,GyZ // variables for gyro raw data
const int MPU=0x68;

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
  pinMode(ultraEcho45R, INPUT);
  pinMode(ultraTrig45R, OUTPUT);
  
  pinMode(ultraEcho20R, INPUT);
  pinMode(ultraTrig20R, OUTPUT);
  
  pinMode(ultraEcho5R, INPUT);
  pinMode(ultraTrig5R, OUTPUT);
  
  pinMode(ultraEcho5L, INPUT);
  pinMode(ultraTrig5L, OUTPUT);
  
  pinMode(ultraEcho20L, INPUT);
  pinMode(ultraTrig20L, OUTPUT);
  
  pinMode(ultraEcho45R, INPUT);
  pinMode(ultraTrig45R, OUTPUT);

  // Setup compass module
  Wire.begin();
  Wire.beginTransmission(MPU); // begins a trnasmission to the GY-521
  Wire.write(0x6B); 
  Wire.write(0); // set to zero    
  Wire.endTransmission(true);

  //Setup motor controllers
  speed = 1500;
  leftMotor.attach(leftMotorPin);
  leftMotor.writeMicroseconds(speed); //stopped
  rightMotor.attach(rightMotorPin);
  rightMotor.writeMicroseconds(speed); //stopped
}

double readLiDAR() {
  // Returns the current distance read by the liDAR
  // SHOULD BE IN ITS OWN THREAD
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
  if(Serial1.available()>=9) { // there should be 9 bytes of input
    if((0x59 == Serial1.read()) && (0x59 == Serial1.read())) { // byte 1 and 2
      unsigned int t1 = Serial1.read(); // byte 3 (Dist_L)
      unsigned int t2 = Serial1.read(); // byte 4 (Dist_H)
      t2 <<= 8; // shift one byte left
      t2 += t1;
      liDARval = t2;
      for(int i=0; i<5; i++)Serial1.read(); // byte 7, 8, 9 are ignored
    }    
  }
  return liDARdist; // in cm
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
  dist45R = readUltrasonic(ultraEcho45R, ultraEcho45R);
  dist20R = readUltrasonic(ultraEcho20R, ultraEcho20R);
  dist5R = readUltrasonic(ultraEcho5R, ultraEcho5R);
  dist5R = readUltrasonic(ultraEcho5L, ultraEcho5L);
  dist20R = readUltrasonic(ultraEcho20L, ultraEcho20L);
  dist45R = readUltrasonic(ultraEcho45L, ultraEcho45L);
}

float getReboundAngle() { // left is positive, right is negative
  // Bubble rebound algorithm using ultrasonic data to return the angle to 
  // get to the path with the fewest/furthest obstacles
  float angle_dist = (45 * (dist45L - dist45R)) + (20 * (dist20L - dist20R)) + (5 * (dist5L - dist5R));
  float sum = dist45L + dist20L + dist5L + dist5R + dist20R + dist45R;
  return angle_dist / sum;
}

void setMotorSpeed(int motorPin, double motorSpeed) {
  /* Control the motor speed using the Victor888's
  *  1300   - full speed backwards
  *  1500 - stopped
  *  1700 - full speed forwards
  */
  analogWrite(motorPin, motorSpeed);
}

void stopMotors() {
  setMotorSpeed(leftMotor, 1500);
  setMotorSpeed(rightMotor, 1500);
}

void updateSpeed(int distance) { // distance is in cm
  /* If nothing is detected in near zone,
     use the liDAR-detected distance to determine speed.
     The further the nearest detected obstacle, the faster
     the speed up to a max of ___. <-- check with motor controller
  */
  if (distance > 1000 && (speed + 5) < 1700) {
    speed+=5;
  } else if (distance < 500 && (speed - 5) > 1300) {
    speed-=5;
  }
}

void updateCompassModule() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false); //parameter indicates that the Arduino will send a restart. The connection is kept active
  Wire.requestFrom(MPU,12,true); //request a total of 12 registers 
  
  GyX=Wire.read()<<8|Wire.read(); 
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
  int xAng = map(AcX,minVal,maxVal,-90,90); int yAng = map(AcY,minVal,maxVal,-90,90); int zAng = map(AcZ,minVal,maxVal,-90,90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI); y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI); z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  
  Serial.print("AngleX= "); Serial.println(x);

  Serial.print("AngleY= "); Serial.println(y);

  Serial.print("AngleZ= "); Serial.println(z); Serial.println("-------"); delay(100); //don't know how to grab current heading, can find angle with gyro though.
  
  Serial.print("Gyroscope: "); //prints data from gyro, into readable data
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
}

float getHeading() {
  updateCompassModule();
  // rando mathy stuff LUUUUUUUCCCCCCCCCCCC
}

void loop() {
  updateUltrasonic();
  if (isNearZone) { // an object is detected within 75cm
    float reboundAngle = getReboundAngle();
    float startingHeading = getHeading()
    float currentHeading = startingHeading;
    
      if (reboundAngle > 0) { // must turn left
        int leftMotorSpeed = 1600;
        int rightMotorSpeed = 1300;
      } else { //turn right
        int leftMotorSpeed = 1300;
        int rightMotorSpeed = 1600;
      }
    
      while (abs(currentHeading - startingHeading) > reboundAngle) {
          setMotorSpeed(leftMotor, leftMotorSpeed);
          setMotorSpeed(rightMotor, rightMotorSpeed);
          delay(50);
          stopMotors();
          currentHeading = getHeading();
      }
  } else { // There is nothing within 75cm
    // Increase or decrease speed based on the closest object detected by the liDAR
    updateSpeed(readLiDAR());
    setMotorSpeed(leftMotor, speed);
    setMotorSpeed(rightMotor, speed);
  }
  delay(100);
}
