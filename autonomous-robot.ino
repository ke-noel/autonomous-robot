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

// __________________________________________________________________________
// Note: pins are not final, some may not be on the appropriate type right now

/* LiDAR pins
  black - GND (to Arduino GND)
  red --- 5V (to Arduino 5V)
  white - liDAR RX (to Arduino TX)
  green - liDAR TX (to Arduino RX)  
*/

// Ultrasonic pin definitions
// arranged at 45, 20 and 5 degrees from the left and right
#define ultraTrig45R 0
#define ultraEcho45R 1
#define ultraTrig20R 2
#define ultraEcho20R 3
#define ultraTrig5R 4
#define ultraEcho5R 5
#define ultraTrig5L 6
#define ultraEcho5L 7
#define ultraTrig20L 8
#define ultraEcho20L 9
#define ultraTrig45L 10
#define ultraEcho45L 11

// Compass module pins

// Victor888 motor controller pins
#define leftMotor 12
#define rightMotor 13

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

float speed = 127; // initial motor speed; PWM 127 is stopped

int16_t GyX,GyY,GyZ // variables for gyro raw data


void setup() {
  Serial1.begin(115200); // HW serial for liDAR
  Serial.begin(3600); // USB to computer serial
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
  
  // Setup thread for reading the serial input from the LIDAR
  // needs include library not yet installed...
  
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
  
  Wire.begin();
  Wire.beginTransmission(MPU); // begins a trnasmission to the GY-521
  Wire.write(0x6B); 
  Wire.write(0); // set to zero    
  Wire.endTransmission(true);
  Serial.begin(9600);
  
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
    isNearZone = true; // object is detected in the near zone
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
  *  0   - full speed backwards
  *  127 - stopped
  *  255 - full speed forwards
  */
  analogWrite(motorPin, motorSpeed);
}

void stopMotors() {
  setMotorSpeed(leftMotor, 127);
  setMotorSpeed(rightMotor, 127);
}

void updateSpeed(int distance) { // distance is in cm
  /* If nothing is detected in near zone,
     use the liDAR-detected distance to determine speed.
     The further the nearest detected obstacle, the faster
     the speed up to a max of ___. <-- check with motor controller
  */
  if (distance > 1000 && (speed + 1) < 255) {
    speed++;
  } else if (distance < 500 && (speed - 1) > 127) {
    speed--;
  }
}

void loop() {
  updateUltrasonic();
  if (isNearZone) { // an object is detected within 75cm
    float reboundAngle = getReboundAngle();
    float startingHeading = 0.0; // PH: COMPASS MODULE STUFF GOES HERE
    float currentHeading = 0.0; // PH
  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false); //parameter indicates that the Arduino will send a restart. The connection is kept active
  Wire.requestFrom(MPU,12,true); //request a total of 12 registers 
  
  GyX=Wire.read()<<8|Wire.read(); 
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
  
  Serial.print("Gyroscope: "); //prints data from gyro, into readable data
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
  Serial.println(" ");
  delay(350);
    
      if(reboundAngle > 0) { // must turn left
        stopMotors();
        while (abs(currentHeading - startingHeading) < reboundAngle) {
          setMotorSpeed(leftMotor, 154); // forward
          setMotorSpeed(rightMotor, 100); // backward
          delay(50);
        }
      } else { // must turn right
        while (abs(currentHeading - startingHeading) < reboundAngle) {
          setMotorSpeed(rightMotor, 100); // backward
          setMotorSpeed(leftMotor, 154); // forward
          delay(50);
        }
      }
  } else { // There is nothing within 75cm
    // Increase or decrease speed based on the closest object detected by the liDAR
    updateSpeed(readLiDAR());
    setMotorSpeed(leftMotor, speed);
    setMotorSpeed(rightMotor, speed);
  }
  delay(100);
  
}
