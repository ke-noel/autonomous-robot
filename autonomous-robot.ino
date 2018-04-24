/* Program for self-driving robot
 * Senses objects using liDAR and ultrasonic and avoids them,
 * prioritizing ultrasonic
 */





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

// Motor pins
#define leftMotor 12
#define rightMotor 13

float dist45R;
float dist20R;
float dist5R;
float dist5L;
float dist20L;
float dist45L;

bool isNearZone; // is an object in the near zone?

volatile int liDARval;

float speed = 0;

void setup() {
  Serial1.begin(115200); // HW serial for LiDAR
  Serial.begin(3600); // USB to computer serial
  delay(100);

  // Set to standard output mode
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
}

double readLiDAR() {
  // SHOULD BE IN ITS OWN THREAD
  /* Input format for TFMini LiDAR
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
      t2 <<= 8;
      t2 += t1;
      liDARval = t2;
      t1 = Serial1.read(); // byte 5 (Strength_L)
      t2 = Serial1.read(); // byte 6 (Strength_H)
      t2 <<= 8;
      t2 += 1;
      for(int i=0; i<3; i++)Serial1.read(); // byte 7, 8, 9 are ignored
    }    
  }
  return liDARval;
}

float readUltrasonic(int echoPin, int trigPin) {
  digitalWrite(echoPin + 1, LOW);
  delayMicroseconds(2);
  digitalWrite(echoPin + 1, HIGH);
  delayMicroseconds(10);
  digitalWrite(echoPin + 1, LOW);
  float duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.0345 / 2; // in cm
  if (distance < 3 || distance > 275) { // ultrasonic rated accurate from 3 to 300 cm
    distance = 0;
  } else if (distance < 100) {
    isNearZone = true; // object is detected in the near zone
  }
  return distance;
}

void updateUltrasonic() {
  dist45R = readUltrasonic(ultraEcho45R, ultraEcho45R);
  dist20R = readUltrasonic(ultraEcho20R, ultraEcho20R);
  dist5R = readUltrasonic(ultraEcho5R, ultraEcho5R);
  dist5R = readUltrasonic(ultraEcho5L, ultraEcho5L);
  dist20R = readUltrasonic(ultraEcho20L, ultraEcho20L);
  dist45R = readUltrasonic(ultraEcho45L, ultraEcho45L);
}

float getReboundAngle() { // left is positive, right is negative
  // Bubble rebound algorithm
  float angle_dist = (45 * (dist45L - dist45R)) + (20 * (dist20L - dist20R)) + (5 * (dist5L - dist5R));
  float sum = dist45L + dist20L + dist5L + dist5R + dist20R + dist45R;
  return angle_dist / sum;
}

void setMotorSpeed(int motorPin, double motorSpeed) {
  analogWrite(motorPin, motorSpeed);
}

void stopMotors() {
  analogWrite(leftMotor, 0);
  analogWrite(rightMotor, 0);
}

void updateSpeed(int distance) { // test for units!!!!!!!
  /* If nothing is detected in near zone,
     use the liDAR-detected distance to determine speed.
     The further the nearest detected obstacle, the faster
     the speed up to a max of ___. <-- check with motor controller
  */
  if (distance > 20) { // assuming measured in metres
    speed += 0.2;
  } else {
    speed -= 0.2;
  }
}

void loop() {
  updateUltrasonic();
  if (isNearZone) { // an object is detected by the ultrasonic sensors
    float reboundAngle = getReboundAngle();
    float startingHeading = 0.0; // CATHERINE STUFF GOES HERE
    float currentHeading = 0.0;
    
      if(reboundAngle > 0) { // must turn left
        stopMotors();
        while (abs(currentHeading - startingHeading) < reboundAngle) {
          setMotorSpeed(leftMotor, 1);
          setMotorSpeed(rightMotor, 0);
          delay(50);
        }
      } else { // must turn right
        while (abs(currentHeading - startingHeading) < reboundAngle) {
          setMotorSpeed(rightMotor, 1);
          setMotorSpeed(leftMotor, 0);
          delay(50);
        }
      }
  } else {
    updateSpeed(readLiDAR());
    setMotorSpeed(leftMotor, speed);
    setMotorSpeed(rightMotor, speed);
  }
  delay(100);
}
