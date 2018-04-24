/* LiDAR pins
  black - GND (to Arduino GND)
  red --- 5V (to Arduino 5V)
  white - liDAR RX (to Arduino TX)
  green - liDAR TX (to Arduino RX)  
*/

// Ultrasonic pins
  #define ultraEchoPinFL 0
  #define ultraTrigPinFL 1

  #define ultraEchoPinFR 2
  #define ultraTrigPinFR 3

  #define ultraEchoPinL 4
  #define ultraTrigPinL 5

  #define ultraEchoPinR 6
  #define ultraTrigPinR 7

  #define ultraEchoPinBL 8
  #define ultraTrigPinBL 9

  #define ultraEchoPinBR 10
  #define ultraTrigPinBR 11

enum ultrasonic {LL,CL,CC,CR,RR}; // L (left), C (centre), R (right)

double ultraLL;
double ultraCL;
double ultraCC;
double ultraCR;
double ultraRR;

volatile int liDARval;

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
  
  pinMode(ultraEchoPinFL, INPUT);
  pinMode(ultraTrigPinFL, OUTPUT);
  
  pinMode(ultraEchoPinFR, INPUT);
  pinMode(ultraTrigPinFR, OUTPUT);
  
  pinMode(ultraEchoPinL, INPUT);
  pinMode(ultraTrigPinL, OUTPUT);
  
  pinMode(ultraEchoPinR, INPUT);
  pinMode(ultraTrigPinR, OUTPUT);
  
  pinMode(ultraEchoPinBL, INPUT);
  pinMode(ultraTrigPinBL, OUTPUT);
  
  pinMode(ultraEchoPinBR, INPUT);
  pinMode(ultraTrigPinBR, OUTPUT);
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

double getUltrasonicDistance (double duration) {
  /* At 21 C, v sound = 340 m/s = 0.034 cm/microsecond
   * So, distance = (0.034cm/microsecond) * time
   */
  return (duration / 2) / 29.1; // check value
}

void readUltrasonic(int echoPin) {
  long duration, distance;
  digitalWrite(echoPin + 1, LOW);
  delayMicroseconds(2);
  digitalWrite(echoPin + 1, HIGH);
  delayMicroseconds(10);
  digitalWrite(echoPin + 1, LOW);
  duration = pulseIn(echoPin, HIGH); // in microseconds
  
  return getUltrasonicDistance(duration);  
}

void setMotorSpeed(int motorPin, double motorSpeed) {
  analogWrite(motorPin, motorSpeed);
}

void stopMotors() {
  for (int x = 0; x < 3; x++) {
    setMotorSpeed(x, 0);
  }
}

void loop() {
  // check sensors
    // check ultrasonic
      // if anything detected by ultrasonic, prioritize ultrasonic
        // in front? close? stop.
        // bubble rebound algorithm
      // otherwise prioritize liDAR
        // 
  delay(50);
}
