#include <LIDARLite.h>

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

enum ultrasonic {FL,FR,L,R,BL,BR};

void setup() {
  Serial.begin(9600);
  
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

double getLIDAR() {
  return 0.0; // placeholder
}

double getUltrasonicDistance (double duration) {
  return (duration / 2) / 29.1; // check value
}

double getUltrasonic(int echoPin) { // use enum
  long duration, distance;
  digitalWrite(echoPin + 1, LOW);
  delayMicroseconds(2);
  digitalWrite(echoPin + 1, HIGH);
  delayMicroseconds(10);
  digitalWrite(echoPin + 1, LOW);
  duration = pulseIn(echoPin, HIGH);
  
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
  getLIDAR();
  for (int i = 0; i < 6; i++) {
    getUltrasonic(2*i);
  }
  delay(200);
}
