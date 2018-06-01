// Victor888 motor controller pins
#define leftMotor 12
#define rightMotor 13

float speed = 127; // initial motor speed; PWM 127 is stopped

void setup() {
  Serial.begin(3600); // USB to computer serial
  delay(100);
  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);
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


void loop() {
  stopMotors();
  Serial.println("Stopped");
  delay(1000);
  setMotorSpeed(leftMotor, 200);
  setMotorSpeed(rightMotor, 200);
  Serial.println("Forwards");
  delay(500);
  stopMotors();
  Serial.println("Stopped");
  delay(1000);
  setMotorSpeed(leftMotor, 50);
  setMotorSpeed(rightMotor, 50);
  Serial.println("Backwards");
  delay(500);
  stopMotors();
  delay(10000);  
}
