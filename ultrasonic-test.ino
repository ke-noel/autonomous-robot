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
    Serial.println("-----------STARTING UP-----------");
  
    delay(100);
    
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
    
    Serial.println("Program ready.");
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

void loop() {
  updateUltrasonic();
  if (isNearZone) { // an object is detected within 75cm
    Serial.println("in near zone!");
  } else {
    Serial.println("not in near zone");
    delay(400);
  }
}
