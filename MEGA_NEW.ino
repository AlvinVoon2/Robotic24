#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include <Adafruit_GPS.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MOTOR_IN1 5  //left rear
#define MOTOR_IN2 4  //left rear
#define MOTOR_IN3 11 //left front
#define MOTOR_IN4 10 //left front
#define MOTOR_IN5 7  //right front
#define MOTOR_IN6 6  //right front
#define MOTOR_IN7 9  //right rear
#define MOTOR_IN8 8  //right rear

//SoftwareSerial Bluetooth(7, 8); //bluetooth module now attached to Serial1 on Mega
//SoftwareSerial mySerial(4, 10);  //uno now attached to Serial3 on Mega

int flagMoveStatus = 0; 
int flagPlantStatus = 0; 

unsigned long startMillis;
unsigned long currentMillis;
bool timerRunning = false;
bool functionExecuted = false;

float currentLat = 0, currentLong = 0;
float targetLat = 15, targetLong = 20;
float targetHeading = 0;
float targetHeading2 = 0;

float tolerance = 5.0;
float tolerance_coord = 0.0005;
unsigned long previousMillis = 0;
float heading = 0.0;

MPU6050 mpu;

#define INTERRUPT_PIN 2
#define LED_PIN 13  // Built-in LED pin on Arduino Uno
bool blinkState = false;
volatile bool mpuInterrupt = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

unsigned long startTime;
unsigned long runDuration = 3000;  // Duration to run the function in milliseconds (e.g., 5000 ms = 5 seconds)

void dmpDataReady() {
  mpuInterrupt = true;
}

float distanceLeft = 0;

static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
//SoftwareSerial gpsSerial(12, 13);  // RX, TX for GPS
//Adafruit_GPS GPS(&gpsSerial);

bool gpsDataValid = false;  // Track GPS data validity

enum State { TURNING,
             MOVING };
State currentState = TURNING;

bool motorsRun = false;        // Track if motors have already run
unsigned long motorStartTime;  // Store start time of motor run

void setup() {
  //Serial3.begin(9600);
  //Bluetooth.begin(9600);

  Serial.begin(9600);    // Initialize the Serial monitor for debugging
  Serial3.begin(9600);   // Initialize Serial1 for sending data

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#endif

  startTime = millis();  // Record the start time

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_IN5, OUTPUT);
  pinMode(MOTOR_IN6, OUTPUT);
  pinMode(MOTOR_IN7, OUTPUT);
  pinMode(MOTOR_IN8, OUTPUT);

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  digitalWrite(MOTOR_IN5, LOW);
  digitalWrite(MOTOR_IN6, LOW);
  digitalWrite(MOTOR_IN7, LOW);
  digitalWrite(MOTOR_IN8, LOW);

  Serial.begin(9600);
  //gpsSerial.begin(GPSBaud);  // GPS

  // GPS setup
  // GPS.begin(GPSBaud);
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // GPS.sendCommand(PGCMD_NOANTENNA);
  // delay(1000);

  while (!Serial)
    ;

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(LED_PIN, OUTPUT);

  Serial.println("start");
  Serial.println(getDirection(currentLat, currentLong, targetLat, targetLong));
  CalculateDistance();

  // / processGPS();
  // readCompass();
  //  Calculate();
  // bearing(currentLat, currentLong, targetLat, targetLong);
}

float initialDistance = 0.0;
int DoneTurning = 0;
int Reach = 0;
int stop = 0;
int counter;
int gap;

void loop() {
  Wire.requestFrom(0x10, 1);

  while (Wire.available()) {
    //Serial.println("there is data");
    gap = Wire.read();
    //Serial.println(gap);
  }
  
  Serial.print("Got data from UNO: ");
  Serial.println(Serial3.available());

  if (Serial3.available() > 0) {
    char receivedChar = Serial3.read(); // read one byte from serial buffer and save to receivedData
    if (receivedChar == '1') { // receive from Uno
       // commence movegarithm
       counter++;
       Serial.println(counter);
       flagMoveStatus=1;
       delay(2000);
       Serial.print("Received from UNO to move: ");
       Serial.println(flagMoveStatus);
       runForDuration(Movegarithm, runDuration, berhenti);
       flagMoveStatus=0;
       flagPlantStatus=1;
       //delay(10000);
    }
    else if (receivedChar == '0') {
       // do nothing
    }
  }

  // Check if we need to send a plant command to Uno
if (counter <= gap) {
  if (flagPlantStatus == 1 && !timerRunning) {  // Only send plant command if timer is not running
    delay(2500);
    Serial3.println(flagPlantStatus);  // Send the message through Serial3 with a newline character
    Serial.println("Sending to UNO: Plant!");
    flagPlantStatus = 0;
  }
}

 // MovegarithmTest ();
 // readCompass();

  //delay(500);

  // runForDuration(forward, 12000, berhenti);
  // readCompass();
  // TurnToTarget();
}

void Calculate() {
  float dlon = radians(targetLong - currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0) {
    a2 += TWO_PI;
  }
  targetHeading = degrees(a2);
  Serial.println("Method 1: " + String(targetHeading));

  float differenceLong = (targetLong - currentLong);
  float x = cos(targetLong) * sin(differenceLong);

  float y = cos(currentLat) * sin(targetLat) - sin(currentLat) * cos(targetLat) * cos(differenceLong);

  targetHeading2 = degrees(atan2(x, y));
  //Serial.println("Method 2: " + String(targetHeading2));
}

void bearing(float lat, float lon, float lat2, float lon2) {
  float teta1 = radians(lat);
  float teta2 = radians(lat2);
  float delta1 = radians(lat2 - lat);
  float delta2 = radians(lon2 - lon);

  float y = sin(delta2) * cos(teta2);
  float x = cos(teta1) * sin(teta2) - sin(teta1) * cos(teta2) * cos(delta2);
  float brng = atan2(y, x);
  brng = degrees(brng);
  brng = (((int)brng + 360) % 360);

  Serial.print("Method 3: ");
  Serial.println(brng);
}

int getDirection(float latitude1, float longitude1, float latitude2, float longitude2) {
  float lat1 = toRadians(latitude1);
  float lat2 = toRadians(latitude2);
  float lng1 = toRadians(longitude1);
  float lng2 = toRadians(longitude2);
  float Y = sin(lng2 - lng1) * cos(lat2);
  float X = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lng2 - lng1);
  float deg = toDegrees(atan2(Y, X));
  if (deg < 0) {
    deg = 360 + deg;
  }
  float angle = deg;
  int a = (int)(abs(angle) + (1 / 7200));
  return a;
}

float toRadians(float angle) {
  return (PI / 180) * angle;
}

float toDegrees(float rad) {
  return (rad * 180) / PI;
}

void CalculateDistance() {
  float delLat = abs(currentLat - targetLat) * 111194.9;
  float delLong = 111194.9 * abs(currentLong - targetLong) * cos(radians((currentLat + targetLat) / 2));
  float distance = sqrt(pow(delLat, 2) + pow(delLong, 2));
  initialDistance = distance;
  distanceLeft = distance;
}

void readCompass() {
  if (!dmpReady) return;

  if (mpuInterrupt) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      heading = ypr[0] * 180 / M_PI;
      //Serial.println(heading);

      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
    mpuInterrupt = false;
  }
}

void runForDuration(void (*func)(), unsigned long duration, void (*timeoutFunc)()) {
  startMillis = millis();  // Reset start time
  timerRunning = true;     // Start the timer
  functionExecuted = false; // Reset function execution flag

  while (millis() - startMillis < duration) {
    func(); // Ensure the function is executed if the timer is running
  }
  timeoutFunc(); // Call the timeout function once the duration has passed
  timerRunning = false; // Reset the timer
  functionExecuted = true; // Mark function as executed
}

void TurnToTarget() {
  float upperRange = targetHeading + tolerance;
  float lowerRange = targetHeading - tolerance;

  int error_heading = heading - targetHeading; //khots
  int speed = map(abs(error_heading), 0, 360, 60, 0); //khots

  //Serial.println(heading);

  if (heading >= lowerRange && heading <= upperRange) {
    DoneTurning = 1;
    berhenti();
    Serial.println("Done Turning");
  }
  else if (error_heading > 0) {
    right(135 - speed);
  }
  else {
    DoneTurning = 0;
    left(135 - speed);
    //Serial.println("Still turning");
    // Serial.println(heading);
  }
}

void processGPS() {
  if (gps.location.isValid()) {
    gpsDataValid = true;
    currentLat = gps.location.lat();
    currentLong = gps.location.lng();
    Serial.println(String(currentLat) + String(currentLong));
  } else {
    gpsDataValid = false;
  }
}

void MoveToTarget() {
  float upperRangeLat = targetLat + tolerance_coord;
  float lowerRangeLat = targetLat - tolerance_coord;
  float upperRangeLong = targetLong + tolerance_coord;
  float lowerRangeLong = targetLong + tolerance_coord; //KhoTS mistake?

  if (currentLat >= lowerRangeLat && currentLat <= upperRangeLat && currentLong >= lowerRangeLong && currentLong <= upperRangeLong) {
    Reach = 1;
    berhenti();
  } else {
    Reach = 0;
    readCompass();
    float correction = map((targetHeading2 - heading), 0, 360, 0, 60); 
    Serial.println(correction);
    //forward(correction);
  }
}

int motorLPWM;
int motorRPWM;
void SendData() {
  //Bluetooth.println(String(heading) + "," + String(targetHeading) + "," + String(currentLat) + "," + String(currentLong) + "," + String(motorLPWM) + "," + String(motorRPWM) + "," + String(distanceLeft));
  Serial.println(String(heading) + "," + String(targetHeading) + "," + String(currentLat) + "," + String(currentLong) + "," + String(motorLPWM) + "," + String(motorRPWM) + "," + String(distanceLeft));
  delay(1000);
}

void forward() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_IN2, 150); //kts change
  analogWrite(MOTOR_IN3, 150); //kts change
  //motorLPWM = 255 + correction;
  //motorRPWM = 255 - correction;
}


void left(int speed) {
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_IN1, speed);
  analogWrite(MOTOR_IN3, speed);

  motorLPWM = speed;
  motorRPWM = speed;
  //digitalWrite(IN1, LOW);
  //digitalWrite(IN2, HIGH);
  //digitalWrite(IN3, HIGH);
  //digitalWrite(IN4, LOW);
  //analogWrite(ENA, speed); // Slightly lower speed for stability
  //analogWrite(ENB, speed);
}

void right(int speed) {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  analogWrite(MOTOR_IN2, speed);
  analogWrite(MOTOR_IN4, speed);

  motorLPWM = speed;
  motorRPWM = speed;
  //  digitalWrite(IN1, HIGH);
  //digitalWrite(IN2, LOW);
  // digitalWrite(IN3, LOW);
  // digitalWrite(IN4, HIGH);
  //analogWrite(ENA, 200); // Slightly lower speed for stability
  //analogWrite(ENB, 200);
  Serial.println("right");
}

void berhenti() {
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, LOW);
  //digitalWrite(IN3, LOW);
  //digitalWrite(IN4, LOW);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  digitalWrite(MOTOR_IN5, LOW);
  digitalWrite(MOTOR_IN6, LOW);
  digitalWrite(MOTOR_IN7, LOW);
  digitalWrite(MOTOR_IN8, LOW);
}

int kfactor = 100; // change value from 0-100
int dampSpeed = 100; //change in pwm relative to base speed
int baseSpeed = 255; //base speed pwm
int angleToTargetRef = 0;
int angleSlow =45;
float dampSpeedC =0;


void MovegarithmTest () {
  int speed =0;
  int angleToTarget=heading-targetHeading2;

  Serial.print("heading=");
  Serial.print(heading);

  Serial.print(", angleToTarget=");
  Serial.print(angleToTarget);
  
  if (angleToTarget<-180) {  
    angleToTargetRef=angleToTarget+360;
  }
  else if (angleToTarget>180) { 
    angleToTargetRef=angleToTarget-360;
  }
  else {
    angleToTargetRef=angleToTarget;
  }

  Serial.print(", angleToTargetRef=");
  Serial.print(angleToTargetRef);
  
  if (angleToTargetRef>=-180 && angleToTargetRef<=-tolerance) {  // turn right
    if (abs(angleToTargetRef)<angleSlow){
      dampSpeedC = map(abs(angleToTargetRef), 0, angleSlow, dampSpeed, 0); 
    }
    else{
      dampSpeedC = 0;
    }
    speed=baseSpeed-dampSpeedC;
    TRSharp(speed);

    Serial.print(", speed=");
    Serial.print(speed);
    Serial.print(", turn right");
  }
  else if (angleToTargetRef>=tolerance) { //turn left
    if (abs(angleToTargetRef)<angleSlow){
      dampSpeedC = map(abs(angleToTargetRef), 0, angleSlow, dampSpeed, 0); 
    }
    else{
      dampSpeedC = 0;
    }
    speed=baseSpeed-dampSpeedC;
    TLSharp(speed);

    Serial.print(", speed=");
    Serial.print(speed);
    Serial.print(", turn left");
  }
  
  else { // move forward/ do nothing
    float correction = map(abs(angleToTargetRef), 0, tolerance, 0, 60); 
    if (angleToTargetRef<=0) {
      correction=-correction;
    }
    correction=-correction;
    //Serial.println(correction);
    forward2(correction);

    Serial.print(", correction=");
    Serial.print(correction);
    Serial.print(", forward");
  }


Serial.println(" ");

}

void Movegarithm () {
  float upperRangeLat = targetLat + tolerance_coord;
  float lowerRangeLat = targetLat - tolerance_coord;
  float upperRangeLong = targetLong + tolerance_coord;
  float lowerRangeLong = targetLong - tolerance_coord;
  int speed =0;

  if (currentLat >= lowerRangeLat && currentLat <= upperRangeLat && currentLong >= lowerRangeLong && currentLong <= upperRangeLong) {
    Reach = 1;
    berhenti();
  } else {
    Reach = 0;
    readCompass();

    int angleToTarget=heading-targetHeading2;
    if (angleToTarget<-180) {  
      angleToTargetRef=angleToTarget+360;
    }
    else if (angleToTarget>180) {
      angleToTargetRef=angleToTarget-360;
    }
    else {
      angleToTargetRef=angleToTarget;
    }
    
    if (angleToTargetRef>=-180 && angleToTargetRef<=-tolerance) {  // turn right
      if (abs(angleToTargetRef)<angleSlow){
        dampSpeedC = map(abs(angleToTargetRef), 0, angleSlow, dampSpeed, 0); 
      }
      else{
        dampSpeedC = 0;
      }
      speed=baseSpeed-dampSpeedC;
      TRSharp(speed);
    }
    else if (angleToTargetRef>=tolerance) { //turn left
      if (abs(angleToTargetRef)<angleSlow){
        dampSpeedC = map(abs(angleToTargetRef), 0, angleSlow, dampSpeed, 0); 
      }
      else{
        dampSpeedC = 0;
      }
      speed=baseSpeed-dampSpeedC;
      TLSharp(speed);
    }
    else { // move forward/ do nothing
      float correction = map(abs(angleToTargetRef), 0, tolerance, 0, 250); 
      if (angleToTargetRef<=0) {
        correction=-correction;
      }
      
      Serial.println(correction);
      forward2(correction);
    }
    
  }
}

void TLSlow () {

}

void TRSlow () {

}

void TLSharp (int speed) {
  analogWrite(MOTOR_IN2, speed); 
  analogWrite(MOTOR_IN4, speed);
  analogWrite(MOTOR_IN6, speed);
  analogWrite(MOTOR_IN8, speed);
  
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN5, LOW);
  digitalWrite(MOTOR_IN7, LOW);
}

void TRSharp (int speed) {
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  digitalWrite(MOTOR_IN6, LOW);
  digitalWrite(MOTOR_IN8, LOW);

  analogWrite(MOTOR_IN1, speed);
  analogWrite(MOTOR_IN3, speed);
  analogWrite(MOTOR_IN5, speed);
  analogWrite(MOTOR_IN7, speed);

}


void forward2(int correction) {
  int speed1=baseSpeed + correction;
  int speed2=baseSpeed - correction;
  if (speed1 > 255){
    speed1=255;
  }
  if (speed2 > 255){
    speed2=255;
  }
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  digitalWrite(MOTOR_IN5, LOW);
  digitalWrite(MOTOR_IN7, LOW);
  analogWrite(MOTOR_IN1, speed1);  // need to check this
  analogWrite(MOTOR_IN3, speed2);  // need to check this
  analogWrite(MOTOR_IN6, speed1);  // need to check this
  analogWrite(MOTOR_IN8, speed2);  // need to check this
}