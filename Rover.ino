#include "Arduino.h"
#include <SoftwareSerial.h>

// GPS Module
void clearBufferArray(void);

// Motion Module
void setSpeed(int,int, int);

// Navigation Module

// Obstacle Module
bool avoidObstacles(void);
bool obstacleAhead(void);

// Ultrasonic Module
class Ultrasonic {
  public:
  Ultrasonic(int pin);
  void ping(void);
  double getDistance(void);
  private:
  int this_pin;
  long duration;
};

// Globals
int E1 = 6; // Left speed control
int E2 = 5; // Right speed control
unsigned char GPSBuffer[64]; // To receive GPS data
int GPSBufferCount = 0; // Index into GPSBuffer
int M1 = 8; // Left direction control
int M2 = 7; // Right direction control
SoftwareSerial SoftSerial(2,3); // For GPS
Ultrasonic ultrasonic(11);

/*** GPS Module ***/
// Nulls GPS buffer
// Thanks MakerFabs
void clearBufferArray(void) {
  for (int i=0; i<GPSBufferCount;i++) {
    GPSBuffer[i]=NULL;
  }
}

/*** Motion Module ***/
// Sets the direction and speed of an individual motor
// Param1: 0-left motor, 1-right motor, Param2: LOW-forward, HIGH-backward, Param3: 0-255 speed
void setSpeed(int mtr, int dir, int spd) {
  if (spd<0) spd=0;
  if (spd>255) spd=255;
  if (mtr==0) {
    digitalWrite(M1,dir);
    analogWrite(E1,spd);
  } else {
    digitalWrite(M2,dir);
    analogWrite(E2,spd);
  }
}

/*** Obstacle Module ***/
// 
bool avoidObstacles(void) {
  int d = 700; // for 45 degree turn at 1 motor at full
  bool motor = false; // false = M1, true = M2
  bool obstacleDetected = false;
  while(obstacleAhead()) {
    obstacleDetected = true;
    setSpeed(0,0,0);
    setSpeed(1,0,0);
    setSpeed(motor,0,255);
    delay(d);
    setSpeed(motor,0,0);
    ultrasonic.ping();
    motor = !motor; // Turn other way next time
    d += 700; // 
  }
  if (obstacleDetected) {
    // Move past obstacle
    setSpeed(0,0,255);
    setSpeed(1,0,255);
    delay(1000);
    setSpeed(0,0,0);
    setSpeed(1,0,0);
  }
  return obstacleDetected;
}

bool obstacleAhead(void) {
  ultrasonic.ping();
  return (ultrasonic.getDistance() > 0.0 && ultrasonic.getDistance() < 10.0);
}

/*** Ultrasonic Module ***/
// Checks range
// Thanks Seeed
void Ultrasonic::ping(void) {
  pinMode(this_pin,OUTPUT);
  digitalWrite(this_pin,LOW);
  delayMicroseconds(2);
  digitalWrite(this_pin,HIGH);
  delayMicroseconds(2);
  digitalWrite(this_pin,LOW);
  pinMode(this_pin,INPUT);
  duration = pulseIn(this_pin,HIGH);
}

// Gets range in inches
// Thanks Seeed
double Ultrasonic::getDistance(void) {
  return duration/74.0/2.0;
}

// Constructor
// Thanks Seeed
Ultrasonic::Ultrasonic(int pin) {
  this_pin = pin;
}

/*** Main ***/
void loop() {
  /*if (avoidObstacles()) {
    return;
  }*/
  String s;
  if (SoftSerial.available()) {
    while(SoftSerial.available()) {
      s.concat((char)SoftSerial.read());
      //GPSBuffer[GPSBufferCount++]=r;
      //if(GPSBufferCount==64) break;
    }
    Serial.println(s);
    //Serial.write(GPSBuffer,GPSBufferCount);
    clearBufferArray();
    GPSBufferCount=0;
  }
  delay(1250);
}

void setup() {
  // Set up motor control pins
  for(int i=5;i<=8;i++) {
    pinMode(i,OUTPUT);
  }
  SoftSerial.begin(9600); // GPS link
  Serial.begin(9600);  // Output over Bluetooth
  
}
