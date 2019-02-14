//# 11/09/2018
//# lEGACY cODE - tOMAS
//# FSM - aBHI -- abyjain@media.mit.edu

#include <Wire.h>
#include <MPU6050.h>
#include <RFduinoBLE.h>

#define WAKE 0
#define NREM 1
#define REM 2

MPU6050 mpu;
int eye1Pin = 2;
int eye2Pin = 3;

double alpha = 0.85;
int eye1;
int eye2;
int pitch;
int roll;
int state = 0; //  0 - Wake , 1 - NREM , 2 - REM

int bodyMinutes = 0 ;
int eyeMinutes = 0 ;
int nullMinutes = 0 ;

int pitchBaseline = 0 ;
int rollBaseline = 0 ;
int eyeBaseline = 0 ;

int pitchThreshold = 1000 ;
int rollThreshold = 1000;
int eyeThreshold = 5 ;

int t_minutes = 0 ;
long t_millis = 0 ;
long t_send = 0 ;

void readData() {
  eye1 = analogRead(eye1Pin);
  eye1 = analogRead(eye1Pin);
  eye2 = analogRead(eye2Pin);
  eye2 = analogRead(eye2Pin);
  Vector normAccel = mpu.readNormalizeAccel();
  pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(eye1Pin, INPUT);
  pinMode(eye2Pin, INPUT);
  RFduinoBLE.advertisementData = "data";
//  RFduinoBLE.advertisementInterval = 1000;

  // Initialize BLE stack
  RFduinoBLE.begin();
  delay(1000);
  readData();
  calibrate();
}

void RFduinoBLE_onReceive(char *data, int len) {
  Serial.println(data[0]);
}


void loop() {
  readData();
  checkMovements();
  updateState();
  sendData();
  delay(10);
}

void calibrate(){
  eyeThreshold = eye1 ;
  pitchThreshold = pitch ;
  rollThreshold = roll ;
  t_millis = millis() ;
}

void sendData(){
  int numVals = 5;
  int vals[numVals];
  vals[0] = eye1;
  vals[1] = eye2;
  vals[2] = pitch;
  vals[3] = roll ;
  vals[4] = state ;
  char buf[numVals * 4]; // Arduino int size is 4 bytes
  for (int _i=0; _i<numVals; _i++)
      memcpy(&buf[_i*sizeof(int)], &vals[_i], sizeof(int));
  while (!RFduinoBLE.send((const char*)buf, numVals * 4));   // send data
}

void checkMovements(){
  t_minutes = (millis() - t_millis)/60000 ;
  if(t_millis > 0){
    int pitchDiff = pitch - pitchBaseline ;
    int rollDiff = roll - rollBaseline ;
    int eyeDiff = eye1 - eyeBaseline ;
    if(abs(pitchDiff) > pitchThreshold || abs(rollDiff) > rollThreshold){
      bodyMinutes = bodyMinutes + 1 ;
      eyeMinutes = 0 ;
      nullMinutes = 0 ;
    }
    else if(abs(eyeDiff) > eyeThreshold){
      bodyMinutes = 0 ;
      eyeMinutes = eyeMinutes + 1 ;
      nullMinutes = 0 ;
    }       
    else{
      bodyMinutes = 0 ;
      eyeMinutes = 0 ;
      nullMinutes = nullMinutes + 1 ;
    }
    t_millis = millis();
   }
}

void updateState(){
   switch(state){
    case WAKE: if(nullMinutes >= 5) state = NREM ;
               break;
    case NREM: if(eyeMinutes >= 3) state = REM ;
               if(bodyMinutes >= 3) state = WAKE ;
               break;
    case REM: if(bodyMinutes >= 3) state = WAKE ;
              if(nullMinutes >= 5) state = NREM ;
               break;
   }
}
