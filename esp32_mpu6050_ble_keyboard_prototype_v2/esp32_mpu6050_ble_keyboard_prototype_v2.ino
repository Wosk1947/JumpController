#include <Wire.h>
#include <MPU6050_light.h>
#include <BleKeyboard.h>
#include <math.h>

MPU6050 mpu(Wire);
BleKeyboard bleKeyboard("ESP32 Keyboard Win", "Espressif", 100);

void setup() {
  Serial.begin(115200);
  pinMode(A5, INPUT_PULLUP);
  mpu.setAddress(0x68);
  Wire.begin();
  byte status = mpu.begin(1,3,2);
  while(status!=0){Serial.println("No signal from MPUs");}
  //mpu.calcOffsets();

  bleKeyboard.begin();
}

enum BodyState
{
    NONE,
    JUMP_START,
    JUMP_LAND,
    SECOND_JUMP_STANDBY,
    JUMP_END,
    DUCK_START,
    DUCK_END,
    TEST
};

const int bufferSize = 50; 
const int rawBufferSize = 150; 
double rawBuffer[rawBufferSize] = {0};
double smthBuffer[bufferSize] = {0};

void shiftBuffer(double buffer[], int size, int n) {
  for (int i = 0; i < size - n; i++) {
    buffer[i] = buffer[i + n];
  }
  for (int i = max(size - n, 0); i < size; i++) {
    buffer[i] = 0;
  }
}

void addToBuffer(double buffer[], int size, double value) {
  shiftBuffer(buffer, size, 1);
  buffer[size - 1] = value;
}

double maxInBuffer(double buffer[], int size) {
  double maxVal = -999999999.0;
  for (int i = 0; i < size; i++) {
    maxVal = max(maxVal, buffer[i]);
  }
  return maxVal;
}

long jumpStartTime = 0;
long duckInitTime = 0;
long jumpLandTime = 0;
long jumpTerminationTime = 1200;
long jumpLandOffsetTime = 200;
long jumpStartOffsetTime = 350;

double alpha = 0.98;
double prevSmthVal = 0;
double currSmthVal = 0;

double prevSmthVal2 = 0;
double currSmthVal2 = 0;

double prevAz = 999;
double lastDip = 0;
double lastDipTime = 0;

BodyState currentState = NONE;

double prevAngle = 0;
double currentAngle = 0;
double currentEMAAngle = 0;
double prevEMAAngle = 0;
double currentHPAngle = 0;
double prevHPAngle = 0;

double angleT = 10;

double jumpStartMaxValRaw = 0;
double jumpStartMaxValSm = 0;


bool debug = false;
int logType = 1;

double jumpValThres = 2.1; //2.1

void loop() {
  //Get values and preprocess
  mpu.update();
  double gX = mpu.getAngleX();
  double aZ = mpu.getAccZ();
  double a = sqrt(mpu.getAccZ() * mpu.getAccZ() + mpu.getAccY() * mpu.getAccY() + mpu.getAccX() * mpu.getAccX());
  aZ = a;
  addToBuffer(rawBuffer, rawBufferSize, aZ);
  currSmthVal = prevSmthVal * alpha + (1-alpha) * aZ;
  prevSmthVal = currSmthVal;
  addToBuffer(smthBuffer, bufferSize, currSmthVal);

  currSmthVal2 = prevSmthVal2 * alpha + (1-alpha) * gX;
  prevSmthVal2 = currSmthVal2;

  if (aZ < prevAz) {
    lastDip = aZ;
    lastDipTime = millis();
  }
  prevAz = aZ;


  ///Angles + High pass test:
  double al = 0.95;
  double k = 4;
  currentAngle = mpu.getAngleY();
  if (isnan(currentAngle)) {
    currentAngle = 0;
  }
  currentEMAAngle = al * prevEMAAngle + (1 - al) * currentAngle;
  prevEMAAngle = currentEMAAngle;
  currentHPAngle = currentEMAAngle + k * (currentAngle - currentEMAAngle);
  //Serial.println(currentHPAngle);
  ////////////////////////

  double jumpLog = 0;
  double logDuck = 0;

  //State Machine
  switch (currentState) {
    case NONE:
      if (smthBuffer[bufferSize-1] >= jumpValThres) {
        //Serial.println("JUMP");
        if(bleKeyboard.isConnected()) {
          if (!debug) {
            bleKeyboard.press(' ');
          }
        }
        jumpLog = 5;
        currentState = JUMP_START;
        jumpStartTime = millis();

        jumpStartMaxValRaw = maxInBuffer(rawBuffer, rawBufferSize);
        jumpStartMaxValSm = 0;

        break;
      }
      if (currSmthVal2 < -45) {
        //Serial.println("ENTER DUCK");
        if(bleKeyboard.isConnected()) {
          if (!debug) {
            bleKeyboard.press(KEY_LEFT_CTRL);
          }
        }
        currentState = DUCK_START;
        logDuck = 1;  
      }
      break;
    case JUMP_START:
      if (millis() > jumpStartTime + jumpTerminationTime) {
        currentState = JUMP_END;
      }
      if (millis() < jumpStartTime + 30) {
        if(bleKeyboard.isConnected()) {
          if (!debug) {
            bleKeyboard.press(' ');
          }
        }
      }
      if (millis() < jumpStartTime + jumpStartOffsetTime) {

        jumpStartMaxValSm = max(jumpStartMaxValSm, smthBuffer[bufferSize-1]);
        

        break;
      }
      if (aZ > 2) {
        if(bleKeyboard.isConnected()) {
          if (!debug) {
            bleKeyboard.release(' ');
          }
        }
        //currentState = JUMP_LAND;
        currentState = SECOND_JUMP_STANDBY;
        jumpLandTime = millis();
      }
      break;
    case SECOND_JUMP_STANDBY:
      if(bleKeyboard.isConnected()) {
          if (!debug) {
            bleKeyboard.release(' ');
          }
        }
      if (millis() > jumpLandTime + 250) {
        currentState = JUMP_END;
        break;
      }
      if (smthBuffer[bufferSize-1] >= jumpValThres) {
        if (maxInBuffer(rawBuffer, rawBufferSize) >= jumpStartMaxValRaw * 0.9) {
          currentState = JUMP_END;
          break;
        }

        if(bleKeyboard.isConnected()) {
          if (!debug) {
            bleKeyboard.press(' ');
          }
        }
        
        jumpLog = 5;
        currentState = JUMP_START;
        jumpStartTime = millis();
        jumpStartMaxValRaw = maxInBuffer(rawBuffer, rawBufferSize);
        //jumpStartMaxValSm = 0;
      }
      break;
    case JUMP_LAND:
      if (millis() > jumpStartTime + jumpTerminationTime) {
        currentState = JUMP_END;
      }
      if (millis() < jumpLandTime + jumpLandOffsetTime) {
        break;
      }
      currentState = JUMP_END;
      break;
    case JUMP_END:
      jumpLog = 5;
      prevSmthVal = 1.0;
      currentState = NONE;
      break;
    case DUCK_START:
      if (currSmthVal2 > -20) {
        //Serial.println("EXIT DUCK");
        if(bleKeyboard.isConnected()) {
          if (!debug) {
            bleKeyboard.release(KEY_LEFT_CTRL);
          }
        }
        currentState = DUCK_END;
      }
      break;
    case DUCK_END:
      logDuck = 1;
      prevSmthVal = 1;
      currentState = NONE;
      break;
  }

 // Serial.println(currentState);

  if (debug && logType == 1) {
    Serial.print(aZ);
    Serial.print("\t");
    Serial.print(smthBuffer[bufferSize-1]);
    Serial.print("\t");
    Serial.print(jumpLog);
    //Serial.print("\t");
    //Serial.print(currentHPAngle);
    Serial.println();
  }


  delay(1);
}