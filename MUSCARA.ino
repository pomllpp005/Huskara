// =====================
// USER PARAMETERS
// =====================
#include <POP32.h>
#define KP 0.011
#define KD 0.2
#define SPEED 65
#define RUN_TIME_MS 6000
#define MAX_OUT_TIME_MS 10
#define SENSOR_THRESHOLD_PERCENT 50

#define CALIBRATION_AVG_SAMPLES 8
#define CALIBRATION_COLLECTOR_SAMPLES 10

// =====================
// PIN MAPPING
// (STM32 pin names)
// =====================


#define PIN_MUX_S0 PA1
#define PIN_MUX_S1 PA2
#define PIN_MUX_S2 PA3
#define PIN_MUX_S3 PA4
#define PIN_MUX_SIG PA0

// =====================
// CONSTANTS
// =====================
#define NUM_SENSORS 16

// =====================
// GLOBAL VARIABLES
// =====================
int sensorValues[NUM_SENSORS];
unsigned int maxSensorValues[NUM_SENSORS]={3273,	3316,	3323,	3005,	3163,	2918,	3204,	2903	,2362,	2741,	3271,	3087,	3233,	3093,	2982,	1520	};
unsigned int minSensorValues[NUM_SENSORS]={235,	233,	231,	223,	219,	212,	225,	217,	216,	218,	229,	225,	238,	228,	221,	208};
int sensorThreshold[NUM_SENSORS];

bool isOnLine = false;
bool lastDetectedSide = false;
int previousError = 0;

bool LineBW = true;   //white = true ;  black = false ;
// =====================
// SETUP
// =====================
void setup() {
  

  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);

Serial.begin(9600);

  // รอปุ่มกดเริ่ม Calibration
  
  calibrateSensors();

  // สร้าง Threshold
  for (byte i = 0; i < NUM_SENSORS; i++) {
    sensorThreshold[i] = (maxSensorValues[i] - minSensorValues[i]) * (SENSOR_THRESHOLD_PERCENT / 100.0) + minSensorValues[i];
  }

  // รอปุ่มกดเริ่มวิ่ง
 waitSW_OK();
   
   AO();

}
void loop(){
 //Readcalibreate();
// readSerialRawSensors();
//  delay(100);
robotRun();
}

// =====================
// MAIN FUNCTION
// =====================
void robotRun() {
  unsigned long startTime = millis();
  unsigned long outStartTime = millis();

  bool running = true;

  while (running) {
    unsigned long currentTime = millis();

    unsigned int position = readLine();
    int error = position - (NUM_SENSORS - 1) * 1000 / 2;
    int deltaError = error - previousError;
    previousError = error;

    if (isOnLine && position && position != (NUM_SENSORS - 1) * 1000) {
      if (sensorValues[0]) lastDetectedSide = 0;
      if (sensorValues[NUM_SENSORS - 1]) lastDetectedSide = 1;

      long controlSignal = error * KP + deltaError * KD;
      controlSignal = constrain(controlSignal, -100, 100);

      int pwmL = SPEED + controlSignal;
      int pwmR = SPEED - controlSignal;

      pwmL = constrain(pwmL, -100, 100);
      pwmR = constrain(pwmR, -100, 100);

      motor(1, pwmL);
      motor(2, pwmR);

      outStartTime = currentTime;
    } else {
      if (currentTime - outStartTime >= MAX_OUT_TIME_MS) {
        running = false;
        break;
      }
      if (!lastDetectedSide) {
        motor(1, -90);
        motor(2, 100);
      } else {
        motor(1, 100);
        motor(2, -90);
      }
    }

    // if (currentTime - startTime >= RUN_TIME_MS) {
    //   running = false;
    //   break;
    // }
  }

 
}

// =====================
// SENSOR FUNCTIONS
// =====================
void selectMuxChannel(uint8_t channel) {
  digitalWrite(PIN_MUX_S0, (channel & 0x01));
  digitalWrite(PIN_MUX_S1, (channel & 0x02));
  digitalWrite(PIN_MUX_S2, (channel & 0x04));
  digitalWrite(PIN_MUX_S3, (channel & 0x08));
}

void readRawSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(i);
    delayMicroseconds(5);
    sensorValues[i] = analogRead(PIN_MUX_SIG);
  }
}
void readSerialRawSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(i);
    delayMicroseconds(5);
    sensorValues[i] = analogRead(PIN_MUX_SIG);
     Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}
void Readcalibreate(){
  readCalibratedSensors();
  for(uint8_t i = 0; i < NUM_SENSORS; i++){
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
 Serial.println();
}


void readCalibratedSensors() {
  readRawSensors();
  for (byte i = 0; i < NUM_SENSORS; i++) {
    unsigned int range = maxSensorValues[i] - sensorThreshold[i];
    if (sensorValues[i] > sensorThreshold[i]) {
      if (sensorValues[i] < maxSensorValues[i]) {
        sensorValues[i] = ((sensorValues[i] - sensorThreshold[i]) * 1000) / range;
      } else {
        sensorValues[i] = 1000;
      }
    } else {
      sensorValues[i] = 0;
    }
  }
  
}

unsigned int readLine() {
  uint32_t weightedSum = 0;
  uint32_t sum = 0;
  isOnLine = false;

  readCalibratedSensors();
  if(LineBW){
    for (byte i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = 1000-sensorValues[i];
    }
  }

  for (byte i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i]) {
      isOnLine = true;
      weightedSum += (uint32_t)sensorValues[i] * i * 1000;
      sum += sensorValues[i];
    }
  }

  return (weightedSum > 0) ? weightedSum / sum : 0;
}

void calibrateSensors() {
  for (byte i = 0; i < NUM_SENSORS; i++) {
    maxSensorValues[i] = maxSensorValues[i]-50;
    minSensorValues[i] = minSensorValues[i]+50;
  }

  for (byte j = 0; j < CALIBRATION_COLLECTOR_SAMPLES; j++) {
    int sum[NUM_SENSORS] = {0};
    for (byte k = 0; k < CALIBRATION_AVG_SAMPLES; k++) {
      readRawSensors();
      for (byte i = 0; i < NUM_SENSORS; i++) {
        sum[i] += sensorValues[i];
      }
    }

    for (byte i = 0; i < NUM_SENSORS; i++) {
      int avg = sum[i] / CALIBRATION_AVG_SAMPLES;
      if (avg > maxSensorValues[i]) maxSensorValues[i] = avg;
      if (avg < minSensorValues[i]) minSensorValues[i] = avg;
    }
  }
}
