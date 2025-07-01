// =====================
// USER PARAMETERS
// =====================
#include <Servo.h>
#define KP 0.015
#define KD 0.2
#define SPEED 30
#define RUN_TIME_MS 3000
#define MAX_OUT_TIME_MS 10
#define SENSOR_THRESHOLD_PERCENT 45

#define CALIBRATION_AVG_SAMPLES 8
#define CALIBRATION_COLLECTOR_SAMPLES 10

// =====================
// PIN MAPPING
// (STM32 pin names)
// =====================
#define R_AIN1  8
#define R_AIN2  7
#define R_PWMA  6
   
#define L_BIN1  4
#define L_BIN2  2
#define L_PWMB  3
 

#define PIN_MUX_S0 9
#define PIN_MUX_S1 10
#define PIN_MUX_S2 11
#define PIN_MUX_S3 12
#define PIN_MUX_SIG A0

// =====================
// CONSTANTS
// =====================
#define NUM_SENSORS 16

// =====================
// GLOBAL VARIABLES
// =====================
int sensorValues[NUM_SENSORS];
unsigned int maxSensorValues[NUM_SENSORS]={791	,784,	799	,745,	726,	686,	720,	754,	777,	775	,780,	799,797,	806	,833,	859};
unsigned int minSensorValues[NUM_SENSORS]={40	,37	,43	,34,	34,	34,	34,	35,	35,	36,	35,	36,	35,	35,	36,	39};
int sensorThreshold[NUM_SENSORS];

bool isOnLine = false;
bool lastDetectedSide = false;
int previousError = 0;

bool LineBW = true;   //white = true ;  black = false ;
// =====================
// SETUP
// =====================
int currentSpeed = 0;
unsigned long lastUpdateTime = 0;
const int updateInterval = 20;

Servo esc;
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
  escbegin(5);
  Switch1();
  setSpeed(35);
  delay(2000);
  robotRun();
   breakHard();
 emergencyStop();
  //รอปุ่มกดเริ่มวิ่ง
// motor(60,-60);
// delay(500);
// motor(-60,60);
// delay(500);
    breakHard();
}
void loop(){
 //Readcalibreate();
//  readSerialRawSensors();
//   delay(100);
// 
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

      motor(pwmL,pwmR);
    

      outStartTime = currentTime;
    } else {
      if (currentTime - outStartTime >= MAX_OUT_TIME_MS) {
        running = false;
        break;
      }
      if (!lastDetectedSide) {
        motor(-100,100);
      } else {
        motor(100,-100);
       
      }
    }

    if (currentTime - startTime >= RUN_TIME_MS) {
      running = false;
      break;
    }
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
    sensorValues[NUM_SENSORS - 1 - i] = analogRead(PIN_MUX_SIG);
    // sensorValues[i] = analogRead(PIN_MUX_SIG);
  }
}
void readSerialRawSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(i);
    delayMicroseconds(5);
     sensorValues[NUM_SENSORS - 1 - i] = analogRead(PIN_MUX_SIG);
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
    maxSensorValues[i] = 1023;
    minSensorValues[i] = 0;
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



void motor(int speedL, int speedR) {
        // Control Motor R
        if (speedR >= 0) {
            digitalWrite(R_AIN1, LOW);
            digitalWrite(R_AIN2, HIGH);
        } else {
            digitalWrite(R_AIN1, HIGH);
            digitalWrite(R_AIN2, LOW);
            speedR = -speedR; // Make speed positive
        }
        analogWrite(R_PWMA, speedR*2.55 ); // Scale 0-100 to 0-255

        // Control Motor L
        if (speedL >= 0) {
            digitalWrite(L_BIN1, LOW);
            digitalWrite(L_BIN2, HIGH);
        } else {
            digitalWrite(L_BIN1, HIGH);
            digitalWrite(L_BIN2, LOW);
            speedL = -speedL; // Make speed positive
        }
        analogWrite(L_PWMB, speedL*2.55); // Scale 0-100 to 0-255
    }
     void Switch1(){
        while(1){
          if(!analogRead(A7))break;
        }
      }


       void escbegin(int pin) {
       
        esc.attach(pin);
        esc.writeMicroseconds(1000);
        delay(1000);
    }
  void setSpeed(int speed) {
        currentSpeed = constrain(speed, 0, 100);
        updateESC();
  }
 void updateESC() {
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= updateInterval) {
            int pulseWidth = map(currentSpeed, 0, 100, 1000, 2000);
            esc.writeMicroseconds(pulseWidth);
            lastUpdateTime = currentTime;
        }
    }

    void emergencyStop() {
        currentSpeed = 0;
        esc.writeMicroseconds(1000);
    }
      void breakHard() {
        // Set both motors to LOW to stop them immediately
        digitalWrite(R_AIN1, HIGH);
        digitalWrite(R_AIN2, HIGH);
        digitalWrite(L_BIN1, HIGH);
        digitalWrite(L_BIN2, HIGH);
        analogWrite(R_PWMA, 180); // Set PWM to 0 to stop the motor
        analogWrite(L_PWMB, 180); // Set PWM to 0 to stop the motor
    }