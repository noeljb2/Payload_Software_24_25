//Payload baro altitude with moving average 
#include "MS5611_NonBlocking.h"
#include <Wire.h>
#include <SD.h>

MS5611_NonBlocking ms5611(0x77, &Wire1);
//patload Station Pin def
#define GPS_TX 0
#define GPS_RX 1
#define LORA_RESET 3
#define BUZ 5
#define TX_CAM 7
#define RX_CAM 8
#define LORA_BUSY 9
#define LORA_CS 10
#define LORA_MISO 12
#define LORA_MOSI 11
#define LORA_DIO1 24
#define CURRENT_SENSE 26
#define LIGHT_SENSER 27
#define SOLENOID1 23
#define SOLENOID2 22
#define PAYLOAD_STATE 20
#define CONT2 15
#define CONT1 14
float groundPressure = 0.0; //baseline pressure
float groundAltitude = 0.0; 
float altitude = 0.0; //raise warning if altitude is 0

#define PRESSURE_AVG_SIZE 30
float pressureSamples[PRESSURE_AVG_SIZE];
int pressureIndex = 0;
volatile double pressureSum = 0.0;
volatile double movingAverage = 0.0;

unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 100; // ms
File logFile;


//lift off detection variables
bool liftoffDetected = false;
float previousAltitude = 0.0;
unsigned long previousTime = 0;
const float LIFTOFF_THRESHOLD = 3.0; //m/s
const float ALTITUDE_CHANGE_THRESHOLD = 2.0; //meters
int detectionCount = 0; //for debounce
const int REQUIRED_CONSECUTIVE_DETECTIONS = 3;


#define led 13


//sample count 
int samplesCount = 0;  // Tracks how many valid samples are in the buffer

//appogee 
float maxAltitude = 0.0;
bool apogeeLogged = false;
bool liftoffDone =false;

float rawPressure=0.0;
float temp = 0.0;

bool barBaselineSet = false;
double baselinePressure = 0.0;

void setup() {
  delay(3000);

  Serial.begin(9600);
  pinMode(led,OUTPUT);
  pinMode(BUZ, OUTPUT);
  pinMode(SOLENOID1, OUTPUT);
  pinMode(SOLENOID2,OUTPUT);
  if (!ms5611.begin()) {
    Serial.println("MS5611 not detected!");
    while (1);
  }
 

  ms5611.setOSR(MS5611_NonBlocking::OSR_4096);  // Use highest resolution
  ms5611.update();


  // After ms5611.begin() and OSR set
  float pressureTotal = 0;
  for (int i = 0; i < PRESSURE_AVG_SIZE; i++) {
    while (!ms5611.dataReady()) {
      ms5611.update();
    }
    pressureTotal += ms5611.getPressure();
    Serial.print("P: "); Serial.println(ms5611.getPressure());

    delay(5);  // Give sensor time
  }
  
  groundPressure = pressureTotal / PRESSURE_AVG_SIZE *100;
  groundAltitude = 44330.0 * (1.0 - pow(groundPressure / 101325.0, 0.1903));

  Serial.print("Ground Pressure: "); Serial.println(groundPressure);
  Serial.print("Ground Altitude: "); Serial.println(groundAltitude);

  pressureSum = 0;
  for (int i = 0; i < PRESSURE_AVG_SIZE; i++) {
    pressureSamples[i] = 0;
  }


  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");
   char filename[13];
  for (int i = 1; i < 1000; i++) {
    sprintf(filename, "log%03d.csv", i);
    if (!SD.exists(filename)) {
      logFile = SD.open(filename, FILE_WRITE);
      if (logFile) {
        Serial.print("Logging to: ");
        Serial.println(filename);
        logFile.println("Time_ms,Pressure_Pa,Temp_C,Altitude_Moving_Average");
      } else {
        Serial.println("Failed to create log file.");
      }
      break;
    }
  }

}

void loop() {
    ms5611.update();
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) 
    {
        lastUpdateTime = currentTime;  // Update the time for the next cycle
        if (ms5611.dataReady()) 
        { 
          rawPressure = ms5611.getPressure();
          pressureSum -= pressureSamples[pressureIndex];
          pressureSamples[pressureIndex] = rawPressure;
          pressureSum += rawPressure;
          pressureIndex = (pressureIndex + 1) % PRESSURE_AVG_SIZE;

          movingAverage = pressureSum / PRESSURE_AVG_SIZE *100;

          altitude = 44330.0 * (1.0 - pow(movingAverage / 101325.0, 0.1903));
          temp = ms5611.getTemperature();
          Serial.println(altitude);
          
          // Set baseline after buffer fills
          if (!barBaselineSet && pressureIndex == 0) {  
            baselinePressure = movingAverage;
            barBaselineSet = true;
            Serial.println("Baseline pressure set for relative altitude calculation.");

            previousAltitude = 44330.0 * (1.0 - pow(movingAverage / baselinePressure, 0.1903));
            previousTime = micros();
          }
        }

        if(barBaselineSet)
        {
          if (altitude > maxAltitude) {
            maxAltitude = altitude;
          }
        }
        if (logFile) 
        {
            logFile.print(currentTime);
            logFile.print(",");
            logFile.print(rawPressure, 2);
            logFile.print(",");
            logFile.print(temp, 2);
            logFile.print(",");
            logFile.println(altitude, 2);
            logFile.flush();
        }

          if (!liftoffDetected && barBaselineSet) 
          {
            if ((altitude - groundAltitude) > 50) {
              detectionCount++;
              if (detectionCount >= REQUIRED_CONSECUTIVE_DETECTIONS) {
                liftoffDetected = true;
                Serial.println("Liftoff Detected!");
                digitalWrite(SOLENOID1, HIGH);
                if (logFile) {
                  logFile.println("Liftoff Detected!");
                }
              }
            } else {
              detectionCount = 0; // Reset if condition not met continuously
            }
          }
      previousAltitude = altitude;
      previousTime = currentTime;
    }
    

    if (!apogeeLogged && altitude < (maxAltitude - 1.0) && liftoffDetected) {
            Serial.println("Apogee Detected!");
            logFile.println("Apogee:");
            logFile.println(maxAltitude-groundAltitude);
            digitalWrite(SOLENOID2, HIGH);
            logFile.close();
            tone(BUZ, 2000, 300);
            apogeeLogged = true;
    }
  
  }

