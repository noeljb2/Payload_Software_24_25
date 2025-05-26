//Payload baro altitude with moving average 
#include "MS5611_NonBlocking.h"
#include <Wire.h>
#include <SD.h>

MS5611_NonBlocking ms5611(0x77, &Wire1);

float groundPressure = 0.0; //baseline pressure
float groundAltitude = 0.0; 
float altitude = 0.0; //raise warning if altitude is 0

#define PRESSURE_AVG_SIZE 30
float pressureSamples[PRESSURE_AVG_SIZE];
int pressureIndex = 0;
volatile double pressureSum = 0.0;
volatile double movingAverage = 0.0;


File logFile;


//lift off detection variables
bool liftoffDetected = false;
float previousAltitude = 0.0;
unsigned long previousTime = 0;
const float LIFTOFF_THRESHOLD = 3.0; //m/s
const float ALTITUDE_CHANGE_THRESHOLD = 2.0; //meters
int detectionCount = 0; //for debounce
const int REQUIRED_CONSECUTIVE_DETECTIONS = 3;


void setup() {
  delay(3000);

  Serial.begin(9600);

  if (!ms5611.begin()) {
    Serial.println("MS5611 not detected!");
    while (1);
  }
 

  ms5611.setOSR(MS5611_NonBlocking::OSR_4096);  // Use highest resolution
  ms5611.update();

  for (int i = 0; i < PRESSURE_AVG_SIZE; i++) {
    pressureSamples[i] = 0.0;
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
        logFile.println("Log Start");
      } else {
        Serial.println("Failed to create log file.");
      }
      break;
    }
  }

}

void loop() {
    //start conversion
    ms5611.update();
    unsigned long currentTime = millis();  
    float deltaTime = (currentTime - previousTime) / 1000.0; // seconds
    //when conversion is ready get newest data
    if (ms5611.dataReady()) {
        ms5611.read();
        float rawPressure = ms5611.getPressure();
        pressureSum -= pressureSamples[pressureIndex];
        pressureSamples[pressureIndex] = rawPressure;
        pressureSum += rawPressure;
        pressureIndex = (pressureIndex + 1) % PRESSURE_AVG_SIZE;

        movingAverage = pressureSum / PRESSURE_AVG_SIZE;

        altitude = 44330.0 * (1.0 - pow(movingAverage / 101325.0, 0.1903));
        float temp = ms5611.getTemperature();
        float altbaro = ms5611.getAltitude();
        if(logFile)
        {
        logFile.print(millis());
        logFile.print(",");
        logFile.print(rawPressure, 2);
        logFile.print(",");
        logFile.print(temp, 2);
        logFile.print(",");
        logFile.print(altitude, 2);
        logFile.print(",");
        logFile.println(altbaro, 2);
        }

    }



     


    if (deltaTime > 0.0 && previousTime > 0) 
    {
        float ascentRate = (altitude - previousAltitude) / deltaTime;


        if (ascentRate > LIFTOFF_THRESHOLD && (altitude - groundAltitude) > ALTITUDE_CHANGE_THRESHOLD) 
        {
            detectionCount++;
            if (detectionCount >= REQUIRED_CONSECUTIVE_DETECTIONS) 
            {
            liftoffDetected = true;
            }
        } else {
            detectionCount = 0;
        }
    }










    previousAltitude = altitude;
    previousTime = currentTime;


    if(millis()>10000 && logFile) 
    {
        logFile.close();
        Serial.println("Log file closed.");
        while(true); //stop logging
    }
  
}