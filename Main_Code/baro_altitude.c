//Payload baro altitude with moving average 
#include "MS5611_NonBlocking.h"
#include <Wire.h>


MS5611_NonBlocking ms5611(0x77, &Wire1);

float groundPressure = 0.0; //baseline pressure
float groundAltitude = 0.0; 
float altitude = 0.0 //raise warning if altitude is 0

#define PRESSURE_AVG_SIZE 30
float pressureBuffer[PRESSURE_AVG_SIZE];
int pressureIndex = 0;
bool pressureBufferFilled = false;
volatile double pressureSum = 0.0;
volatile double movingAverage = 0.0;
double baselinePressure = 0.0;


void setup() {
  //delay before serial connection
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

}

void loop() {
  //start conversion
  ms5611.update();






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
    

  }

  
}