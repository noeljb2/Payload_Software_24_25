//Noel Johnbosco 
//5/14/2025


//Payload Abstracted Overview 

/*
  Must power on, beep battery voltage, and beep a standby armed-and-ready mode
  Must be able to receive GPS information and transmit it to your ground station (there will be a separate ground station for the payload)
  Must be able to detect a successful nose cone separation (light sensor)
  This is when the camera starts recording
  Must be able to control the separation from the tether (pyro output to tender descender) [altitude based]
  Must be able to control the deployment of the parachute (pyro output to line cutter) [altitude based]
*/



//Soar_24_25 Payload Computer Software
#include <Wire.h>
#include <RadioLib.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <MS5611.h> // Include the MS5611 library


//Ground Station Pin def
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

//GPS serial
#define mySerial Serial1 


//LoRa def
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RESET, LORA_BUSY);


//GPS def
SFE_UBLOX_GNSS_SERIAL myGNSS;



//Piezo Buzzer PWM Timer Setup
TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(BUZ), PinMap_PWM);
uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(BUZ), PinMap_PWM));
HardwareTimer *MyTim = new HardwareTimer(Instance);

//GNSS Values
unsigned long lastGNSS = 0;                    
long latitude = 0;
long longitude = 0;
long gnssAltitude = 0;
byte SIV = 0;

#define numSamples 10 // Number of samples for moving average

// Moving Average Variables for Barometric Pressure
float pressureSamples[numSamples];
uint sampleIndex = 0;
volatile float pressureSum = 0.0;
volatile float movingAverage = 0.0;
float baselinePressure = 0.0;                      
unsigned long trendStartTime = 0;
float trendStartAltitude = 0.0;

// Time variables
const unsigned long baroInterval = 50000;      // 50000us interval for MS5611
unsigned long lastBaroTime = 0;


// Barometric Sensor Variables
float lastAltitude = 0.0;
float currentAltitude = 0.0;

MS5611 ms5611(0x77);

void setup() 
{
  Serial.begin(9600);
  Wire.begin();

  if (!ms5611.begin()) {
    Serial.println("MS5611 not found or failed to initialize!");
    while (1);
  }

  ms5611.setOversampling(OSR_ULTRA_HIGH); // Best resolution, ~9ms conversion time

  //second order temperature compensation
  ms5611.setCompensation(true);


  trendStartTime = micros();
}




void loop() 
{
  static unsigned long lastBaroMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastBaroMillis >= 100) { // 100 ms non-blocking delay
    lastBaroMillis = currentMillis;

    int status = ms5611.read();
    if (status == MS5611_READ_OK) {
      float temperature = ms5611.getTemperature();     // in °C
      float pressure = ms5611.getPressure();           // in mbar (hPa)

      // Update moving average buffer
      pressureSum -= pressureSamples[sampleIndex];
      pressureSamples[sampleIndex] = realPressure;
      pressureSum += realPressure;
      sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

      movingAverage = pressureSum / NUM_SAMPLES;

      // Set baseline after buffer is filled
      if (!barBaselineSet && sampleIndex == 0) {
          baselinePressure = movingAverage;
          barBaselineSet = true;
          Serial.println("Baseline pressure set.");
      }
      // Compute altitude and barometric velocity
      if (barBaselineSet) {
          currentAltitude = 44330.0 * (1.0 - pow(movingAverage / baselinePressure, 0.1903));

          unsigned long currentTime = micros();
          if ((currentTime - trendStartTime) >= 10000) {  //0.01s calculations
            float deltaAltitude = currentAltitude - trendStartAltitude;
            float deltaTime = (currentTime - trendStartTime) / 1.0e6;
            baroVelocity = deltaAltitude / deltaTime;

            trendStartAltitude = currentAltitude;
            trendStartTime = currentTime;
          }

          Serial.print("Temp: ");
          Serial.print(realTemperature);
          Serial.print(" °C, Pressure: ");
          Serial.print(realPressure);
          Serial.print(" mBar, Altitude: ");
          Serial.print(currentAltitude);
          Serial.print(" m, Velocity: ");
          Serial.print(baroVelocity);
          Serial.println(" m/s");
      }
    }
  }
}