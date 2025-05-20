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

#include <Wire.h>
#include <RadioLib.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "MS5611.h" // Include the MS5611 library
#include <math.h>

//Ground Station Pin def
#define GPS_TX 0
#define GPS_RX 1
#define LORA_RESET 3
#define BUZ 5
#define TX_CAM 7
#define RX_CAM 8
#define VOLT_Check 25
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


MS5611 MS5611(0x77,&Wire1);


uint32_t start, stop;

void setup()
{
  Serial.begin(115200);

  Wire1.begin();
  if (MS5611.begin() == true)
  {
    Serial.print("MS5611 found: ");
    Serial.println(MS5611.getAddress());
  }
  else
  {
    Serial.println("MS5611 not found. halt.");
    while (1);
  }
  MS5611.setOversampling(OSR_ULTRA_HIGH);

}

void loop()
{
  MS5611.read();           //  note no error checking => "optimistic".
  float pressurePascal = MS5611.getPressurePascal();
  Serial.print(MS5611.getTemperature(), 2);
  Serial.print('\t');
  Serial.print(pressurePascal, 2);
  Serial.print('\t');
  float altitude = 44330.0 * (1.0 - pow(pressurePascal / 101325.0, 0.1903));
  Serial.print(altitude,2);
  Serial.println();
  delay(1000);
}