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
#include <MS5611.h> // Include the MS5611 library
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


void beepDigit(int count) {
  for (int i = 0; i < count; i++) {
    tone(BUZ, 4000); delay(1000);
    noTone(BUZ); delay(1000);
  }
}

void setup()
{
  Serial.begin(9600);

  tone(BUZ,4000);
  delay(2000);
  noTone(BUZ);

  pinMode(VOLT_Check, INPUT);
  analogReadResolution(12);
  float adcvoltage = analogRead(VOLT_Check);
  Serial.println(adcvoltage);
  float voltage = 5.096 * 3.3 * (adcvoltage / 4095.0);  //5.096 is the voltage divider factor 
  Serial.println(voltage/5.096);
  Serial.print(voltage);

  int voltageInt = (int)round(voltage * 100); 
  int hundreds = voltageInt / 100;     
  Serial.println(hundreds);   
  int tens = (voltageInt / 10) % 10;     
  int ones = voltageInt % 10;           
    

  beepDigit(hundreds+1);
  delay(2000);  // Pause between parts

  beepDigit(tens);
  delay(2000);

  beepDigit(ones);
}

void loop()
{

}