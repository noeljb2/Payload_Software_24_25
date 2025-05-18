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

//GPS serial
#define mySerial Serial1 


//LoRa def
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RESET, LORA_BUSY);

float Lora_Frequency = 915.0; //915MHz
int LORA_BANDWIDTH = 0; //125kHz
int LORA_SPREADING_FACTOR = 7; //SF7
int LORA_CODING_RATE = 1; //4/5
int LORA_SYNC_WORD = 0x12; //0x12
int LORA_POWER = 20; //20dBm
int LORA_PREAMBLE_LENGTH = 8; //8 bytes



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


/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;  //100HZ 


//MS5611 def
MS5611 ms5611(0x77);


void setup()
{
  Serial.begin(9600);
  Wire.begin();

  pinMode(BUZ, OUTPUT);
  pinMode(CONT1, INPUT);
  pinMode(CONT2, INPUT);
  pinMode(CURRENT_SENSE, OUTPUT);
  pinMode(LIGHT_SENSER, INPUT);
  pinMode(SOLENOID1, OUTPUT);
  pinMode(SOLENOID2, OUTPUT);
  pinMode(VOLT_Check, INPUT);

  //Baro Sensor Initialization
  if (!ms5611.begin()) 
  {
    Serial.println("MS5611 not found or failed to initialize!");
    while (1);
  }

  //LoRa Initialization
  radio.begin(Lora_Frequency, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODING_RATE, LORA_SYNC_WORD, LORA_POWER, LORA_PREAMBLE_LENGTH);



  //GPS Initialization
  mySerial.begin(9600);
  myGNSS.setUART1Output(COM_TYPE_UBX);
  
  // Set up auto PVT message (non-blocking)
  myGNSS.setNavigationFrequency(1); // 1 Hz
  myGNSS.setAutoPVT(true); // Enable automatic NAV PVT messages
  myGNSS.setAutoPVTcallbackPtr(&pvtCallback); // Register callback function


  //beeping battery voltage 

  if(voltage > 0.130 && voltage < 0.180) // voltage divider should read: 0.176 V for the volt check analog read, SO a range is set
  {
    Serial.println("Battery Voltage is good");
  }
  else
  {
    Serial.println("Battery Voltage is bad");
    MyTim->setPWM(channel, BUZ, 4000, 0); // 4kHz , 0% dutycycle
    delay(50);
    MyTim->setPWM(channel, BUZ, 4000, 50); // 4kHz, 10% dutycycle
    delay(50);
    while(1);
  }



  //BATTERY VOLTAGE BEEP CHECK
  float batteryVoltage = 3.3 * (analogRead(VOLT_Check) / 1023.0);

  if(batteryVoltage > 0.130 && batteryVoltage < 0.180) // voltage divider should read: 0.176 V for the volt check analog read, SO a range is set
  {
    Serial.println("Battery Voltage is good");
    // Store each decimal digit of batteryVoltage into an int array
    int digits[6]; // Enough for e.g. "0.1760"
    int temp = (int)(batteryVoltage * 10000); // Shift 4 decimal places
    for (int i = 0; i < 6; i++) {
      digits[5 - i] = temp % 10;
      temp /= 10;
    }

    //Buzzes the battery voltage 
    for(int j=0;j<digits.length;j++)
    {
      for (int i=0; i<digits[j]; i++)
      {
        MyTim->setPWM(channel, BUZ, 4000, 50); // 4kHz, 10% dutycycle
        delay(50);
        MyTim->setPWM(channel, BUZ, 4000, 0); // 4kHz , 0% dutycycle
        delay(100);
      }
    }
    
  }
  else
  {
    Serial.println("Battery Voltage is bad");
    MyTim->setPWM(channel, BUZ, 4000, 0); // 4kHz , 0% dutycycle
    delay(50);
    MyTim->setPWM(channel, BUZ, 4000, 50); // 4kHz, 10% dutycycle
    delay(50);
    while(1);
  }

  







}
