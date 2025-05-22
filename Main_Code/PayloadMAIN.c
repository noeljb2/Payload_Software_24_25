//Noel Johnbosco 
//5/14/2025


//  Payload Abstracted Overview 

/*
  Must power on, beep battery voltage, and beep a standby armed-and-ready mode
  Must be able to receive GPS information and transmit it to your ground station (there will be a separate ground station for the payload)
  Must be able to detect a successful nose cone separation (light sensor)
  This is when the camera starts recording
  Must be able to control the separation from the tether (pyro output to tender descender) [altitude based]
  Must be able to control the deployment of the parachute (pyro output to line cutter) [altitude based]
*/


//  Soar_24_25 Payload Computer Software
#include <Wire.h>
#include <RadioLib.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <MS5611.h> 


//  Ground Station Pin def
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

//  GPS serial
#define mySerial Serial1 


//  LoRa def
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RESET, LORA_BUSY);

float Lora_Frequency = 915.0; //915MHz
int LORA_BANDWIDTH = 0; //125kHz
int LORA_SPREADING_FACTOR = 7; //SF7
int LORA_CODING_RATE = 1; //4/5
int LORA_SYNC_WORD = 0x12; //0x12
int LORA_POWER = 20; //20dBm
int LORA_PREAMBLE_LENGTH = 8; //8 bytes



//  GPS def
SFE_UBLOX_GNSS_SERIAL myGNSS;
//set gps higher acceleration threshold find in library  ******

//  GNSS Values
unsigned long lastGNSS = 0;                    
long latitude = 0;
long longitude = 0;
long gnssAltitude = 0;
byte SIV = 0;


//  MS5611 def w moving avg def
MS5611 ms5611(0x77,&Wire1);
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

//  Light sensor Basline 
float LightVoltage = 0.0;


//  Flight states 

bool DetectAppogee = false;  
bool DetectNoseConeSeperation = false; //based on light sensor 
bool TetherSeperation = false;
bool MainParachute = false; 


// Timing Variables 
unsigned long currentMicros;
unsigned long lastBeepTime = 0;
unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 500000; // 500 ms change this to increase or decrease loop cycle

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// save transmission state between loops
volatile int transmissionState = RADIOLIB_ERR_NONE;

void setFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

void beepDigit(int count) {
  for (int i = 0; i < count; i++) {
    tone(BUZ, 4000); delay(1000);
    noTone(BUZ); delay(1000);
  }
}

void errorCode()
{
  //some buz and a while to stop the code

}

void setup()
{
  Serial.begin(9600);
  Wire1.begin();

  pinMode(BUZ, OUTPUT);
  pinMode(CONT1, INPUT);
  pinMode(CONT2, INPUT);
  pinMode(CURRENT_SENSE, OUTPUT);
  pinMode(LIGHT_SENSER, INPUT);
  pinMode(SOLENOID1, OUTPUT);
  pinMode(SOLENOID2, OUTPUT);
  pinMode(VOLT_Check, INPUT);


  // ----Cont Check-----
  if(!(analogRead(CONT1) >= 500)){
    Serial.println(analogRead(CONT1));
    Serial.println("No cont1");
    errorCode();
  }
  if(!(analogRead(CONT2) >= 500)){
    Serial.println(analogRead(CONT2));
    Serial.println("No cont2");
    errorCode();
  }



  // -------Baro Sensor Initialization-----
  if (!ms5611.begin()) 
  {
    Serial.println("MS5611 not found or failed to initialize!");
    while (1);
  }
  ms5611.setOversampling(OSR_ULTRA_HIGH);
  ms5611.read();
  groundPressure = ms5611.getPressurePascal();
  groundAltitude = 44330.0 * (1.0 - pow(filteredPressure / 101325.0, 0.1903));

  // --------LoRa Initialization----
  int state = radio.begin(Lora_Frequency, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODING_RATE, LORA_SYNC_WORD, LORA_POWER, LORA_PREAMBLE_LENGTH);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  radio.setCurrentLimit(140);
  radio.setPacketSentAction(setFlag);

  for (int i = 0; i < PRESSURE_AVG_SIZE; i++) {
    pressureSamples[i] = 0.0;
  }

  // ----GPS Initialization------
  unsigned long gnssStart = millis();
  do {
    Serial.println("GNSS: trying 38400 baud");
    mySerial.begin(38400);
    if (myGNSS.begin(mySerial) == true) break;
    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    mySerial.begin(9600);
    if (myGNSS.begin(mySerial) == true) {
        Serial.println("GNSS: connected at 9600 baud, switching to 38400");
        myGNSS.setSerialRate(38400);
        delay(100);
    } else {
        //myGNSS.factoryDefault();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
    if(millis() - gnssStart >= 10000){
      errorCode1();
    }
  } while(1);
  Serial.println("GNSS serial connected");
  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
  

  //***** SET UP NON BLOCKING for this module



  // -----Voltage beep----------
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
  delay(2000);  // pause between parts
  beepDigit(tens);
  delay(2000);
  beepDigit(ones);

  
  // -------light Sensor Calibration----

  //essentially polls light sensor data to set a baseline for the payload inside rocket





  
}


//            ****  LOOP  ****


void loop()
{
  currentMicros = micros();


  if (currentMicros - lastLoopTime >= loopInterval) //sets a loop rate so this is set at 500 ms 
  {
    lastLoopTime = currentMicros;

    ms5611.read();
    float rawPressure = ms5611.getPressurePascal();
    pressureSum -= pressureSamples[pressureIndex];
    pressureSamples[pressureIndex] = rawPressure;
    pressureSum += rawPressure;
    pressureIndex = (pressureIndex + 1) % PRESSURE_AVG_SIZE;

    movingAverage = pressureSum / PRESSURE_AVG_SIZE;

    altitude = 44330.0 * (1.0 - pow(movingAverage / 101325.0, 0.1903));

    latitude = myGNSS.getLatitude();
    longitude = myGNSS.getLongitude();
    gnssAltitude = myGNSS.getaltitude();  //optional? 
    SIV = myGNSS.getSIV(); //probably optional ****
 
    String data = String(latitude) + "," + String(longitude) + "," + String(altitude);
    radio.transmit(data);

    int lightLevel = analogRead(LIGHT_SENSOR);


    // ---State Machine----
    if (!NoseConeSeparated && lightLevel > baseline + 300) { //tune this numeber 
      NoseConeSeparated = true;
      digitalWrite(TX_CAM, HIGH); //what ever the intialization for camera is **********************
    }
    if (!TetherReleased && altitude > 200) { //set altitude at a height 
      TetherReleased = true;
      digitalWrite(SOLENOID1, HIGH);
      unsigned long fireStart = micros();
      while (micros() - fireStart < 1000000);
      digitalWrite(SOLENOID1, LOW);
    }
    if (!ParachuteDeployed && TetherReleased && altitude < 100) { //parachute deployed at 100m atm...****?
      ParachuteDeployed = true;
      digitalWrite(SOLENOID2, HIGH);
      unsigned long fireStart = micros();
      while (micros() - fireStart < 1000000); //waiting for the ematch or co2 to be fully deployed? ********
      digitalWrite(SOLENOID2, LOW);
    }


  }
  
}
