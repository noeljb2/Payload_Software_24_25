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
#include "MS5611_NonBlocking.h"

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
MS5611_NonBlocking ms5611(0x77, &Wire1);
float groundPressure = 0.0; //baseline pressure
float groundAltitude = 0.0; 
float altitude = 0.0 //raise warning if altitude is 0

#define PRESSURE_AVG_SIZE 30
float pressureSamples[PRESSURE_AVG_SIZE] = {0};
int pressureIndex = 0;
volatile double pressureSum = 0.0;
volatile double movingAverage = 0.0;

unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 100; // ms

//  Light sensor Basline w Constants
const float ADC_RESOLUTION = 1023.0;
const float REF_VOLTAGE = 3.3;
const int BUFFER_SIZE = 30;
const int LOG_BATCH_SIZE = 100;  // Log every 100 samples

float basevoltageread= 0.0;
float Volt_Threshold= 0.0;
bool thresholdReach = false;





// Light Moving average
float voltageBuffer[BUFFER_SIZE] = {0};
int bufferIndex = 0;
int bufferCount = 0;
float voltageSum = 0;



// Lift off Detection Variables
bool liftoffDetected = false;
float previousAltitude = 0.0;
unsigned long previousTime = 0;
const float LIFTOFF_THRESHOLD = 3.0; //m/s
const float ALTITUDE_CHANGE_THRESHOLD = 2.0; //meters
int detectionCount = 0; //for debounce
const int REQUIRED_CONSECUTIVE_DETECTIONS = 3;
float maxAltitude = 0.0;
bool apogeeLogged = false;
bool liftoffDone = false;



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
  for (int i = 0; i < count; i++) 
  {
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
  pinMode(LIGHT_SENSER, INPUT);
  pinMode(SOLENOID1, OUTPUT);
  pinMode(SOLENOID2, OUTPUT);
  pinMode(VOLT_Check, INPUT);

  //tell run cam to stop recording **when run cam is powered it starts recording 

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
    errorCode1();
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
  groundPressure = ms5611.getPressurePascal();
  groundAltitude = 44330.0 * (1.0 - pow(groundPressure / 101325.0, 0.1903));

  pressureSum = 0;
  for (int i = 0; i < PRESSURE_AVG_SIZE; i++) {
    pressureSamples[i] = groundPressure;
    pressureSum += pressureSamples[i];
  }
  movingAverage = pressureSum / PRESSURE_AVG_SIZE;



  // --------LoRa Initialization----
  int state = radio.begin(Lora_Frequency, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODING_RATE, LORA_SYNC_WORD, LORA_POWER, LORA_PREAMBLE_LENGTH);
  if (state == RADIOLIB_ERR_NONE) 
  {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    errorCode1();
    while (true) { delay(10); }
  }

  radio.setCurrentLimit(140);
  radio.setPacketSentAction(setFlag);



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

  // ------SD Initialization--------
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
        errorCode1();
      }
      break;
    }
  }
  // -------light Sensor Calibration----

  //essentially polls light sensor data to set a baseline for the payload inside rocket
  bool baslinebarfilled = false;
  //baseline voltage read from the light sensor 
  analogReadResolution(10);
  while(!baslinebarfilled)
  {
    int rawValue = analogRead(LIGHT_SENSER);
    float voltage = (rawValue / ADC_RESOLUTION) * REF_VOLTAGE;
    voltageSum -= voltageBuffer[bufferIndex];
    voltageBuffer[bufferIndex] = voltage;
    voltageSum += voltage;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    if (bufferCount < BUFFER_SIZE) 
    {
    bufferCount++;
    }else
    {
      baslinebarfilled=true;
    }
  }
  basevoltageread = voltageSum / bufferCount;
  
}


//            ****  LOOP  ****


void loop()
{
  ms5611.update();
  currentMicros = micros();
  if (currentMicros - lastLoopTime >= loopInterval) //sets a loop rate so this is set at 500 ms 
  {
    lastLoopTime = currentMicros;

    if(ms5611.dataRead())
    {
      float rawPressure = ms5611.getPressure();
      pressureSum -= pressureSamples[pressureIndex];
      pressureSamples[pressureIndex] = rawPressure;
      pressureSum += rawPressure;
      pressureIndex = (pressureIndex + 1) % PRESSURE_AVG_SIZE;

      movingAverage = pressureSum / PRESSURE_AVG_SIZE;

      altitude = 44330.0 * (1.0 - pow(movingAverage / 101325.0, 0.1903));
      float temp = ms5611.getTemperature();
      if (altitude > maxAltitude)
      {
        maxAltitude = altitude;
      }  
      if (!liftoffDetected) 
          {
            if ((altitude - groundAltitude) > 50) 
            {
              detectionCount++;
              if (detectionCount >= REQUIRED_CONSECUTIVE_DETECTIONS) 
              {
                liftoffDetected = true;
                Serial.println("Liftoff Detected!");
                digitalWrite(SOLENOID1, HIGH);
                if (logFile) 
                {
                  logFile.println("Liftoff Detected!");
                }
              }
            } else 
            {
              detectionCount = 0; // Reset if condition not met continuously
            }
            previousAltitude = altitude;
            previousTime = currentMicros;
          }
    }
    if(myGNSS.getPVT())
    {
      latitude = myGNSS.getLatitude() / 1e7;
      longitude = myGNSS.getLongitude() / 1e7;
      altitude = myGNSS.getAltitudeMSL() / 1000.0; // Convert to meters
      SIV = myGNSS.getSIV();  // Get number of satellites in view
    }



    
      String data = String(latitude) + "," + String(longitude) + "," + String(altitude);
      radio.transmit(data);

      int lightLevel = analogRead(LIGHT_SENSOR);
  }

  // ----LightLevel-----
  //checking the light level on the photo resistor
  int rawValue = analogRead(LIGHT_SENSER);
  float voltage = (rawValue / ADC_RESOLUTION) * REF_VOLTAGE;

  // Update moving average
  voltageSum -= voltageBuffer[bufferIndex];
  voltageBuffer[bufferIndex] = voltage;
  voltageSum += voltage;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if (bufferCount < BUFFER_SIZE) bufferCount++;

  float averageVoltage = voltageSum / bufferCount;





  // ---State Machine----
  if (!apogeeLogged && altitude < (maxAltitude - 1.0) && liftoffDetected) {
    Serial.println("Apogee Detected!");
    tone(BUZ, 2000, 300);
    apogeeLogged = true;
  }
  if (!NoseConeSeparated && averageVoltage > basevoltageread + 0.5) { //tune this numeber 
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
    while (micros() - fireStart < 1000000); 
    digitalWrite(SOLENOID2, LOW);
  } 
}


void setupCamera() {
    delay(3000);  // Optional delay before init
    rcSerial.begin(115200);
    delay(3000);  // Allow time for camera to initialize
    // Build command to toggle recording
    txBuf[0] = 0xCC;
    txBuf[1] = 0x01;  // Command ID
    txBuf[2] = 0x01;  // Parameter (toggle)
    txBuf[3] = calcCrc(txBuf, 3);
}
void startRecording() {
    if (recState == 0) {
        rcSerial.write(txBuf, 4);
        recState = 1;
    }
}
void stopRecording() {
    if (recState == 1) {
        rcSerial.write(txBuf, 4);
        recState = 0;
    }
}
uint8_t calcCrc(uint8_t *buf, uint8_t numBytes) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < numBytes; i++)
        crc = crc8_calc(crc, *(buf + i), 0xD5);
    return crc;
}
uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80)
            crc = (crc << 1) ^ poly;
        else
            crc = crc << 1;
    }
    return crc;
}


