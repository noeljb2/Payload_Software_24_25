//Code for Receiving GPS and displaying on OLED Screen
#include <RadioLib.h>
#include <Wire.h>
#include <SparkFun_Qwiic_OLED.h>

//Ground Station Pins
#define Record 9
#define VoltCheck 16

#define LoRaDIO1 23
#define LoRa_BUSY 18
#define LoRa_RST 17
#define LoRa_CS 37
#define RX_EN 0
#define TX_EN 1
#include <res/qw_fnt_5x7.h>


SX1262 radio = new Module(LoRa_CS, LoRaDIO1, LoRa_RST, LoRa_BUSY);
QwiicMicroOLED myOLED;
// flag to indicate that a packet was received
volatile bool receivedFlag = false;
// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}


void setup() 
{
  Serial.begin(9600);
  delay(1000);
  // Start Wire2 (I2C3) explicitly
  //Wire2.begin();

  /*if (!myOLED.begin(Wire2, 0x3D)) 
  { 
      Serial.println("OLED Screen not Initialized");
      while (1); 
  }
  myOLED.reset(true); 
  myOLED.display();
  myOLED.setFont(&QW_FONT_5X7);*/
  radio.setPacketReceivedAction(setFlag);
  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin(915.0,125,8,5,RADIOLIB_SX126X_SYNC_WORD_PRIVATE,22,8,1.6,false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
  radio.setDio1Action(setFlag);
  radio.setRfSwitchPins(RX_EN, TX_EN);
  
  
  radio.startReceive();
  Serial.println("Waiting for GPS Signal...");

  //myOLED.reset(true);
  //myOLED.setCursor(0, 0);
  //myOLED.print("Waiting for GPS Signal");
  //myOLED.display();
}

void loop() 
{
  if(receivedFlag) 
  {
    // reset flag
    receivedFlag = false;
  
    String str;
    int state = radio.readData(str);

    if (state == RADIOLIB_ERR_NONE) 
    {
      Serial.println(F("[SX1262] Received packet!"));
      Serial.print(F("[SX1262] Data:\t\t"));
      Serial.println(str);

      float longitude = 0.0, latitude = 0.0,altitude=0.0;
      int FlightState;
      if (parseGPS(str, latitude,longitude,altitude,FlightState)) {
        Serial.print(F("Longitude: "));
        Serial.println(longitude, 7); 
        Serial.print(F("Latitude: "));
        Serial.println(latitude, 7);   
        Serial.print(F("altitude: "));
        Serial.println(altitude, 7); 
        Serial.print(F("FlightState: "));
        switch (FlightState)
        {
          case 0:
          Serial.println("Pad");
          break;
          case 1:
          Serial.println("Liftoff");
          break;
          case 2:
          Serial.println("Apogee Detected");
          break;
          case 3:
          Serial.println("NoseConeSeperated");
          break;
          case 4:
          Serial.println("Tether Released");
          break;
          case 5:
          Serial.println("Main Deployed");
          break;
          case 6:
          Serial.println("Landed");
          break;
          default:
          Serial.println("error No Flightstate");
        }
      }
      else 
      {
        Serial.println("GPS Parse Error!");
      }

      // Print RSSI & SNR
      Serial.print(F("[SX1262] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));

      Serial.print(F("[SX1262] SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));
    }
    else 
    {
      Serial.println("Error receiving data");
    }
  }
}



bool parseGPS(String data, float &latitude, float &longitude, float &altitude, int &FlightState) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  int thirdComma = data.indexOf(',', secondComma + 1);

  if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
    return false; 
  }


  // Extract substrings
  String latStr = data.substring(0, firstComma).trim();
  String longStr = data.substring(firstComma + 1, secondComma).trim();
  String altStr = data.substring(secondComma + 1, thirdComma).trim();
  String FlightStr = data.substring(thirdComma+1).trim();


  latitude = latStr.toFloat();
  longitude = longStr.toFloat();
  altitude = altStr.toFloat();
  FlightState = FlightStr.toInt();
  return true;
}


