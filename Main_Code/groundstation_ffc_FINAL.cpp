#include <RadioLib.h>
#include <Wire.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// Ground Station Pins
#define Record 9
#define VoltCheck 16

#define LoRaDIO1 23
#define LoRa_BUSY 18
#define LoRa_RST 17
#define LoRa_CS 37
#define RX_EN 0
#define TX_EN 1

#define frequency 915.0

#define mySerial Serial5 

SFE_UBLOX_GNSS_SERIAL myGNSS;
SX1262 radio = new Module(LoRa_CS, LoRaDIO1, LoRa_RST, LoRa_BUSY);

#define SD_CS BUILTIN_SDCARD  // Teensy 4.1 SD slot
File logFile;

// Flag for LoRa packet reception
volatile bool receivedFlag = false;
int transmissionState = RADIOLIB_ERR_NONE;

void setFlag(void) {
    receivedFlag = true;
}

void setup() {
    Serial.begin(9600);
    delay(1000);
    Wire2.begin();
    mySerial.begin(9600);

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized.");
    char filename[13];
    for (int i = 1; i < 1000; i++) 
    {
        sprintf(filename, "GroundStation%03d.csv", i);
        if (!SD.exists(filename)) 
        {
            logFile = SD.open(filename, FILE_WRITE);
            if (logFile) 
            {
                Serial.print("Logging to: ");
                Serial.println(filename);
                logFile.println("Log Start");
            } else 
            {
                Serial.println("Failed to create log file.");
            }
            break;
        }
    }
    
    if (logFile) {
        logFile.println("Time, PacketNum, Latitude, Longitude, Satellites, Altitude, FlightState, BatVOLT");
        logFile.flush();
    } else {
        Serial.println("Failed to create log file.");
        while(1);
    }


    radio.setPacketReceivedAction(setFlag);
    Serial.print(F("[SX1262] Initializing ... "));
    int state = radio.begin(frequency, 125, 8, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, 1.6, false);
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
}

void loop() {
    if (receivedFlag) {
        receivedFlag = false;

        String packetStr;
        int state = radio.readData(packetStr);

        if (state == RADIOLIB_ERR_NONE) 
        {
            Serial.println(F("[SX1262] Received packet!"));
            
            
            //String str = String(packetNum) + "," + String(latitude)  + "," + String(longitude) + "," + String(SIV) + "," + String(currentAltitude) + "," + String(flightState) + "," + String(batteryVoltage);
            // Split on commas
            int idx1 = packetStr.indexOf(',');
            int idx2 = packetStr.indexOf(',', idx1 + 1);
            int idx3 = packetStr.indexOf(',', idx2 + 1);
            int idx4 = packetStr.indexOf(',', idx3 + 1);
            int idx5 = packetStr.indexOf(',', idx4 + 1);
            int idx6 = packetStr.indexOf(',', idx5 + 1);

            // Check all commas found
            if (idx1 > 0 && idx2 > idx1 && idx3 > idx2 && idx4 > idx3 && idx5 > idx4 && idx6 > idx5) 
            {

                String packetNumStr = packetStr.substring(0, idx1);
                String latitudeStr = packetStr.substring(idx1 + 1, idx2);
                String longitudeStr = packetStr.substring(idx2 + 1, idx3);
                String satellitesStr = packetStr.substring(idx3 + 1, idx4);
                String altitudeStr = packetStr.substring(idx4 + 1, idx5);
                String flightStateStr = packetStr.substring(idx5 + 1, idx6);
                String batteryVoltageStr = packetStr.substring(idx6 + 1);

                // Convert to appropriate types
                uint32_t packetNum = packetNumStr.toInt();
                float latitude = latitudeStr.toFloat();      // can be negative
                float longitude = longitudeStr.toFloat();    // can be negative
                int satellites = satellitesStr.toInt();
                int altitude = altitudeStr.toInt();
                uint8_t flightState = (uint8_t)flightStateStr.toInt();
                float batteryVoltage = batteryVoltageStr.toFloat();

                Serial.print("Packet: "); Serial.print(packetNum);
                Serial.print(", Latitude: "); Serial.print(latitude, 6);
                Serial.print(", Longitude: "); Serial.print(longitude, 6);
                Serial.print(", Satellites: "); Serial.print(satellites);
                Serial.print(", Altitude: "); Serial.print(altitude);
                Serial.print(", Flight State: "); Serial.print(flightState);
                Serial.print(", Battery Voltage: "); Serial.println(batteryVoltage, 3);

            }
           // Log to SD card if enabled
            if (logFile) {
                logFile.print(millis());
                logFile.print(", ");
                logFile.print(packetNum);
                logFile.print(", ");
                logFile.print(latitude, 6);
                logFile.print(", ");
                logFile.print(longitude, 6);
                logFile.print(", ");
                logFile.print(satellites);
                logFile.print(", ");
                logFile.print(altitude);
                logFile.print(", ");
                logFile.print(flightState);
                logFile.print(", ");
                logFile.println(batteryVoltage, 3);
                logFile.flush();
            }
        } else {
            Serial.println("Error receiving data");
        }
    }
}

