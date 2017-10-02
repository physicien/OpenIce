/*------------------------------------------------------------------------------
OpenIce Datalogger

File Name: sensor_module.ino
Processor/Platform: ATMega328P (tested)
Development Environment: Arduino 1.8.4

Copyright (C) 2017 by Emmanuel Bourret
<emmanuel dot bourret at umontreal dot ca>

This file is part of OpenIce. 

OpenIce is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

OpenIce is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with OpenIce.  If not, see <http://www.gnu.org/licenses/>.

------------------------------------------------------------------------------*/

#include "settings.h"
#include "datastructures.h"
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include "RtcDS3231.h"
#include "SdFat.h"

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

RF24 radio(RF_CE_PIN,RF_CS_PIN);            //Configure the nrf24l01+ pins
RF24Network network(radio);
RF24Mesh mesh(radio,network);
RtcDS3231<TwoWire> rtc(Wire);
SdFat sd;
File dataFile;

uint32_t displayTimer = 0;
uint32_t counter = 0;

#define countof(a) (sizeof(a) / sizeof(a[0]))

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------

void hardwareSetup() {
    Serial.begin(SERIAL_BAUD);
    pinMode(RF_CS_PIN, OUTPUT);
    pinMode(SD_CS_PIN, OUTPUT);
}

void initialPrint() {
    Serial.print(F("Datalogger Module Started\n"));
    Serial.print(F("\n"));
}

// -----------------------------------------------------------------------------
// nRF24L01+
// -----------------------------------------------------------------------------

void radioSetup() {
    // Set the nodeID to 0 for the master node
    mesh.setNodeID(NODEID);
    Serial.println(F("Initialization of the mesh..."));
    // Connect to the mesh
    mesh.begin();
    Serial.print(F("Node "));
    Serial.print(mesh.getNodeID());
    Serial.print(F(" Online"));
    Serial.print(F("\n\n"));
}

void radioReceive() {
    uint8_t nodeId;
    float atmTmp;
    float atmPress;
    float atmHum;
    double iceTmp;
    struct data_to_sd data;

    // Call mesh.update to keep the network updated
    mesh.update();

    // In addition, keep the 'DHCP service' running on the master node so
    // addresses will be assigned to the sensor nodes
    mesh.DHCP();

    // Check for incoming data from the sensors
    if(network.available()) {
        RF24NetworkHeader header;
        network.peek(header);
        payload_from_slave payload;
        RtcDateTime now = rtc.GetDateTime();
        switch(header.type) {
            // Display the incoming values from the sensor nodes
            case 'M' : network.read(header, &payload, sizeof(payload));
                nodeId = payload.nodeId;
                atmTmp = payload.atmTemperature;
                atmPress = payload.atmPressure;
                atmHum = payload.atmHumidity;
                iceTmp = payload.iceTemperature;

                Serial.print(F(" On slave:"));
                Serial.print(nodeId,DEC);
                Serial.println();
                Serial.print(F("Temperature: "));
                Serial.print(atmTmp,2);
                Serial.println(F(" deg C"));
                Serial.print(F("Pressure: "));
                Serial.print(atmPress,2);
                Serial.println(F(" Pa"));
                Serial.print(F("%RH: "));
                Serial.print(atmHum,2);
                Serial.println(F(" %"));
                Serial.print(F("Trtd: "));
                Serial.print(iceTmp,3);
                Serial.println(F(" deg C"));
                Serial.println();

                data = {now.Year(), now.Month(), now.Day(), now.Hour(),
                    now.Minute(), now.Second(), nodeId, atmTmp,atmPress,
                    atmHum, iceTmp};
                    
                sdWrite(data);
                break;
            default: network.read(header,0,0);
                Serial.println(header.type);
                break;
        }
    }
}

void radioMeshUpdate() {
    // Meanwhile, every UP_TIME ms...
    if(millis() - displayTimer > UP_TIME) {
        // Show DHCP table
        displayTimer = millis();
        Serial.println(" ");
        Serial.println(F("********Assigned Addresses********"));
        for(int i=0; i<mesh.addrListTop; i++) {
            Serial.print(F("NodeID: "));
            Serial.print(mesh.addrList[i].nodeID);
            Serial.print(F(" RF24Network Address: 0"));
            Serial.println(mesh.addrList[i].address,OCT);
        }
        Serial.println(F("**********************************"));

        // Send same master message to all slaves
        for(int i=0; i<mesh.addrListTop; i++) {
            counter++;
            payload_from_master payload = {counter};//, showLed};
            RF24NetworkHeader header(mesh.addrList[i].address, OCT);
            int x = network.write(header, &payload, sizeof(payload));
        }
    }
}

// -----------------------------------------------------------------------------
// RTC
// -----------------------------------------------------------------------------
void rtcSetup() {
    Wire.begin();
    Serial.print(F("Compiled: "));
    Serial.print(__DATE__);
    Serial.println(__TIME__);

    rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    printDateTime(compiled);
    Serial.println();

    if (!rtc.IsDateTimeValid()) 
    {
        // Common Cuases:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing

        Serial.println(F("RTC lost confidence in the DateTime!"));

        // following line sets the RTC to the date & time this sketch was
        // compiled it will also reset the valid flag internally unless
        // the Rtc device is having an issue

        rtc.SetDateTime(compiled);
    }

    if (!rtc.GetIsRunning())
    {
        Serial.println(F("RTC was not actively running, starting now"));
        rtc.SetIsRunning(true);
    }

    RtcDateTime now = rtc.GetDateTime();
    if (now < compiled) 
    {
        Serial.print(F("RTC is older than compile time!  (Updating DateTime)"));
        Serial.print(F("\n"));
        rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        Serial.println(F("RTC is newer than compile time. (this is expected)"));
    }
    else if (now == compiled) 
    {
        Serial.print(F("RTC is the same as compile time! "));
        Serial.println(F("(not expected but all is fine)"));
    }

    // never assume the Rtc was last configured by you, so
    // just clear them to your needed state
    rtc.Enable32kHzPin(false);
    rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 
}

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
}

// -----------------------------------------------------------------------------
// SD
// -----------------------------------------------------------------------------

void sdSetup() {
    Serial.print(F("Initializing SD card..."));
    if (!sd.begin(SD_CS_PIN)) {
        Serial.println(F("initialization failed!"));
        return;
    }
    Serial.println(F("initialization done."));
}

void sdWrite(struct data_to_sd data) {
    dataFile = sd.open("data.txt", FILE_WRITE);
    // If the file opened okay, write to it
    if(dataFile) {
        Serial.print(F("Writing to data.txt..."));
        char date[20];
        snprintf_P(date, 
            countof(date),
            PSTR("%04u/%02u/%02u\t%02u:%02u:%02u"),
            data.y,
            data.m,
            data.d,
            data.hh,
            data.mm,
            data.ss );
        dataFile.print(date);
        dataFile.print(F("\t"));
        dataFile.print(data.nodeId,DEC);
        dataFile.print(F("\t"));
        dataFile.print(data.atmTemperature,2);
        dataFile.print(F("\t"));
        dataFile.print(data.atmPressure,2);   
        dataFile.print(F("\t"));
        dataFile.print(data.atmHumidity,2);   
        dataFile.print(F("\t"));
        dataFile.println(data.iceTemperature,3);
        // Close the file
        dataFile.close();
        Serial.println(F("done."));
    }
    else {
        // If the file didn't open, print an error
        Serial.println(F("error opening data.txt"));
    }
}

// -----------------------------------------------------------------------------
// Common methods
// -----------------------------------------------------------------------------

void setup() {
    hardwareSetup();
    initialPrint();
    rtcSetup();
    sdSetup();
    radioSetup();
}

void loop() {
    radioReceive();
    radioMeshUpdate();
}
