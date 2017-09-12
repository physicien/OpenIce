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
#include "DS3231.h"
#include "SdFat.h"

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

RF24 radio(RF_CE_PIN,RF_CS_PIN);            //Configure the nrf24l01+ pins
RF24Network network(radio);
RF24Mesh mesh(radio,network);
RTClib rtc;
SdFat sd;
File dataFile;

uint32_t displayTimer = 0;
uint32_t counter = 0;

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
    Serial.print(F("\n\n"));
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
        DateTime now = rtc.now;
        switch(header.type) {
            // Display the incoming values from the sensor nodes
            case 'M' : network.read(header, &payload, sizeof(payload));
                nodeId = payload.nodeId;
                atmTmp = payload.atmTemperature;
                atmPress = payload.atmPressure;
                atmHum = payload.atmHumidity;
                iceTmp = payload.iceTemperature;

                Serial.print(F(" On slave:"));
                Serial.print(nodeId);
                Serial.println();
                Serial.print(F("Temperature: "));
                Serial.print(atmTmp);
                Serial.println(F(" degrees C"));
                Serial.print(F("Pressure: "));
                Serial.print(atmPress);
                Serial.println(F(" Pa"));
                Serial.print(F("%RH: "));
                Serial.print(atmHum);
                Serial.println(F(" %"));
                Serial.print(F("Trtd = "));
                Serial.print(iceTmp);
                Serial.println(F(" deg C"));
                Serial.println();

                data = {now.year(), now.month(), now.day(), now.hour(),
                    now.minute(), now.second(), nodeId, atmTmp,atmPress,
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
// SD and RTC
// -----------------------------------------------------------------------------
void rtcSetup() {
    Wire.begin();
}

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
        dataFile.print(data.y, DEC);
        dataFile.print(F("/"));
        dataFile.print(data.m, DEC);
        dataFile.print(F("/"));
        dataFile.print(data.d, DEC);
        dataFile.print(F("\t"));
        dataFile.print(data.hh, DEC);
        dataFile.print(F(":"));
        dataFile.print(data.mm, DEC);
        dataFile.print(F(":"));
        dataFile.print(data.ss, DEC);
        dataFile.print(F("\t"));
        dataFile.print(data.nodeId);
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
