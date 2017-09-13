/*------------------------------------------------------------------------------
OpenIce Ice Sensor

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
along with OpenIce. If not, see <http://www.gnu.org/licenses/>.

------------------------------------------------------------------------------*/

#include "settings.h"
#include "datastructures.h"
#include <stdint.h>
#include <SPI.h>
#include <Wire.h>
#include "JeeLib.h"
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include "PlayingWithFusion_MAX31865.h"
#include "PlayingWithFusion_MAX31865_STRUCT.h"
#include "SparkFunBME280.h"

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

ISR(WDT_vect) {Sleepy::watchdogEvent();}    //Setup the watchdog
RF24 radio(RF_CE_PIN,RF_CS_PIN);            //Configure the nrf24l01+ pins
RF24Network network(radio);
RF24Mesh mesh(radio, network);
BME280 bme;
PWFusion_MAX31865_RTD rtd_ch0(MAX_CS_PIN);
SPISettings settingsA(16000000, MSBFIRST, SPI_MODE3);

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------

void blink(byte pattern) {
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_R_PIN, OUTPUT);
    switch (pattern) {
        case 1:               // setup_pass
            for (byte i=0; i<3; i++) {
                if (i>0) delay(WAIT_TIME);
                digitalWrite(LED_G_PIN, HIGH);
                delay(NOTIFICATION_TIME);
                digitalWrite(LED_G_PIN, LOW);
            }
            break;
        case 2:               // bme_fault
            for (byte i=0; i<3; i++) {
                if (i>0) delay(WAIT_TIME);
                digitalWrite(LED_R_PIN, HIGH);
                if (i=1) {
                    delay(NOTIFICATION_TIME);
                }
                else {
                    delay(LONG_TIME);
                }
                digitalWrite(LED_R_PIN, LOW);
            }
            break; 
        case 3:               // max_fault
            for (byte i=0; i<2; i++) {
                if (i>0) delay(WAIT_TIME);
                digitalWrite(LED_R_PIN, HIGH);
                delay(NOTIFICATION_TIME);
                digitalWrite(LED_R_PIN, LOW);
            }
            break; 
        case 4:               // nrf_fault
                digitalWrite(LED_R_PIN, HIGH);
                delay(NOTIFICATION_TIME);
                digitalWrite(LED_R_PIN, LOW);
            break; 
        default:              // Success, data sent
            digitalWrite(LED_G_PIN, HIGH);
            delay(NOTIFICATION_TIME);
            digitalWrite(LED_G_PIN, LOW);
        break;
    }
}

void hardwareSetup() {
    Serial.begin(SERIAL_BAUD);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_R_PIN, OUTPUT);
    pinMode(MOSFET_PIN, OUTPUT);
    pinMode(RF_CS_PIN, OUTPUT);
    pinMode(MAX_CS_PIN, OUTPUT);
}

void initialPrint(){
    Serial.print(F("Sensor Module Started\n"));
    Serial.print(F("NodeID: "));
    Serial.println(NODEID);
    Serial.print(F("RTD probe: "));
    if (RTD_TYPE == 1) {
        Serial.print(F("PT100"));
    }
    else {
        Serial.print(F("PT1000"));
    }
    Serial.print(F("\n\n"));
}

// -----------------------------------------------------------------------------
// BME280
// -----------------------------------------------------------------------------

void bmeForceRead() {
    // We set the sensor in "forced mode" to force a reading.
    // After the reading the sensor will go back  to sleep mode.
    uint8_t value = bme.readRegister(BME280_CTRL_MEAS_REG);
    value = (value & 0xFC) + 0x01;
    bme.writeRegister(BME280_CTRL_MEAS_REG, value);

    // Measurement Time (as per BME280 datasheet section 9.1)
    // T_max(ms) = 1.25
    // + (2.3 * T_oversampling)
    // + (2.3 * P_oversampling + 0.575)
    // + (2.4 * H_oversampling + 0.575)
    // ~ 9.3ms for current settings
    delay(10);
}

void bmeSetup() {
    bme.settings.commInterface = I2C_MODE;
    bme.settings.I2CAddress = 0x77;
    bme.settings.runMode = 1;
    bme.settings.tStandby = 0;
    bme.settings.filter = 0;
    bme.settings.tempOverSample = 1;
    bme.settings.pressOverSample = 1;
    bme.settings.humidOverSample = 1;
    Serial.print(F("Starting BME280... result of .begin(): 0x"));
  
    //Calling .begin() causes the settings to be loaded
    delay(10);  // Make sure sensor had enough time to turn on. BME280 requires 
                // 2ms to start up.
    Serial.println(bme.begin(), HEX);
  
    Serial.print(F("Displaying ID, reset and ctrl regs\n"));
    
    Serial.print(F("ID(0xD0): 0x"));
    Serial.println(bme.readRegister(BME280_CHIP_ID_REG), HEX);
    Serial.print(F("Reset register(0xE0): 0x"));
    Serial.println(bme.readRegister(BME280_RST_REG), HEX);
    Serial.print(F("ctrl_meas(0xF4): 0x"));
    Serial.println(bme.readRegister(BME280_CTRL_MEAS_REG), HEX);
    Serial.print(F("ctrl_hum(0xF2): 0x"));
    Serial.println(bme.readRegister(BME280_CTRL_HUMIDITY_REG), HEX);
  
    Serial.print(F("\n\n"));
  
    Serial.print(F("Displaying all regs\n"));
    uint8_t memCounter = 0x80;
    uint8_t tempReadData;
    for(int rowi = 8; rowi < 16; rowi++ )
    {
    Serial.print(F("0x"));
    Serial.print(rowi, HEX);
    Serial.print(F("0:"));
    for(int coli = 0; coli < 16; coli++ )
    {
      tempReadData = bme.readRegister(memCounter);
      Serial.print((tempReadData >> 4) & 0x0F, HEX);//Print first hex nibble
      Serial.print(tempReadData & 0x0F, HEX);//Print second hex nibble
      Serial.print(F(" "));
      memCounter++;
    }
    Serial.print(F("\n"));
    }
    
    Serial.print(F("\n\n"));
    
    Serial.print(F("Displaying concatenated calibration words\n"));
    Serial.print(F("dig_T1, uint16: "));
    Serial.println(bme.calibration.dig_T1);
    Serial.print(F("dig_T2, int16: "));
    Serial.println(bme.calibration.dig_T2);
    Serial.print(F("dig_T3, int16: "));
    Serial.println(bme.calibration.dig_T3);
    
    Serial.print(F("dig_P1, uint16: "));
    Serial.println(bme.calibration.dig_P1);
    Serial.print(F("dig_P2, int16: "));
    Serial.println(bme.calibration.dig_P2);
    Serial.print(F("dig_P3, int16: "));
    Serial.println(bme.calibration.dig_P3);
    Serial.print(F("dig_P4, int16: "));
    Serial.println(bme.calibration.dig_P4);
    Serial.print(F("dig_P5, int16: "));
    Serial.println(bme.calibration.dig_P5);
    Serial.print(F("dig_P6, int16: "));
    Serial.println(bme.calibration.dig_P6);
    Serial.print(F("dig_P7, int16: "));
    Serial.println(bme.calibration.dig_P7);
    Serial.print(F("dig_P8, int16: "));
    Serial.println(bme.calibration.dig_P8);
    Serial.print(F("dig_P9, int16: "));
    Serial.println(bme.calibration.dig_P9);
  
    Serial.print(F("dig_H1, uint8: "));
    Serial.println(bme.calibration.dig_H1);
    Serial.print(F("dig_H2, int16: "));
    Serial.println(bme.calibration.dig_H2);
    Serial.print(F("dig_H3, uint8: "));
    Serial.println(bme.calibration.dig_H3);
    Serial.print(F("dig_H4, int16: "));
    Serial.println(bme.calibration.dig_H4);
    Serial.print(F("dig_H5, int16: "));
    Serial.println(bme.calibration.dig_H5);
    Serial.print(F("dig_H6, uint8: "));
    Serial.println(bme.calibration.dig_H6);

    Serial.println(F("\nCalibration done\n\n"));
}

struct data_from_bme bmeSensor() {
    float atmTmp;
    float atmPress;
    float atmHum;
    bmeForceRead();
    
    atmTmp = bme.readTempC();
    Serial.println();
    Serial.print(F("Temperature: "));
    Serial.print(atmTmp, 2);
    Serial.println(F(" degree C"));

    atmPress = bme.readFloatPressure();
    Serial.print(F("Pressure: "));
    Serial.print(atmPress, 2);
    Serial.println(F(" %"));

    atmHum = bme.readFloatHumidity();
    Serial.print(F("%RH: "));
    Serial.print(atmHum,2);
    Serial.println(F(" %"));
    
    struct data_from_bme data = {atmTmp, atmPress, atmHum};
    return data;
}   

// -----------------------------------------------------------------------------
// MAX31865
// -----------------------------------------------------------------------------

void rtdSetup() {
    Serial.print(F("Starting MAX31865..."));
    SPI.begin();
    SPI.setClockDivider(CLOCK_SPEED);
    SPI.setDataMode(SPI_MODE3);
    rtd_ch0.MAX31865_config();
    delay(100);             // give the sensor time to set up
    Serial.println(F("done\n\n"));
}

double rtdRead() {
    float R0;
    float A = 3.9083E-3;
    float B = -5.775E-7;
    double res;
    double tmp;

    analogWrite(MOSFET_PIN, MOSFET_V);
    SPI.beginTransaction(settingsA);
    digitalWrite(MAX_CS_PIN, LOW);
    rtd_ch0.MAX31865_config();      // New setup because MOSFET
    delay(100);                     // Give the sensor time to set up

    static struct var_max31865 RTD_CH0;
    RTD_CH0.RTD_type = RTD_TYPE;

    struct var_max31865 *rtd_ptr;
    rtd_ptr = &RTD_CH0;
    rtd_ch0.MAX31865_full_read(rtd_ptr);    // Update MAX31865 reading
    
    // ******************** Print RTD 0 Information ********************
    if(0 == RTD_CH0.status)             // No fault, print info to serial port
    {
      if(1 == RTD_CH0.RTD_type)         // Handle values for PT100
      {
        // Calculate RTD resistance
        res = (double)RTD_CH0.rtd_res_raw * 430 / 32768;
        Serial.print(F("Rrtd = "));     // Print RTD resistance heading
        Serial.print(tmp);              // Print RTD resistance
        R0 = 100;
      }
      else if(2 == RTD_CH0.RTD_type)    // Handle values for PT1000
      {
        // Calculate RTD resistance
        res = (double)RTD_CH0.rtd_res_raw * 4700 / 32768;
        Serial.print(F("Rrtd = "));     // Print RTD resistance heading
        Serial.print(tmp);              // Print RTD resistance
        R0 = 1000;
      }
      Serial.println(F(" ohm"));
      // Calculate RTD temperature (simple calc, +/- 2 deg C from -100C to 100C)
      // more accurate curve can be used outside that range
//      tmp = ((double)RTD_CH0.rtd_res_raw / 32) - 256;
      tmp = (-R0 * A +sqrt(R0 * R0 *A * A - 4 * R0 * B * (R0 - res))) / (2 * R0 * B);
      Serial.print(F("Trtd = "));       // Print RTD temperature heading
      Serial.print(tmp);                // Print RTD resistance
      Serial.println(F(" deg C"));      // Print RTD temperature heading
    }  // End of no-fault handling
    else 
    {
      blink(3);
      Serial.print(F("RTD Fault, register: "));
      Serial.print(RTD_CH0.status);
      if(0x80 & RTD_CH0.status)
      {
        Serial.println(F("RTD High Threshold Met"));// RTD high threshold fault
      }
      else if(0x40 & RTD_CH0.status)
      {
        Serial.println(F("RTD Low Threshold Met")); // RTD low threshold fault
      }
      else if(0x20 & RTD_CH0.status)
      {
        Serial.println(F("REFin- > 0.85 x Vbias")); // REFin- > 0.85 x Vbias
      }
      else if(0x10 & RTD_CH0.status)
      {
        Serial.println(F("FORCE- open"));// REFin- < 0.85 x Vbias, FORCE- open
      }
      else if(0x08 & RTD_CH0.status)
      {
        Serial.println(F("FORCE- open"));// RTDin- < 0.85 x Vbias, FORCE- open
      }
      else if(0x04 & RTD_CH0.status)
      {
        Serial.println(F("Over/Under voltage fault"));  
        // overvoltage/undervoltage fault
      }
      else
      {
        Serial.println(F("Unknown fault, check connection")); 
        // print RTD temperature heading
      }
    }  // end of fault handling
   
    digitalWrite(MAX_CS_PIN, HIGH);
    SPI.endTransaction();
    analogWrite(MOSFET_PIN, 0);
    return tmp;
}

// -----------------------------------------------------------------------------
// nRF23L01+
// -----------------------------------------------------------------------------

void radioSetup() {
    // Set the nodeID manually
    mesh.setNodeID(NODEID);
    // Connect to the mesh
    Serial.println(F("Connecting to the mesh..."));
    mesh.begin();
    radio.powerDown();
}

void radioSend(struct payload_from_slave payload) {
    radio.powerUp();
    mesh.update();
    // Send an 'M' type message containing the current millis()
    if (!mesh.write(&payload, 'M', sizeof(payload))) {
    // If a write fails, check connectivity to the mesh network
        if ( ! mesh.checkConnection() ) {
        // Refresh the network address
        Serial.println(F("Renewing Address"));
        mesh.renewAddress();
        } else {
        Serial.println(F("Send fail, Test OK"));
        blink(4);
        }
    } else {
        Serial.println(F("Send OK: "));
    }
    radio.powerDown();  
    blink(0);
}

void radioMeshUpdate() {
    radio.powerUp();
    while (network.available()) {
        RF24NetworkHeader header;
        payload_from_master payload;
        network.read(header, &payload, sizeof(payload));
        Serial.print(F("Received packet #"));
        Serial.print(payload.counter);
    radio.powerDown();  
    }
}

// -----------------------------------------------------------------------------
// Common methods 
// -----------------------------------------------------------------------------

void setup() {
    hardwareSetup();
    initialPrint();
    bmeSetup();
    rtdSetup();
    radioSetup();
    blink(1);
}

void loop() {
    double tmp;

    for (byte i=0; i<NB_TIME; i++)
        Sleepy::loseSomeTime(SLEEP_TIME);

    // Read data from the sensors
    data_from_bme bmeData = bmeSensor();
    tmp = rtdRead();

    // Send to the master node every one minutes
    payload_from_slave payload = {NODEID, bmeData.atmTemperature,
        bmeData.atmPressure, bmeData.atmHumidity, tmp};
    radioSend(payload);
    radioMeshUpdate();
}
