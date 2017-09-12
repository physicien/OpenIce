#ifndef _DATA_STRUCTURES_h
#define _DATA_STRUCTURES_h

// Reading from BME280
typedef struct data_from_bme {
    float atmTemperature;
    float atmPressure;
    float atmHumidity;
} data_from_bme;

// Payload from MASTER
typedef struct payload_from_master {
    unsigned long counter;
} payload_from_master;

// Payload from SLAVE
typedef struct payload_from_slave {
    uint8_t nodeId;
    float atmTemperature;
    float atmPressure;
    float atmHumidity;
    double iceTemperature;
} payload_from_slave;

#endif
