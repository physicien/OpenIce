#ifndef _DATA_STRUCTURES_h
#define _DATA_STRUCTURES_h

// Datas for SD card
typedef struct data_to_sd {
    uint16_t y;
    uint8_t m;
    uint8_t d;
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
    uint8_t nodeId;
    float atmTemperature;
    float atmPressure;
    float atmHumidity;
    double iceTemperature;
} data_to_sd;

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
