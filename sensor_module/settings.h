// ----------------------------------------------------------------------------
// Settings
// ----------------------------------------------------------------------------

// This is node ID, should be unique in the same network
#define NODEID              1

// Clock speed of the crystal
#define CLOCK_SPEED         SPI_CLOCK_DIV16

// MOSFET voltage (2V gate)
#define MOSFET_V            102     // 5V -> 102 and 3.3V -> 155

//RTD type
#define RTD_TYPE            2       // 1 for PT100, 2 for PT1000

// Define serial baudrate
#define SERIAL_BAUD         115200

// Various PIN definitions
#define LED_G_PIN           5
#define LED_R_PIN           6
#define MOSFET_PIN          7
#define RF_CE_PIN           8
#define RF_CS_PIN           9
#define MAX_CS_PIN          10

// Flash LED for thus amount of milliseconds after an event
#define NOTIFICATION_TIME   5
#define LONG_TIME           200
#define WAIT_TIME           200

// Sleep time between measurements
#define NB_TIME             1
#define SLEEP_TIME          60000       //msec range 0...65535
