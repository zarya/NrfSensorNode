//Debug
#define DEBUG

// Enable OTA Config (requires RECEIVER to be enabled)
#define OTA-CONFIG

// Set the ADC reference to the internal 1.1v referece (ONLY ATMEGA238 and 168)
//#define ADC_INTERNAL_REFERENCE

// Enable the config menu
#define CONFIG_MENU

// Enable DHT11/21/22 support
#define CONFIG_DHT

#define DHTPIN 3 
#define DHTTYPE DHT11
//#define DHTTYPE DHT21
//#define DHTTYPE DHT22

// Report vcc value
#define CONFIG_BATTERY

// Enable onewire temp support
#define CONFIG_ONEWIRE

// Enable WS2801 driver
//#define WS2801
#define WS2801_DATA 9 
#define WS2801_CLK 6
#define WS2801_LEDS 1 

//Receiver
#define RECEIVER

#define serial_baud_rate          115200 
#define serial_input_terminator   '\n'

#define NETWORK_MASTER 0

#ifdef DEBUG 
#define IF_DEBUG(x) ({x;})
#else
#define IF_DEBUG(x)
#endif


