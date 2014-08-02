#include "config.h"
#include <Sleep_n0m1.h>

#ifdef CONFIG_ONEWIRE
#include <OneWire.h>
#endif

#ifdef CONFIG_DHT
#include "dht.h"
#endif

#define MS_PER_HOUR    3.6e6

// ID of the settings block
#define CONFIG_VERSION "nc7"

// Tell it where to store your config data in EEPROM
#define CONFIG_START 32

//Node Configuration struct
typedef struct deviceInfo {
  char version[4]; //byte 0 - 3
  uint16_t NetworkChannel; //byte 4-5
  uint16_t NetworkNodeID; //byte 6-7
  boolean p0; //byte 8
  boolean p1; //byte 9
  uint8_t p0_debounce; //byte 10
  uint8_t p1_debounce; //byte 11
  boolean onewire; //byte 12
  uint8_t analog[8]; //byte 13-20
  uint8_t dht; //byte 21
  uint8_t digital[7]; //byte 22-28
  uint8_t leaf; // 29
} deviceInfo;

// Structure of our payload
struct payload_t
{
  char type;
  uint8_t sensor;
  uint8_t value_high;
  uint8_t value_low;
  uint8_t options;
};

#ifdef OTA-CONFIG
// OTA Config load payload
struct config_payload_t
{
  uint8_t pos;
  uint8_t data;
};
#endif

#ifdef WS2801
struct ws2801_payload_t
{
  uint8_t func;
  uint8_t l_led;
  uint8_t h_led;
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
#endif

struct output_payload_t
{
  uint8_t pin;
  uint8_t value;
};

#ifdef DEBUG
static int uart_putchar (char c, FILE *stream);
#endif
void loadConfig();
void saveConfig();
#ifdef RECEIVER
#ifdef OTA-CONFIG
void handle_ota(RF24NetworkHeader& header);
#endif
void handle_pin_output(RF24NetworkHeader& header);
void send_reply(uint16_t _dst, char _type, char _message);
#ifdef WS2801
void handle_ws2801(RF24NetworkHeader& header);
#endif

void receive_packet();

#endif
void send_packet(char _type, uint8_t _id, int16_t _value, uint8_t options);
void Pulse_0();
void Pulse_1();
#ifdef CONFIG_ONEWIRE
void get_onewire(void);
#endif
#ifdef CONFIG_DHT
void readDHTSensor(void);
#endif
#ifdef CONFIG_MENU
void show_info();
void set_devid();
void set_channel();
void set_p0();
void set_p1();
void set_p0_debounce();
void set_p1_debounce();
void set_analog();
#ifdef CONFIG_ONEWIRE
void set_1w();
#endif
#ifdef CONFIG_DHT
void set_dht();
#endif
void setupMenus();
#endif
void read_analog(int pin);
void setup(void);
void loop(void);
