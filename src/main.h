#include "config.h"

#ifdef CONFIG_ONEWIRE
#include <OneWire.h>
#endif

#if CONFIG_DHT == 11
#include <Dht11.h>
#endif
#if CONFIG_DHT == 21
#include <Dht21.h>
#endif
#if CONFIG_DHT == 22
#include <Dht22.h>
#endif


//Node Configuration struct
typedef struct deviceInfo {
  char version[4];
  uint16_t NetworkChannel;
  uint16_t NetworkNodeID;
  boolean p0;
  boolean p1;
  int p0_debounce;
  int p1_debounce;
  boolean onewire;
  int analog[8];
  int dht;
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

static int uart_putchar (char c, FILE *stream);
void send_packet(char _type, uint8_t _id, int16_t _value, uint8_t options);
void loadConfig();
void saveConfig();
void Pulse_0();
void Pulse_1();
void get_onewire(void);
void readDHTSensor(Dht& sensor);
void show_info();
void set_devid();
void set_channel();
void set_p0();
void set_p1();
void set_p0_debounce();
void set_p1_debounce();
void set_analog();
void set_1w();
void set_dht();
void setupMenus();
void read_analog(int pin);
void setup(void);
void loop(void);
