#include "config.h"
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <OneWire.h>
#include "printf.h"
#include <EEPROM.h>

#if DHT == 11
#include <Dht11.h>
#endif
#if DHT == 21
#include <Dht21.h>
#endif
#if DHT == 22
#include <Dht22.h>
#endif

#ifdef CONFIG_MENU
#include <SerialUI.h>
#endif

#define serial_baud_rate           57600
#define serial_input_terminator   '\n'

#define MS_PER_HOUR    3.6e6

#define DEBOUNCE_P0 1500
#define DEBOUNCE_P1 1500

#define NETWORK_MASTER 0

// ID of the settings block
#define CONFIG_VERSION "nc4"

// Tell it where to store your config data in EEPROM
#define CONFIG_START 32

RF24 radio(8,7);

// Network uses that radio
RF24Network network(radio);

typedef struct deviceInfo {
  char version[4];
  uint16_t NetworkChannel;
  uint16_t NetworkNodeID;
  boolean p0;
  boolean p1;
  int p0_debounce;
  int p1_debounce;
  boolean onewire;
  int analog_pin_enable[8];
  int dht;
} 
deviceInfo;
//Config version, NetworkChannel, NodeID,p0,p1,p0db,p1db,1w,a0,a1,a2,a3,a4,a5,a6,a7,dht
deviceInfo NodeConfig = {CONFIG_VERSION,76,5,false,false,15,15,true,0,0,0,0,0,0,0,0,0};

unsigned long P0previous = 0;
unsigned long P0cycle = 0;

unsigned long P1previous = 0;
unsigned long P1cycle = 0;


OneWire  ds(5);

#ifdef CONFIG_MENU
//Define menu
SUI_DeclareString(device_greeting,"+++ Config");

SUI_DeclareString(settings_title, "Node Settings");
SUI_DeclareString(settings_key, "settings");
SUI_DeclareString(settings_help, "Perform setup and config");

SUI_DeclareString(settings_devid_key, "nodeid");
SUI_DeclareString(settings_devid_help, "Set node ID [int]");

SUI_DeclareString(settings_channel_key, "channel");
SUI_DeclareString(settings_channel_help, "Set dev channel [int]");

SUI_DeclareString(settings_p0_key, "p0");
SUI_DeclareString(settings_p0_help, "Pulse interrupt 0 [0/1]");

SUI_DeclareString(settings_p1_key, "p1");
SUI_DeclareString(settings_p1_help, "Pulse interrupt 1 [0/1]");

SUI_DeclareString(settings_p0_debounce_key, "p0debounce");
SUI_DeclareString(settings_p0_debounce_help, "Pulse interrupt 0 debounce [0-255] * 100");

SUI_DeclareString(settings_p1_debounce_key, "p1debounce");
SUI_DeclareString(settings_p1_debounce_help, "Pulse interrupt 1 debounce [0-255] * 100");

SUI_DeclareString(settings_1w_key, "1w");
SUI_DeclareString(settings_1w_help, "One Wire [0/1]");

#ifdef DHT
SUI_DeclareString(settings_dht_key, "dht");
SUI_DeclareString(settings_dht_help, "Set DHT pin 0 for disable");
#endif

SUI_DeclareString(settings_show_key, "show");

SUI::SerialUI mySUI = SUI::SerialUI(device_greeting);
#endif

// Structure of our payload
struct payload_t
{
  char type;
  uint8_t sensor;
  uint16_t value;
  
};

void setup(void)
{
#ifdef CONFIG_MENU
  mySUI.begin(serial_baud_rate);
  mySUI.setTimeout(20000);
  mySUI.setMaxIdleMs(30000);
  mySUI.setReadTerminator(serial_input_terminator);
  setupMenus();
#endif 
  loadConfig();
  SPI.begin();
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(7,7);
  network.begin(NodeConfig.NetworkChannel, NodeConfig.NetworkNodeID);
  
  if (NodeConfig.p0) {
    attachInterrupt(0, Pulse_0, RISING);
  }
  if (NodeConfig.p1) {
    attachInterrupt(1, Pulse_1, RISING);
  }
  
  Serial.println("Sensor node v2 starting up\n\r Press ? for config");
  delay(2000);
}

void Pulse_0() {
    unsigned long P0now = millis();
    unsigned long P0time = P0now - P0previous;
    
    if (P0time < NodeConfig.p0_debounce * 100) return;

    P0previous = P0now;
    Serial.println("P0 Pulse");
    payload_t payload = { 'P', (uint8_t) 0, (uint16_t) 1 };
    RF24NetworkHeader header(NETWORK_MASTER);
    bool ok = network.write(header,&payload,sizeof(payload));
    P1cycle++;
}

void Pulse_1() {
    unsigned long P1now = millis();
    unsigned long P1time = P1now - P1previous;
    
    if (P1time < NodeConfig.p1_debounce * 100) return;

    P1previous = P1now;

    Serial.println("P1 Pulse");
    payload_t payload = { 'P', (uint8_t) 1, (uint16_t) 1 };
    RF24NetworkHeader header(NETWORK_MASTER);
    bool ok = network.write(header,&payload,sizeof(payload));
    P1cycle++;
}

void loop(void)
{
#ifdef CONFIG_MENU
  if (mySUI.checkForUser(150))
  {
    mySUI.enter();
    while (mySUI.userPresent())
    {
      mySUI.handleRequests();
    }

  }
#endif
  network.update();
  if (NodeConfig.onewire) {
    get_onewire();
  }
  int i;
  for (i = 0; i < 8; i = i + 1) {
    if (NodeConfig.analog_pin_enable[i] > 0) {
        read_analog(i);
    }
  }
  //Read DHT
#ifdef DHT
  if (NodeConfig.dht > 0) {
#if DHT == 11
    static Dht11 dht(NodeConfig.dht);
#endif
#if DHT == 21
    static Dht21 dht(NodeConfig.dht);
#endif
#if DHT == 22
    static Dht22 dht(NodeConfig.dht);
#endif
    readDHTSensor(dht);
  }
#endif
  
  for (int x=0; x <= 500; x++){
    network.update();
    delay(10);
  }
}

void get_onewire(void)
{
  float celsius, fahrenheit;
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  
  while (ds.search(addr)) {
      if (OneWire::crc8(addr, 7) != addr[7]) {
          return;
      }
      
      // the first ROM byte indicates which chip
      switch (addr[0]) {
        case 0x10:
          type_s = 1;
          break;
        case 0x28:
          type_s = 0;
          break;
        case 0x22:
          type_s = 0;
          break;
        default:
          return;
      } 

      ds.reset();
      ds.select(addr);
      ds.write(0x44,1);
      for (int x=0; x <= 150; x++){
        network.update();
        delay(10);
      }
      
      present = ds.reset();
      ds.select(addr);    
      ds.write(0xBE);
      
      for ( i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = ds.read();
      }

      unsigned int raw = (data[1] << 8) | data[0];
      if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
          // count remain gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
      } else {
        byte cfg = (data[4] & 0x60);
        if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
        // default is 12 bit resolution, 750 ms conversion time
      }
      celsius = (float)raw / 16.0;
      Serial.print(addr[7]);
      Serial.print("  Temperature = ");
      Serial.println(celsius);
      
      float temperatuur = celsius * 100;
      payload_t payload = { 'T', (uint8_t) addr[7], (uint16_t) temperatuur }; 
      RF24NetworkHeader header(NETWORK_MASTER);
      bool ok = network.write(header,&payload,sizeof(payload));
      delay(20);
  }
  ds.reset_search();
}

#ifdef CONFIG_MENU
void show_info()
{
  SUI::Menu * current_menu = mySUI.currentMenu();
  mySUI.print("Device ID: ");
  mySUI.println(NodeConfig.NetworkNodeID);
  mySUI.print("Network channel: ");
  mySUI.println(NodeConfig.NetworkChannel);
  mySUI.print("Pulse interrupt 0 enabled: ");
  mySUI.println(NodeConfig.p0);
  mySUI.print("Pulse interrupt 1 enabled: ");
  mySUI.println(NodeConfig.p1);
  mySUI.print("Pulse interrupt 0 debounce: ");
  mySUI.println(NodeConfig.p0_debounce * 100);
  mySUI.print("Pulse interrupt 1 debounce: ");
  mySUI.println(NodeConfig.p1_debounce * 100);
  mySUI.print("OneWire enabled: ");
  mySUI.println(NodeConfig.onewire);
  mySUI.print("DHT pin: ");
  mySUI.println(NodeConfig.dht);
}

void set_devid()
{
  mySUI.showEnterNumericDataPrompt();
  int new_id = mySUI.parseInt();
  NodeConfig.NetworkNodeID = new_id;
  mySUI.println("Reboot the device to make the change available");
  mySUI.println(NodeConfig.NetworkNodeID, DEC);
  saveConfig();
  mySUI.returnOK();
}

void set_channel()
{
  mySUI.showEnterNumericDataPrompt();
  int new_channel = mySUI.parseInt();
  NodeConfig.NetworkChannel = new_channel;
  radio.setChannel(NodeConfig.NetworkChannel);
  mySUI.println(NodeConfig.NetworkChannel, DEC);
  saveConfig();
  mySUI.returnOK();
}

void set_p0()
{
  mySUI.showEnterNumericDataPrompt();
  boolean new_p0 = mySUI.parseInt();
  NodeConfig.p0 = new_p0;
  mySUI.println(NodeConfig.p0, DEC);
  mySUI.println("Reboot the device to make the change available");
  saveConfig();
  mySUI.returnOK();
}

void set_p1()
{
  mySUI.showEnterNumericDataPrompt();
  boolean new_p1 = mySUI.parseInt();
  NodeConfig.p1 = new_p1;
  mySUI.println(NodeConfig.p1, DEC);
  mySUI.println("Reboot the device to make the change available");
  saveConfig();
  mySUI.returnOK();
}

void set_p0_debounce()
{
  mySUI.showEnterNumericDataPrompt();
  int debounce = mySUI.parseInt();
  NodeConfig.p0_debounce = debounce;
  mySUI.println(NodeConfig.p0_debounce, DEC);
  saveConfig();
  mySUI.returnOK();
}

void set_p1_debounce()
{
  mySUI.showEnterNumericDataPrompt();
  int debounce = mySUI.parseInt();
  NodeConfig.p1_debounce = debounce;
  mySUI.println(NodeConfig.p1_debounce, DEC);
  saveConfig();
  mySUI.returnOK();
}

void set_1w()
{
  mySUI.showEnterNumericDataPrompt();
  boolean new_1w = mySUI.parseInt();
  NodeConfig.onewire = new_1w;
  mySUI.println(NodeConfig.onewire, DEC);
  saveConfig();
  mySUI.returnOK();
}

#ifdef DHT
void set_dht()
{
  mySUI.showEnterNumericDataPrompt();
  boolean new_dht = mySUI.parseInt();
  NodeConfig.dht = new_dht;
  mySUI.println(NodeConfig.dht, DEC);
  saveConfig();
  mySUI.returnOK();
}
#endif

void setupMenus()
{
  SUI::Menu * settingsMenu = mySUI.topLevelMenu();
  settingsMenu->setName(settings_title);
  settingsMenu->addCommand(settings_devid_key, set_devid, settings_devid_help);
  settingsMenu->addCommand(settings_channel_key, set_channel, settings_channel_help);
  settingsMenu->addCommand(settings_p0_key, set_p0, settings_p0_help);
  settingsMenu->addCommand(settings_p1_key, set_p1, settings_p1_help);
  settingsMenu->addCommand(settings_p0_debounce_key, set_p0_debounce, settings_p0_debounce_help);
  settingsMenu->addCommand(settings_p1_debounce_key, set_p1_debounce, settings_p1_debounce_help);
  settingsMenu->addCommand(settings_1w_key, set_1w, settings_1w_help);
#ifdef DHT
  settingsMenu->addCommand(settings_dht_key, set_dht, settings_dht_help);
#endif
  settingsMenu->addCommand(settings_show_key, show_info);
}
#endif

void loadConfig() {
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {
    for (unsigned int t=0; t<sizeof(NodeConfig); t++)
      *((char*)&NodeConfig + t) = EEPROM.read(CONFIG_START + t);
  }
  else {
    Serial.println("Config corrupted");
  }
}

void saveConfig() {
  for (unsigned int t=0; t<sizeof(NodeConfig); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&NodeConfig + t));
}

void read_analog(int pin) {
    int data = analogRead(pin);
    payload_t payload = { 'A', (uint8_t) pin, (uint16_t) data };
    RF24NetworkHeader header(NETWORK_MASTER);
    bool ok = network.write(header,&payload,sizeof(payload));
}

#ifdef DHT
void readDHTSensor(Dht& sensor) {
    if (sensor.read() == Dht::OK){ 
        Serial.print("Humidity (%): ");
        Serial.println(sensor.getHumidity());

        Serial.print("Temperature (C): ");
        Serial.println(sensor.getTemperature());

        float temp = sensor.getHumidity() * 100;
        payload_t payloadT = { 'T', (uint8_t) NodeConfig.dht, (uint16_t) temp };
        RF24NetworkHeader headerT(NETWORK_MASTER);
        network.write(headerT,&payloadT,sizeof(payloadT));

        float humd = sensor.getTemperature() * 100;
        payload_t payloadH = { 'H', (uint8_t) NodeConfig.dht, (uint16_t) humd };
        RF24NetworkHeader headerH(NETWORK_MASTER);
        network.write(headerH,&payloadH,sizeof(payloadH));
    }
}
#endif
