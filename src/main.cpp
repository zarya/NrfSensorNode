#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <EEPROM.h>

#ifndef CONFIG_MENU
#include <Arduino.h>
#endif

#include "main.h"
#include "config.h"
#include "printf.h"

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

#ifdef CONFIG_MENU
#include <SerialUI.h>
#endif

RF24 radio(8,7);

// Network uses that radio
RF24Network network(radio);

#ifdef CONFIG_ONEWIRE
//Enable onewire on pin 5
OneWire  ds(5);
#endif

//Config version, NetworkChannel, NodeID,p0,p1,p0db,p1db,1w,a0,a1,a2,a3,a4,a5,a6,a7,dht
deviceInfo NodeConfig = {CONFIG_VERSION,80,5,false,false,15,15,true,0,0,0,0,0,0,0,0,0};

//Storage for Interrupt pins
unsigned long P0previous = 0;
unsigned long P0cycle = 0;
unsigned long P1previous = 0;
unsigned long P1cycle = 0;

#ifdef CONFIG_MENU
//Define menu
SUI_DeclareString(device_greeting,"+++ Config");
SUI_DeclareString(settings_title, "Node Settings");
SUI_DeclareString(settings_key, "settings");
SUI_DeclareString(settings_help, "");
SUI_DeclareString(settings_devid_key, "nodeid");
SUI_DeclareString(settings_devid_help, "Set node ID [int]");
SUI_DeclareString(settings_channel_key, "channel");
SUI_DeclareString(settings_channel_help, "Set dev channel [int]");
SUI_DeclareString(settings_p0_key, "p0");
SUI_DeclareString(settings_p0_help, "P0 [0/1]");
SUI_DeclareString(settings_p1_key, "p1");
SUI_DeclareString(settings_p1_help, "P1 [0/1]");
SUI_DeclareString(settings_p0_debounce_key, "p0d");
SUI_DeclareString(settings_p0_debounce_help, "P0 debounce [0-255] * 100");
SUI_DeclareString(settings_p1_debounce_key, "p1d");
SUI_DeclareString(settings_p1_debounce_help, "P1 debounce [0-255] * 100");

#ifdef CONFIG_ONEWIRE
SUI_DeclareString(settings_1w_key, "1w");
SUI_DeclareString(settings_1w_help, "1W [0/1]");
#endif

#ifdef CONFIG_DHT
SUI_DeclareString(settings_dht_key, "dht");
SUI_DeclareString(settings_dht_help, "Set DHT D[2-4,6,9,10] 0 to disable");
#endif

SUI_DeclareString(settings_analog_key, "analog");
SUI_DeclareString(settings_analog_help, "Set A[0-7] 0 to disable");

SUI_DeclareString(settings_show_key, "show");

SUI::SerialUI mySUI = SUI::SerialUI(device_greeting);
#endif

#ifdef DEBUG
static FILE uartout = {0} ;
static int uart_putchar (char c, FILE *stream)
{
    Serial.write(c) ;
    return 0 ;
}
#endif

#ifdef RECEIVER
void send_reply(uint16_t _dst, char _type, char _message)
{
    RF24NetworkHeader header(_dst, _type);
    network.write(header,&_message,sizeof(_message));
}
#ifdef OTA-CONFIG
void handle_ota(RF24NetworkHeader& header)
{
    config_payload_t config_payload;
    network.read(header,&config_payload,sizeof(config_payload));
    *((char*)&NodeConfig + config_payload.pos) = config_payload.data;
    Serial.print("Setting POS: ");
    Serial.print(config_payload.pos,DEC);
    Serial.print(" Value: ");
    Serial.println(config_payload.data,DEC); 
}
#endif

void receive_packet() {
    uint32_t timestamp_buffer;
    while ( network.available() ) {
        RF24NetworkHeader header;
        network.peek(header);

        switch (header.type)
        {
            case 'P':
                network.read(header,&timestamp_buffer,4);
                Serial.print("Ping from ");
                Serial.print(timestamp_buffer); 
                Serial.println(header.from_node);
                send_reply(header.from_node,'Q',timestamp_buffer);
                break;
#ifdef OTA-CONFIG
            case 'C':
                handle_ota(header);
                network.read(header,0,0);
                break;
#endif
            default:
                network.read(header,0,0);
                printf_P(PSTR("*** WARNING *** Unknown message type %c\n\r"),header.type);
                break;
        };
    }
}
#endif

//Send the packet over the NRF
void send_packet(char _type, uint8_t _id, int16_t _value, uint8_t options) {
    uint8_t value_low;
    uint8_t value_high;
    value_low = (_value & 0xff); 
    value_high = (_value >> 4) & 0xff;
    payload_t payload = { _type, _id, value_high, value_low, options };
    RF24NetworkHeader header(NETWORK_MASTER,'S');
    network.write(header,&payload,sizeof(payload));
}

//P0 Interrupt function
void Pulse_0() {
    unsigned long P0now = millis();
    unsigned long P0time = P0now - P0previous;
    
    if (P0time < NodeConfig.p0_debounce * 100) return;

    P0previous = P0now;
    Serial.println("P0 Pulse");
    send_packet('P', 0, 1, 0);
    P1cycle++;
}

//P1 Interrupt function
void Pulse_1() {
    unsigned long P1now = millis();
    unsigned long P1time = P1now - P1previous;
    
    if (P1time < NodeConfig.p1_debounce * 100) return;

    P1previous = P1now;

    Serial.println("P1 Pulse");
    send_packet('P', 1, 1, 0);
    P1cycle++;
}

#ifdef CONFIG_ONEWIRE
//Onewire temp sensor function
void get_onewire(void)
{
  float celsius;
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

      //Delay 800ms wile doing network updates
      for (int x=0; x <= 80; x++){
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
      int negative = 0;
      if ( raw & 0x8000 )  {
        raw ^= 0xffff;
        raw++;
        negative = 1;
      }
      celsius = (float)raw / 17.0;
      Serial.print(addr[7]);
      Serial.print(" T = ");
      if (negative) Serial.print("-");
      Serial.println(celsius);
      
      float temperatuur = celsius * 100;
      send_packet('T', (uint8_t) addr[7], (int16_t) temperatuur,negative); 
      delay(20);
  }
  ds.reset_search();
}
#endif

#ifdef CONFIG_DHT
//DHT Sensor function
void readDHTSensor(Dht& sensor) {
    if (sensor.read() == Dht::OK){ 
        Serial.print("H = ");
        Serial.println(sensor.getHumidity());

        Serial.print("T = ");
        Serial.println(sensor.getTemperature());

        float temp = sensor.getTemperature() * 100;
        send_packet('T', (uint8_t) NodeConfig.dht, (int16_t) temp, 0);

        float humd = sensor.getHumidity() * 100;
        send_packet('H', (uint8_t) NodeConfig.dht, (int16_t) humd, 0);
    }
}
#endif

#ifdef CONFIG_MENU
void show_info()
{
  SUI::Menu * current_menu = mySUI.currentMenu();
  mySUI.print("Device ID: ");
  mySUI.println(NodeConfig.NetworkNodeID);
  mySUI.print("Channel: ");
  mySUI.println(NodeConfig.NetworkChannel);
  mySUI.print("P0 enabled: ");
  mySUI.println(NodeConfig.p0);
  mySUI.print("P1 enabled: ");
  mySUI.println(NodeConfig.p1);
  mySUI.print("P0 debounce: ");
  mySUI.println(NodeConfig.p0_debounce * 100);
  mySUI.print("P1 debounce: ");
  mySUI.println(NodeConfig.p1_debounce * 100);
  mySUI.print("OneWire enabled: ");
  mySUI.println(NodeConfig.onewire);
  mySUI.print("DHT pin: ");
  mySUI.println(NodeConfig.dht);
  mySUI.println("        01234567");
  mySUI.print("Analog: ");
  int i;
  for (i = 0; i < 8; i = i ++) {
    mySUI.print(NodeConfig.analog[i]);
  }
  mySUI.println("");
}

void set_devid()
{
  mySUI.showEnterNumericDataPrompt();
  int new_id = mySUI.parseInt();
  NodeConfig.NetworkNodeID = new_id;
  mySUI.println("Reboot needed");
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
  mySUI.println("Reboot needed");
  saveConfig();
  mySUI.returnOK();
}

void set_p1()
{
  mySUI.showEnterNumericDataPrompt();
  boolean new_p1 = mySUI.parseInt();
  NodeConfig.p1 = new_p1;
  mySUI.println(NodeConfig.p1, DEC);
  mySUI.println("Reboot needed");
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

void set_analog()
{
  mySUI.print("Pin:");
  mySUI.showEnterNumericDataPrompt();
  int pin = mySUI.parseInt();
  if (pin < 8) {
    mySUI.println("\n[0-1] disable or enable");
    mySUI.showEnterNumericDataPrompt();
    NodeConfig.analog[pin] = mySUI.parseInt();
    mySUI.println(NodeConfig.analog[pin],DEC);
    saveConfig();
    mySUI.returnOK();
  } else {
    mySUI.returnError("invallid pin");
  }
}

#ifdef CONFIG_ONEWIRE
void set_1w()
{
  mySUI.showEnterNumericDataPrompt();
  boolean new_1w = mySUI.parseInt();
  NodeConfig.onewire = new_1w;
  mySUI.println(NodeConfig.onewire, DEC);
  saveConfig();
  mySUI.returnOK();
}
#endif

#ifdef CONFIG_DHT
void set_dht()
{
  mySUI.showEnterNumericDataPrompt();
  boolean new_dht = mySUI.parseInt();
  if (new_dht == 0 or new_dht == 2 or new_dht == 3 or new_dht == 4 or new_dht == 6 or new_dht == 9 or new_dht == 10){
      NodeConfig.dht = new_dht;
    mySUI.println(NodeConfig.dht, DEC);
    saveConfig();
    mySUI.returnOK();
  }else mySUI.returnError("invallid pin");
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
#ifdef CONFIG_ONEWIRE
  settingsMenu->addCommand(settings_1w_key, set_1w, settings_1w_help);
#endif
#ifdef CONFIG_DHT
  settingsMenu->addCommand(settings_dht_key, set_dht, settings_dht_help);
#endif
  settingsMenu->addCommand(settings_analog_key, set_analog, settings_analog_help);
  settingsMenu->addCommand(settings_show_key, show_info);
}
#endif

//Load config from EEPROM
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

//Save config to EEPROM
void saveConfig() {
  for (unsigned int t=0; t<sizeof(NodeConfig); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&NodeConfig + t));
}

//Read analog pin value 
void read_analog(int pin) {
    int data = analogRead(pin);
    send_packet('A', (uint8_t) pin, (int16_t) data, 0);
}

//Setup frunction
void setup(void)
{
#ifdef CONFIG_MENU
  mySUI.begin(serial_baud_rate);
  mySUI.setTimeout(20000);
  mySUI.setMaxIdleMs(30000);
  mySUI.setReadTerminator(serial_input_terminator);
  setupMenus();
#endif
#ifdef DEBUG
#ifndef CONFIG_MENU
  Serial.begin(serial_baud_rate);
#endif
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

#ifdef DEBUG
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout ;
#endif

  Serial.println("Bootup");
  delay(2000);
}

void loop(void)
{
#ifdef CONFIG_MENU
  //Start config menu
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

#ifdef RECEIVER
  receive_packet();
#endif

  //Read onewire on D5
#ifdef CONFIG_ONEWIRE
  if (NodeConfig.onewire) {
    get_onewire();
  }
#endif

  //Read the enabled analog pins
  int i;
  for (i = 0; i < 8; i = i + 1) {
    if (NodeConfig.analog[i] > 0) {
        read_analog(i);
    }
  }

  //Read DHT
#ifdef CONFIG_DHT
  if (NodeConfig.dht > 0) {
#if CONFIG_DHT == 11
    static Dht11 dht(NodeConfig.dht);
#endif
#if CONFIG_DHT == 21
    static Dht21 dht(NodeConfig.dht);
#endif
#if CONFIG_DHT == 22
    static Dht22 dht(NodeConfig.dht);
#endif
    readDHTSensor(dht);
  }
#endif

  //Sleep for about 5sec.  
  for (int x=0; x <= 500; x++){
    network.update();
    delay(10);
  }
}
