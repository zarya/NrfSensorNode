#include <Arduino.h>
#include "printf.h"
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <EEPROM.h>
#include "main.h"
#include "config.h"

#ifndef CONFIG_MENU
#include <Arduino.h>
#endif

#ifdef CONFIG_MENU
#include <SerialUI.h>
#endif

#if defined(CONFIG_AS3935) || defined(BMP085) || defined(HTU21D)
#include <Wire.h>
#endif

#ifdef CONFIG_AS3935
#include <AS3935_Wire.h>
void printAS3935Registers();
void AS3935Irq();
volatile int AS3935IrqTriggered;
AS3935 AS3935(2,0);
#endif

#ifdef CONFIG_ONEWIRE
#include <OneWire.h>
//Enable onewire on pin 5
OneWire  ds(5);
#endif

//Load DHT module
#ifdef CONFIG_DHT
#include "dht.h"
dht DHT;
#endif

#ifdef HTU21D
#include <Adafruit_HTU21DF.h>
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
#endif

//load WS1801 module
#ifdef WS2801
#include <SPI.h>
#include <Adafruit_WS2801.h>
Adafruit_WS2801 strip = Adafruit_WS2801(WS2801_LEDS, WS2801_DATA, WS2801_CLK);
#endif

//Load BMP085/180 libs
#ifdef BMP085
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
#endif

//Initialize radio
RF24 radio(8,7);

// Network uses that radio
RF24Network network(radio);

//Load Sleep functions
Sleep sleep;

#ifdef DEBUG
static FILE uartout = {0} ;
static int uart_putchar (char c, FILE *stream)
{
    Serial.write(c) ;
    return 0 ;
}
#endif

//                                                                                  d             d
//                                                             1    a a a a a a a a h d d d d d d 1
//Config version, NetworkChannel, NodeID,p0,p1,p0db,p1db,      w,   0,1,2,3,4,5,6,7,t,2,3,4,5,6,9,0 leaf 
deviceInfo NodeConfig = {CONFIG_VERSION,80,5,false,false,15,15,true,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//   ~   ~ ~ ~ ~ 
//D2,3,4,5,6,9,10

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
SUI_DeclareString(settings_1w_key, "1w");
SUI_DeclareString(settings_1w_help, "1W [0/1]");
SUI_DeclareString(settings_dht_key, "dht");
SUI_DeclareString(settings_dht_help, "Set DHT D[2-4,6,9,10] 0 to disable");
SUI_DeclareString(settings_analog_key, "analog");
SUI_DeclareString(settings_analog_help, "Set A[0-7] 0 to disable");
SUI_DeclareString(settings_show_key, "show");
SUI_DeclareString(settings_clear_key, "clear");
SUI::SerialUI mySUI = SUI::SerialUI(device_greeting);
#endif

#ifdef CONFIG_AS3935
void printAS3935Registers()
{
    int noiseFloor = AS3935.getNoiseFloor();
    int spikeRejection = AS3935.getSpikeRejection();
    int watchdogThreshold = AS3935.getWatchdogThreshold();
    Serial.print("Noise floor is: ");
    Serial.println(noiseFloor,DEC);
    Serial.print("Spike rejection is: ");
    Serial.println(spikeRejection,DEC);
    Serial.print("Watchdog threshold is: ");
    Serial.println(watchdogThreshold,DEC);  
}

// this is irq handler for AS3935 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
void AS3935Irq()
{
    AS3935IrqTriggered = 1;
}
#endif


#ifdef RECEIVER
void send_reply(uint16_t _dst, char _type, char _message)
{
    RF24NetworkHeader header(_dst, _type);
    if (!network.write(header,&_message,sizeof(_message)))
        IF_DEBUG(printf_P(PSTR("*** WARNING *** Packet not send.\n\r")));
}

#ifdef OTA-CONFIG
void handle_ota(RF24NetworkHeader& header)
{
    config_payload_t config_payload;
    network.read(header,&config_payload,sizeof(config_payload));
    *((char*)&NodeConfig + config_payload.pos) = config_payload.data;
    IF_DEBUG(printf_P(PSTR("Setting POS: %i value: %i\n\r"),config_payload.pos,config_payload.data));
    saveConfig(); 
}
#endif

void handle_pin_output(RF24NetworkHeader& header)
{
    output_payload_t output_payload;
    network.read(header,&output_payload,sizeof(output_payload));
    IF_DEBUG(printf_P(PSTR("Setting pin: %i to: %i\n\r"),output_payload.pin,output_payload.value));
    if (output_payload.pin == 2 and NodeConfig.digital[0] == 1)
        digitalWrite(2,output_payload.value);
    else if (output_payload.pin == 3 and NodeConfig.digital[1] == 1)
        analogWrite(3,output_payload.value);
    else if (output_payload.pin == 4 and NodeConfig.digital[2] == 1)
        digitalWrite(4,output_payload.value);
    else if (output_payload.pin == 5 and NodeConfig.digital[3] == 1)
        analogWrite(5,output_payload.value);
    else if (output_payload.pin == 6 and NodeConfig.digital[4] == 1)
        analogWrite(6,output_payload.value);
    else if (output_payload.pin == 9 and NodeConfig.digital[5] == 1)
        analogWrite(9,output_payload.value);
    else if (output_payload.pin == 10 and NodeConfig.digital[6] == 1)
        analogWrite(10,output_payload.value);
}

#ifdef WS2801
void handle_ws2801(RF24NetworkHeader& header)
{
    ws2801_payload_t ws_payload;
    network.read(header,&ws_payload,sizeof(ws_payload));
    IF_DEBUG(printf_P(PSTR("WS2801: %i %i %i %i %i %i (%i)\n\r"),ws_payload.func,ws_payload.led_l,ws_payload.led_h,ws_payload.r,ws_payload.g,ws_payload.b,ws_payload.led_h,sizeof(header)));
    uint32_t c;
    c = ws_payload.r;
    c <<= 8;
    c |= ws_payload.g;
    c <<= 8;
    c |= ws_payload.b;
    strip.setPixelColor(ws_payload.led_l, c);
    if (ws_payload.led_h>0)
    {
        int i;
        for(i = ws_payload.led_l+1; i <= ws_payload.led_h; i++)
        {
            strip.setPixelColor(i, c);
        }
    }
    strip.show();
}
#endif

void receive_packet()
{
    while ( network.available() )
    {
        uint32_t timestamp_buffer;
        RF24NetworkHeader header;
        network.peek(header);

        switch (header.type)
        {
            case 'P':
                network.read(header,&timestamp_buffer,4);
                IF_DEBUG(printf_P(PSTR("Ping from %i TS: %i\n\r"),header.from_node,timestamp_buffer)); 
                send_reply(header.from_node,'Q',timestamp_buffer);
                break;
#ifdef OTA-CONFIG
            case 'C':
                handle_ota(header);
                network.read(header,0,0);
                break;
#endif
            case 'O':
                handle_pin_output(header);
                break;
#ifdef WS2801
            case 'W':
                handle_ws2801(header);
                break;
#endif
            default:
                network.read(header,0,0);
                IF_DEBUG(printf_P(PSTR("*** WARNING *** Unknown message type %c\n\r"),header.type));
                break;
        };
    }
}
#endif

//Send the packet over the NRF
void send_packet(char _type, uint8_t _id, float _value)
{
    payload_t payload = { _type, _id, _value };
    RF24NetworkHeader header(NETWORK_MASTER,'S');
    if (!network.write(header,&payload,sizeof(payload)))
        IF_DEBUG(printf_P(PSTR("*** WARNING *** Packet not send.\n\r")));
}

//P0 Interrupt function
void Pulse_0()
{
    unsigned long P0now = millis();
    unsigned long P0time = P0now - P0previous;
    
    if (P0time < NodeConfig.p0_debounce * 100) return;
    P0cycle++;
    P0previous = P0now;
}

//P1 Interrupt function
void Pulse_1()
{
    unsigned long P1now = millis();
    unsigned long P1time = P1now - P1previous;
    if (P1time < NodeConfig.p1_debounce * 100) return;
    P1cycle++;
    P1previous = P1now;
}


//Pulse function
void Pulse()
{
    if (P0cycle > 0)
    {
        IF_DEBUG(printf_P(PSTR("P0 Pulse\n\r")));
        send_packet('P', 0, P0cycle);
        P0cycle = 0;
    }
    if (P1cycle > 0)
    {
        IF_DEBUG(printf_P(PSTR("P1 Pulse\n\r")));
        send_packet('P', 1, P1cycle);
        P1cycle = 0;
    }
}

#ifdef CONFIG_ONEWIRE
//Onewire temp sensor function
void get_onewire(void)
{
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];
  
    while (ds.search(addr))
    {
        if (OneWire::crc8(addr, 7) != addr[7])
            return;
      
        // the first ROM byte indicates which chip
        switch (addr[0])
        {
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
        for (int x=0; x <= 80; x++)
        {
            network.update();
#ifdef RECEIVER
            receive_packet();
#endif
            delay(10);
        }
      
        present = ds.reset();
        ds.select(addr);    
        ds.write(0xBE);
      
        for ( i = 0; i < 9; i++)
            data[i] = ds.read();

        unsigned int raw = (data[1] << 8) | data[0];
        if (type_s)
        {
            raw = raw << 3;
            if (data[7] == 0x10)
                raw = (raw & 0xFFF0) + 12 - data[6];
        } 
        else 
        {
            byte cfg = (data[4] & 0x60);
            if (cfg == 0x00) 
                raw = raw << 3;  // 9 bit resolution, 93.75 ms
            else if (cfg == 0x20)
                raw = raw << 2; // 10 bit res, 187.5 ms
            else if (cfg == 0x40) 
                raw = raw << 1; // 11 bit res, 375 ms
            // default is 12 bit resolution, 750 ms conversion time
        }
        float celsius;
        celsius = 1;
        if(raw & 0x8000) {
            raw = (raw ^ 0xffff) + 1;
            celsius = -1;
        }
        celsius *= (float)raw / 16.0;
        IF_DEBUG(printf_P(PSTR("1W: %i T = %d\n\r"),addr[7], (int16_t)celsius));
        send_packet('T', (uint8_t) addr[7], celsius); 
        delay(20);
    }
    ds.reset_search();
}
#endif

#ifdef BMP085
//BMP085/180 sensor fuction
void readBMP()
{
    /* Get a new sensor event */ 
    sensors_event_t event;
    bmp.getEvent(&event);
    // Read sensor data
    if (event.pressure)
    {
        IF_DEBUG(printf_P(PSTR("BMP: %i\n\r"),(int16_t)event.pressure));
        send_packet('D', 0, event.pressure);
    } 
    else
    {
        IF_DEBUG(printf_P(PSTR("BMP: Error")));
    }
}
#endif

#ifdef CONFIG_DHT
//DHT Sensor function
void readDHTSensor()
{
#if DHTTYPE == 11
    int chk = DHT.read11(DHTPIN);
#endif
#if DHTTYPE == 21 
    int chk = DHT.read21(DHTPIN);
#endif
#if DHTTYPE == 22 
    int chk = DHT.read22(DHTPIN);
#endif
#if DHTTYPE == 44 
    int chk = DHT.read44(DHTPIN);
#endif

    if (chk == DHTLIB_ERROR_CHECKSUM or chk == DHTLIB_ERROR_TIMEOUT)
    {
        IF_DEBUG(printf_P(PSTR("DHT: Error\n\r")));
        return;
    }
    float h = (float)DHT.humidity; 
    float t = DHT.temperature;
    IF_DEBUG(printf_P(PSTR("DHT: H = %i T = %i\n\r"),(int16_t)h,(int16_t)t));
    send_packet('T', 0, t);
    delay(150);
    send_packet('H', 0, h);
}
#endif

#ifdef HTU21D 
//HTU21D Sensor function
void readHTU21DSensor()
{
    float h = (float)htu.readHumidity(); 
    float t = htu.readTemperature();
    IF_DEBUG(printf_P(PSTR("HTU21D: H = %i T = %i\n\r"),(int16_t)h,(int16_t)t));
    send_packet('T', 0, t);
    delay(150);
    send_packet('H', 0, h);
}
#endif

#ifdef CONFIG_BATTERY
// Function tnx to http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
long readVcc()
{
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
    #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif  

    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
    uint8_t high = ADCH; // unlocks both

    long result = (high<<8) | low;

    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result;
}
#endif

#ifdef CONFIG_MENU
//Factory default
void clear_info()
{
    for (unsigned int t=0; t<sizeof(NodeConfig); t++)
        EEPROM.write(CONFIG_START + t, 0);
    mySUI.println("Factory reset please reboot");
    show_info();
}

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
    for (i = 0; i < 8; i = i ++)
    {
        mySUI.print(NodeConfig.analog[i]);
    }
    mySUI.println("");
    mySUI.println("         23456910");
    mySUI.print("Digital: ");
    for (i = 0; i < 7; i = i ++)
    {
        mySUI.print(NodeConfig.digital[i]);
    }
    mySUI.println("");
    mySUI.print("Leaf node: ");
    mySUI.println(NodeConfig.leaf);
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
    if (pin < 8)
    {
        mySUI.println("\n[0-1] disable or enable");
        mySUI.showEnterNumericDataPrompt();
        NodeConfig.analog[pin] = mySUI.parseInt();
        mySUI.println(NodeConfig.analog[pin],DEC);
        saveConfig();
        mySUI.returnOK();
    }
    else
    {
        mySUI.returnError("invallid pin");
    }
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

void set_dht()
{
    mySUI.showEnterNumericDataPrompt();
    boolean new_dht = mySUI.parseInt();
    NodeConfig.dht = new_dht;
    mySUI.println(NodeConfig.dht, DEC);
    saveConfig();
    mySUI.returnOK();
}

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
    settingsMenu->addCommand(settings_dht_key, set_dht, settings_dht_help);
    settingsMenu->addCommand(settings_analog_key, set_analog, settings_analog_help);
    settingsMenu->addCommand(settings_show_key, show_info);
    settingsMenu->addCommand(settings_clear_key, clear_info);
}
#endif

//Load config from EEPROM
void loadConfig()
{
    if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {
        for (unsigned int t=0; t<sizeof(NodeConfig); t++)
            *((char*)&NodeConfig + t) = EEPROM.read(CONFIG_START + t);
    }
    else 
    {
        IF_DEBUG(printf_P(PSTR("*** WARNING *** Config corrupted %c\n\r")));
    }
}

//Save config to EEPROM
void saveConfig()
{
    for (unsigned int t=0; t<sizeof(NodeConfig); t++)
        EEPROM.write(CONFIG_START + t, *((char*)&NodeConfig + t));
}

//Read analog pin value 
void read_analog(int pin)
{
    int data = analogRead(pin);
    send_packet('A', (uint8_t) pin, (float)data);
    IF_DEBUG(printf_P(PSTR("Analog %i: %i\n\r"), pin,data));
}

//Setup function
void setup(void)
{
#ifdef CONFIG_MENU
    //Setup config menu
    mySUI.begin(serial_baud_rate);
    mySUI.setTimeout(20000);
    mySUI.setMaxIdleMs(30000);
    mySUI.setReadTerminator(serial_input_terminator);
    setupMenus();
#endif

//Do stuff to be able to enable debugging
#ifdef DEBUG
#ifndef CONFIG_MENU
    Serial.begin(serial_baud_rate);
#endif
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &uartout ;
#endif

#ifdef CONFIG_AS3935
    Wire.begin();
    AS3935.reset();
    if(!AS3935.calibrate())
        IF_DEBUG(printf_P(PSTR("AS3935 not detected")));
    AS3935.setIndoors();
    AS3935.enableDisturbers();
    printAS3935Registers();
    AS3935IrqTriggered = 0;
#endif

    //Startup the Node
    loadConfig(); //Load config from EEProm
    SPI.begin(); //Startup the SPI driver
    radio.begin(); //Startup the radio driver
    radio.setPALevel(RF24_PA_HIGH); //Set radio output to max
    IF_DEBUG(radio.printDetails()); //Print radio details
    network.begin(NodeConfig.NetworkChannel, NodeConfig.NetworkNodeID); //Start network layer

#ifdef ADC_INTERNAL_REFERENCE
    analogReference(INTERNAL);
#endif

#ifdef WS2801
    strip.begin();
    strip.show();
#endif

#ifdef BMP085
    //Initialize BMP085/180 
    if(!bmp.begin())
        IF_DEBUG(printf_P(PSTR("*** No BMP Detected")));
#endif

#ifdef HTU21D
    //Initialize HTU21D
    if (!htu.begin())
        IF_DEBUG(printf_P(PSTR("*** No HTU21D Detected")));
#endif

#ifdef RECEIVER
    if (NodeConfig.digital[0] == 1)
        pinMode(2,OUTPUT);
    else if (NodeConfig.digital[1] == 1)
        pinMode(3,OUTPUT);
    else if (NodeConfig.digital[2] == 1)
        pinMode(4,OUTPUT);
    else if (NodeConfig.digital[3] == 1)
        pinMode(5,OUTPUT);
    else if (NodeConfig.digital[4] == 1)
        pinMode(6,OUTPUT);
    else if (NodeConfig.digital[5] == 1)
        pinMode(9,OUTPUT);
    //else if (NodeConfig.digital[6] == 1)
    //  pinMode(10,OUTPUT);
#endif

#ifndef CONFIG_AS3935 
    if (NodeConfig.p0)
    {
        pinMode(2,OUTPUT);
        attachInterrupt(0, Pulse_0, RISING);
        digitalWrite(2,HIGH);
    }
#endif
#ifdef CONFIG_AS3935
    attachInterrupt(0,AS3935Irq,RISING);
#endif
    if (NodeConfig.p1)
    {
        pinMode(3,OUTPUT);
        attachInterrupt(1, Pulse_1, RISING);
        digitalWrite(3,HIGH);
    }

    IF_DEBUG(printf_P(PSTR("*** Bootup\n\r"))); 
    delay(2000);
}

void loop(void)
{
    //Send the interrupt pulses.
    Pulse();

#ifdef CONFIG_MENU
    //Start config menu
    if (mySUI.checkForUser(150))
    {
        mySUI.enter();
        while (mySUI.userPresent())
            mySUI.handleRequests();

    }
#endif
    network.update();

#ifdef RECEIVER
    receive_packet();
#endif

#ifdef CONFIG_ONEWIRE
    //Read onewire on D5
    if (NodeConfig.onewire)
        get_onewire();
#endif

    //Read the enabled analog pins
    int i;
    for (i = 0; i < 8; i = i + 1) 
    {
        if (NodeConfig.analog[i] > 0)
        {
            read_analog(i);
        }
    }

#ifdef CONFIG_DHT
    //Read DHT
    if (NodeConfig.dht > 0)
        readDHTSensor();
#endif

#ifdef HTU21D
    readHTU21DSensor();
#endif

#ifdef BMP085
    //Read the BMP085/180 sensor
    readBMP();
#endif

#ifdef CONFIG_BATTERY
    //Send vcc voltage
    delay(150);
    long vcc = readVcc();
    float result = (float)vcc / 1000;
    send_packet('B',0, result);
    IF_DEBUG(printf_P(PSTR("VCC: %d\n\r"), vcc));
    delay(150);
#endif

    if (NodeConfig.leaf == 0)
    {
        //Sleep for about 5sec.  
        for (int x=0; x <= 500; x++)
        {
            network.update();
#ifdef RECEIVER
            receive_packet();
#endif
            //Send interrupt pulses
            Pulse();
            delay(10);
        }
    }
    else
    {
        IF_DEBUG(printf_P(PSTR("Sleeping\n\r")));
        IF_DEBUG(delay(10));
        delay(500); // Give the radio time to transmit
        radio.powerDown();
        sleep.pwrDownMode();
        if (NodeConfig.leaf == 245)
        {
            sleep.sleepInterrupt(0,RISING);
            radio.powerUp();
            Pulse_0();
            delay(NodeConfig.p0_debounce * 100);
        }
        if(NodeConfig.leaf == 255)
        {
            sleep.sleepInterrupt(1,RISING);
            radio.powerUp();
            Pulse_1();
            delay(NodeConfig.p1_debounce * 100);
        }
        if(NodeConfig.leaf < 245)
        {
            unsigned long sleeptime = NodeConfig.leaf*1000;
            sleep.sleepDelay(sleeptime);
            radio.powerUp();
        }
    }
}
