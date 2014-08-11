NrfSensorNode
=============

This is the software for the NRF24L01 Sensor node see https://gigafreak.net/stats/

* To use this install http://inotool.org/quickstart

# Features
* Serial config menu
* Onewire bus (DS18S20,DS18B20)
* DHT11,DHT21,DHT22,DHT44 Humidity and Temperature sensor
* HTU21D Humidity and Temperature sensor
* BMP085/180 pressure sensor support
* 2 x Interrupt driven pulse inputs
* 8 x Analog pin input
* OTA-Config
* PWM pin control
* IO pin control
* WS2801 ledstrip support

## Wishlist
* Serial camera support

# Structs
## Payload
| name | type | description |
-------|------|-------------|
| type | char | Packet type |
| sensor | uint8 | Sensor ID |
| value | float | Sensor value | 
 
### Packet types

| Type | Description |
-------|-------------|
| T | Temperature |
| H | Humidity |
| P | Pulse |
| A | Analog pin reading |
| P | Ping request |
| Q | Ping reply |
| C | Config data |
| O | Output pin data |
| D | Pressure |

## Config Payload
| name | type | description |
-------|------|-------------|
| pos  | uint8 | posision in the config struct |
| data | uint8 | value |

