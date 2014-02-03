NrfSensorNode
=============

This is the software for the NRF24L01 Sensor node see https://gigafreak.net/stats/

* To use this install http://inotool.org/quickstart

# Features
* Serial config menu
* Onewire bus (DS18S20,DS18B20)
* DHT11,DHT21,DHT22 Humidity and Temperature sensor
* 2 x Interrupt driven pulse inputs
* 8 x Analog pin input

## Wishlist
* PWM pin control
* IO pin control
* OTA-Config
* 


# Payload
| name | type | description |
-------|------|-------------|
| type | char | Packet type |
| sensor | uint8 | Sensor ID |
| value_high | uint 8 | Higher 8 bits of the int16 |
| value_low| uint8 | Lower 8 bits of the int16 |
| options | uint8 | Option char |
 
## Packet types

| Type | Description |
-------|-------------|
| T | Temperature |
| H | Humidity |
| P | Pulse |
| A | Analog pin reading |

## Packet options
| Bit | Description |
|-----|-------------|
| 1 | Negative value|
| 2 | Unused | 
| 3 | Unused |
| 4 | Unused |
| 5 | Unused |
| 6 | Unused |
| 7 | Unused |
| 8 | Unused |


