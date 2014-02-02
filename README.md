NrfSensorNode
=============

This is the software for the NRF24L01 Sensor node see https://gigafreak.net/stats/

* To use this install http://inotool.org/quickstart

# Payload
| name | type | description |
-------|------|-------------|
| type | char | Packet type |
| sensor | uint8 | Sensor ID |
| value_high | uint 8 | Higher bits of the uint16 |
| value_low| uint8 | Lower 8 bits of the uint16 |
| options | uint8 | Option char |
 
# Packet types

| Type | Description |
-------|-------------|
| T | Temperature |
| H | Humidity |
| P | Pulse |
| A | Analog pin reading |

# Packet options
| Bit | Description |
|-----|-------------|
| 1 | Negative value|


