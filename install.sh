#!/bin/sh
git submodule init
git submodule update
rm -rf lib/RF24Network/tests
rm -rf lib/RF24Network/RPi
rm -rf lib/RF24Network/examples
rm -rf lib/RF24/RPi
rm -rf lib/RF24/tests
rm -rf lib/RF24/examples
mkdir temp
cd temp
git clone https://github.com/SloMusti/AS3935-Arduino-Library AS3935
cp -R AS3935/AS3935 ../lib
cd ..
rm -rf temp 
