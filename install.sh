#!/bin/sh
git submodule init
git submodule update
rm -rf lib/RF24Network/tests
rm -rf lib/RF24Network/RPi
rm -rf lib/RF24Network/examples
rm -rf lib/RF24/examples_RPi
rm -rf lib/RF24/RPi
rm -rf lib/RF24/tests
rm -rf lib/RF24/examples
