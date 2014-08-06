#!/bin/sh
git submodule init
git submodule update
rm -rf lib/RF24Network/tests
rm -rf lib/RF24Network/RPi
rm -rf lib/RF24Network/examples
