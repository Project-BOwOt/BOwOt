#!/bin/bash
set -euo pipefail
rm -f main.elf main.hex
avr-gcc -mmcu=atmega32 -Os -std=gnu99 -Wall -Wextra -o main.elf main_all_sensors_fixed.c
avr-objcopy -O ihex -R .eeprom main.elf main.hex
avrdude -c usbasp -p m32 -U flash:w:main.hex:i
