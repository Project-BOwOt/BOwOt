#!/bin/bash

# ============================================================
# ATmega32 Build + Burn Script
#
# Usage:
#   ./burn.sh blinking.c
#
# This script:
#   1. Checks the source file
#   2. Compiles using avr-gcc
#   3. Creates .elf
#   4. Creates .hex
#   5. Checks USBasp/ATmega32 connection
#   6. Burns the HEX using AVRDUDE (without -B flag)
# ============================================================


# =========================
# CONFIGURATION
# =========================

MCU="atmega32"
F_CPU="8000000UL"

PROGRAMMER="usbasp"

# Source file is provided by user
SOURCE="$1"


# =========================
# COLORS
# =========================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'


# =========================
# CHECK ARGUMENT
# =========================

if [ -z "$SOURCE" ]; then
    echo -e "${RED}Error: No source file specified.${NC}"
    echo
    echo "Usage:"
    echo "  ./burn.sh main.c"
    exit 1
fi


# =========================
# CHECK SOURCE FILE
# =========================

if [ ! -f "$SOURCE" ]; then
    echo -e "${RED}Error: Source file '$SOURCE' not found.${NC}"
    exit 1
fi


# =========================
# CHECK AVR TOOLS
# =========================

for tool in avr-gcc avr-objcopy avrdude; do
    if ! command -v "$tool" &> /dev/null; then
        echo -e "${RED}Error: $tool not found.${NC}"
        exit 1
    fi
done


# =========================
# FILE NAMES
# =========================

BASENAME=$(basename "$SOURCE" .c)

ELF="${BASENAME}.elf"
HEX="${BASENAME}.hex"


echo
echo "============================================================"
echo "        ATmega32 BUILD + BURN"
echo "============================================================"
echo
echo "Source      : $SOURCE"
echo "MCU         : $MCU"
echo "CPU         : $F_CPU"
echo "Programmer  : $PROGRAMMER"
echo "ELF         : $ELF"
echo "HEX         : $HEX"
echo


# =========================
# COMPILE
# =========================

echo -e "${BLUE}[1/4] Compiling $SOURCE ...${NC}"

avr-gcc \
    -mmcu=$MCU \
    -DF_CPU=$F_CPU \
    -Os \
    -Wall \
    -Wextra \
    -o "$ELF" \
    "$SOURCE"

if [ $? -ne 0 ]; then
    echo
    echo -e "${RED}Compilation FAILED.${NC}"
    exit 1
fi

echo -e "${GREEN}Compilation successful.${NC}"
echo


# =========================
# CREATE HEX
# =========================

echo -e "${BLUE}[2/4] Creating HEX file ...${NC}"

avr-objcopy \
    -O ihex \
    -R .eeprom \
    "$ELF" \
    "$HEX"

if [ $? -ne 0 ]; then
    echo
    echo -e "${RED}HEX generation FAILED.${NC}"
    exit 1
fi

echo -e "${GREEN}HEX file created: $HEX${NC}"
echo


# =========================
# CHECK ATMEGA32
# =========================

echo -e "${BLUE}[3/4] Checking ATmega32 connection ...${NC}"
echo

avrdude \
    -c "$PROGRAMMER" \
    -p "$MCU"

if [ $? -ne 0 ]; then
    echo
    echo -e "${RED}ATmega32 connection FAILED.${NC}"
    echo
    echo "Check:"
    echo "  - USBasp connected"
    echo "  - USBasp attached to WSL"
    echo "  - J3 / JP3 Slow-SCK jumper set on USBasp"
    echo "  - ATmega32 powered"
    echo "  - ISP wiring"
    echo "  - RESET connection"
    exit 1
fi

echo
echo -e "${GREEN}ATmega32 detected successfully.${NC}"
echo


# =========================
# BURN HEX
# =========================

echo -e "${BLUE}[4/4] Burning $HEX to ATmega32 ...${NC}"
echo

avrdude \
    -c "$PROGRAMMER" \
    -p "$MCU" \
    -U "flash:w:$HEX:i"

if [ $? -ne 0 ]; then
    echo
    echo -e "${RED}Programming FAILED.${NC}"
    exit 1
fi


# =========================
# SUCCESS
# =========================

echo
echo "============================================================"
echo -e "${GREEN}          PROGRAMMING SUCCESSFUL!${NC}"
echo "============================================================"
echo
echo "Program : $SOURCE"
echo "HEX     : $HEX"
echo "MCU     : ATmega32"
echo
echo -e "${GREEN}Your program has been burned successfully.${NC}"
echo