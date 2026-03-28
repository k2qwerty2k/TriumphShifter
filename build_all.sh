#!/bin/sh

MCUS="atmega48 atmega48p atmega88 atmega88p atmega168 atmega168p atmega328 atmega328p"

rm -Rfv ./rel_8MHz
rm -Rfv ./rel_16MHz
mkdir -p ./rel_8MHz
mkdir -p ./rel_16MHz

for M in $MCUS ; do
	make clean
	USE_SERIAL=1 F_CPU=8000000 MCU="$M" make
	if [ $? -ne 0 ] ; then
		echo "ERROR ${M}"
		exit
	fi
	cp main.hex ./rel_8MHz/main_serial_${M}.hex

	make clean
	USE_SERIAL=0 F_CPU=8000000 MCU="$M" make
	if [ $? -ne 0 ] ; then
		echo "ERROR ${M}"
		exit
	fi
	cp main.hex ./rel_8MHz/main_${M}.hex
done

for M in $MCUS ; do
	make clean
	USE_SERIAL=1 F_CPU=16000000 MCU="$M" make
	if [ $? -ne 0 ] ; then
		echo "ERROR ${M}"
		exit
	fi
	cp main.hex ./rel_16MHz/main_serial_${M}.hex

	make clean
	USE_SERIAL=0 F_CPU=16000000 MCU="$M" make
	if [ $? -ne 0 ] ; then
		echo "ERROR ${M}"
		exit
	fi
	cp main.hex ./rel_16MHz/main_${M}.hex
done
