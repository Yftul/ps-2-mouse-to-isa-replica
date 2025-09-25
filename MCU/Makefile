all:
	@avr-gcc -Wall -g -Os -mmcu=atmega8 -o ../BIN/main.bin main.c
	@avr-objcopy -j .text -j .data -O ihex ../BIN/main.bin ../BIN/main.hex
