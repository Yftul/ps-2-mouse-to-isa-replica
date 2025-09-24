all:
	@avr-gcc -Wall -g -Os -mmcu=atmega8 -o main.bin main.c
	@avr-objcopy -j .text -j .data -O ihex main.bin main.hex
