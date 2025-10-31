all:
	@avr-gcc -Wall -g2 -Os -mmcu=atmega8 -o ./BIN/main.bin ./MCU/main.c
	@avr-objcopy -j .text -j .data -O ihex ./BIN/main.bin ./BIN/main.hex
	@rm ./BIN/main.bin
