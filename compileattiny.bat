del main3.elf
del main3.o
del main3.hex
avr-gcc -mmcu=attiny2313 -std=gnu99 -Wall -Os -o main3.elf main3.c -w
avr-objcopy -j .text -j .data -O ihex main3.elf main3.hex
avr-size --mcu=attiny2313 --format=avr main3.elf
avrdude -c usbasp -p t2313 -U lfuse:w:0x64:m  -U flash:w:"main3.hex":a

