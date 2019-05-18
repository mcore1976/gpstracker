del main3b.elf
del main3b.o
del main3b.hex
avr-gcc -mmcu=attiny2313 -std=gnu99 -Wall -Os -o main3b.elf main3b.c -w
avr-objcopy -j .text -j .data -O ihex main3b.elf main3b.hex
avr-size --mcu=attiny2313 --format=avr main3b.elf
avrdude -c usbasp -p t2313 -U lfuse:w:0x64:m  -U flash:w:"main3b.hex":a

