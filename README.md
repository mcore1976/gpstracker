 DIY ultra cheap GPS bike/car tracker based on ATMEL ATTINY 2313/4313 or ATMEGA 328P (arduino uno chip) and SIM800L module from China (SIM80X/90X depending on your location and frequency used). 

The device when called by mobile phone polls cell-id info from nearest 2G cell, uses GPRS to query Google servers for GPS location of that cell and sends back text message to your phone with current location link to Google map with timestamp. 

The part list is (with the cost as in 2018): 

- SIM800L (3.5 USD on Aliexpress, but notice this module DOES NOT have GPS so we are polling info from base stations - if you want real GPS you need to use SIM808 breadboard and change the code of MCU) 
- ATMEL ATTINY2313 (2USD)  or ATMEGA 328P (arduino uno)
- LM7805 (0.5USD) - optional if you intend to connect to car/bike battery directly
- 6x 1N4007 (0.3 USD) - optional - for CAR/BIKE battery or USB Powerbank
- 2x 1000uF / 16V capacitor ( 0.5 USD) - when powered from 3xAA battery pack only 1 capacitor is needed 
- 100nF capacitor (0.2 USD) 
- universal PCB, pins & connector (2 USD) 

To upload program code to the chip using cheapest USBASP programmer (less than 2 USD on eBay/Aliexpress) look at this page :
http://www.learningaboutelectronics.com/Articles/Program-AVR-chip-using-a-USBASP-with-10-pin-cable.php

The script attached in repository  ( "compileatmega" or "compileattiny" ) can be used to upload data to the chip if you have Linux machine with following packages : "avr-gcc" and "avrdude". For example in Ubuntu download these packages using command : "sudo apt-get install avr-gcc"  and "sudo apt-get install avrdude"  and you are ready to go. 


The code is written in avr-gcc and was uploaded via USBASP. Both binary output versions are provided : for ATTINY 2313(2313A/2313V) and ATMEGA 328P.
For smallest chip ATTINY2313 the code takes about 2KB of Flash memory so the chip memory is gets full.
Considering the SIM800L capability if more Flash memory is available ( ATTINY 4313 with 4KB or ATMEGA328P) the chip could even upload the GPS data to some EMAIL/FTP/HTTP server to get car tracking history. 

The solution has ultra low power consumption because it is utilizing SLEEP MODE on SIM8XX/9XX module and POWER DOWN feature on ATTINY/ATMEGA MCU (current in standby is below 2mA ) and connects to GPRS/polls GPS only upon request. Also the LED on the SIM800L is switched off to further reduce current consumption.
This will give you something like at least 1 month of work time on smallest USB powerbanks like 2000mAh or 3xAA battery ( I personally do recommed to connect to the 3xAA because powerbanks have LED and converters that drain extra current). 
The ATTINY/ATMEGA wakes up when SIM8xx PIN RING goes to GND (in case of incoming call or SMS or MODULE RESTART - "Call Ready" message from URC). 

When SIM8xx module sends SMS/GPRS data it it may draw a lot of current ( up to 2A ) in short peaks so it is crucial to use good cables and thick copper lines for GND and VCC on PCB. This is the main issue people face when dealing with SIM8xx/9xx modules. The voltage may additionaly drop during this situation so that is why such big capacitor is in use. 
In the design there are 1N4007 diodes connected in serial+parallel to ensure that voltage is dropped to correct levels (when powered from car/bike battery or USB 5V powerbank) which is around 4,4V for SIM8XX module (5V would damage SIM800L) and 2A current can be handled. Also we have to secure that during current peaks the voltage will not drop below 3V for SIM800L ( otherwise SIM800L would restart itself during SMS/GPRS). 

The tracker can be powered 3 ways  ( and depending on powering method you can get rid of LM7805 or some 1N4007 diodes to simplify the design): 

a) powering directly from car/bike battery - ensure that proper cables are used (must sustain 2Amps) and attach small heatsink to LM7805 TO220 case. It will not get hot all the time but ensure that current/heat protection within LM7805 would not activate. In such case the tracker will consume in standby something like 12mA due to LM7805 drop (conversion from 12V to 5V). You may try to put some replacement of LM7805 like switching power supply step-down converter.

b) powering from USB 5V Powerbanks - it is good to use the cheapest USB powerbanks that do not have current sensor. Remember that GPS tracker will draw VERY LOW current ( lower than 2mA). Some advanced powerbanks tend to switch off USB 5V when they find out that there is very little current consumed. If you have Powerbank with signalling LED then I suggest to get rid of this LED to reduce power drain on Powerbank. 

c) Powering from 3 x AA battery pack (LR6). Estimated working time on set of batteries is more than 1 month due to low current ( less than 2mA). Batteries must be connected directly to VCC on ATTINY-ATMEGA/SIM8XX without any diodes. This will give something like 4.5V of powering voltage. Avoid Ni-Mh Accumulators. They give only 1.2V each. 

The solution has been field tested and works without a glitch. 

You can see how it works :
here :      https://www.youtube.com/watch?v=t7mvomytDq4
and here :  https://www.youtube.com/watch?v=546j1f_qA50
