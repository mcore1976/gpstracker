 DIY ultra cheap GPS bike/car tracker based on ATMEL ATTINY 2313/4313 and SIM800L module from China (SIM80X/90X depending on your location and frequency used). 

Total cost of parts is below 10USD

 The device when ringed by cellphone polls GPS location from nearest 2G cell and sends back location link to Google map with timestamp. 

The part list is (with the cost as in 2018): 
- SIM800L (3.5 USD on Aliexpress, but notice this module DOES NOT have GPS so we are polling info from base stations - if you want real GPS you need to use SIM808 breadboard) 
- ATMEL ATTINY2313 (2USD) 
- LM7805 (0.5USD) 
- 6x 1N4007 (0.3 USD) 
- 2x 1000uF / 16V capacitor ( 0.5 USD) 
- 100nF capacitor (0.2 USD) 
- universal PCB, pins & connector (2 USD) 

The code is written in avr-gcc and was uploaded via USBASP to ATTINY 2313(2313A/4313A). 
It takes about 1,9KB of Flash memory so the chip is almost full. 
Considering the SIM800L capability if more Flash memory is available ( ATTINY 4313 with 4KB) the chip could even upload the GPS data to some EMAIL/FTP/HTTP server to get car tracking history. 

The solution has low power consumption (current in standby is below 5-20mA ), works mostly in 2G standby and connects to GPRS/polls GPS only upon request. When SIM8xx module sends SMS/GPRS data it draws a lot of current ( up to 2A ) in short peaks so it is crucial to use good cables and thick  copper lines for GND and VCC on PCB. The voltage will additionaly drop during this situation so that is why such big capacitor is in use. 
In the design there are 1N4007 diodes connected in parallel to ensure that voltage is dropped to correct levels which is around 4,4V for SIM8XX module (5V would damage SIM800L). Also we have to secure that during current peaks the voltage will not drop below 3V for SIM800L ( otherwise SIM800L would restart itself during SMS/GPRS).

The tracker can be powered 2 ways :

a) powering from car/bike battery - ensure that proper cables are used (must sustain 2Amps) and attach small heatsink to LM7805 TO220 case. It will not get hot all the time but ensure that current/heat protection within LM7805 would not activate.

b) powering from USB 5V Powerbanks - it is good to used the cheapest USB powerbanks that do not have current sensor. Remember that GPS tracker will draw VERY LOW current. Some advanced powerbanks tend to disconnect 5V when they find out that  there is very little current.

The solution has benn field tested and works without a glitch. 

Have Fun !

Adam 
