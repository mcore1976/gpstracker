 DIY ultra cheap (A)GPS car tracker based on ATMEL ATTINY 2313/4313 and SIM800L module from China (SIM80X/90X depending on your location and frequency used). 

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
The solution has low power consumption (current in standby is below 30mA ), works mostly in 2G standby and connects to GPRS/polls GPS only upon request. Anyway there is possibility to attach Li-Ion cell to the PCB so it could be used to power the tracker when car battery is disconnected but you would have to attach some heatsink to LM7805 stabilizer because it will charge Li-Ion battery when car battery is available.
