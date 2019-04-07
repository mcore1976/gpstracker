/* ---------------------------------------------------------------------------
 * GPS car tracker on ATMEGA328P + SIM800L module version 2.1
 * by Adam Loboda - adam.loboda@wp.pl
 * a) accuracy is nearest GSM cell around locator
 * b) baudrate UART is 9600 as most stable using RC internal clock
 * c) ATMEGA clock was lowered to 1MHz (8 MHz div 8) for lower power consumption
 *
 * other considerations:
 *
 * please configure SIM800L to fixed 9600 first by AT+IPR=9600 command 
 * to ensure stability ans save config via AT&W command
 * you may do it by connecting FTDI232 serial port converter to PC
 * and using putty enter commands directly to SIM800L
 *
 * because of sleepmode use on ATMEGA328P : INT0 pin (#4) of ATMEGA328P 
 * must be connected to RI/RING ping on SIM800L module
 * RING falling edge initiates interrupt INT0 and wakes up ATMEGA328
 * 
 * other connections to be made :
 * SIM800L RXD to ATMEGA328 TXD PIN #3,
 * SIM800L TXD to ATMEGA328 RXD PIN #2
 * ATMEGA328 VCC (PIN #7) to SIM800L VCC and +4.4V
 * ATMEGA328 GND (PIN #8 and PIN #22) to SIM800L GND and 0V 
 * the rest is the same as on ATTINY2313 schematic
 * ---------------------------------------------------------------------------
 */

#include <inttypes.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <string.h>
#include <avr/power.h>

#define UART_NO_DATA 0x0100
// internal RC oscillator 8MHz with divison by 8 and U2X0 = 1, gives 0.2% error rate for 9600 bps UART speed
// and lower current consumption
// for 1MHz : -U lfuse:w:0x62:m     on ATMEGA328P
#define F_CPU 1000000UL

#define BAUD 9600
// formula for 1MHz clock and U2X0 = 1 double UART speed 
#define MYUBBR ((F_CPU / (BAUD * 8L)) - 1)



const char AT[] PROGMEM = { "AT\n\r" };
const char ISOK[] PROGMEM = { "OK" };
const char ISRING[] PROGMEM = { "RING" };
const char ISREG1[] PROGMEM = { "+CREG: 0,1" };             // SIM registered in HPLMN 
const char ISREG2[] PROGMEM = { "+CREG: 0,5" };             // SIM registeref in ROAMING NETWORK
const char SHOW_REGISTRATION[] PROGMEM = {"AT+CREG?\n\r"};  // check registration status
const char DISREGURC[] PROGMEM = {"AT+CREG=0\n\r"};         // we disable realtime reporting of 2G network status

const char PIN_IS_READY[] PROGMEM = {"+CPIN: READY"};              
const char PIN_MUST_BE_ENTERED[] PROGMEM = {"+CPIN: SIM PIN"};

const char SHOW_PIN[] PROGMEM = {"AT+CPIN?\n\r"};
const char ECHO_OFF[] PROGMEM = {"ATE0\n\r"};
const char ENTER_PIN[] PROGMEM = {"AT+CPIN=\"1111\"\n\r"};   // put PINCODE of your SIMCARD here if you have different than 1111...
const char CFGRIPIN[] PROGMEM = {"AT+CFGRI=1\n\r"};
const char HANGUP[] PROGMEM = {"ATH\n\r"};

const char SMS1[] PROGMEM = {"AT+CMGF=1\r\n"};
const char SMS2[] PROGMEM = {"AT+CMGS=\""};       
const char DELSMS[] PROGMEM = {"AT+CMGDA=\"DEL ALL\"\r\n"};

const char CRLF[] PROGMEM = {"\"\n\r"};
const char CLIP[] PROGMEM = {"AT+CLIP=1\r\n"};


// Flightmode ON OFF - for saving battery while in underground garage with no GSM signal
const char FLIGHTON[] PROGMEM = { "AT+CFUN=4\r\n" };
const char FLIGHTOFF[] PROGMEM = { "AT+CFUN=1\r\n" };

// Sleepmode ON OFF, SIM800L supports mode #2
const char SLEEPON[] PROGMEM = { "AT+CSCLK=2\r\n" };
const char SLEEPOFF[] PROGMEM = { "AT+CSCLK=0\r\n" };

// Fix UART speed to 9600 bps
const char SET9600[] PROGMEM = { "AT+IPR=9600\r\n" };

// Save settings to SIM800L
const char SAVECNF[] PROGMEM = { "AT&W\r\n" };

// Disable SIM800L LED for further reduction of power consumption
const char DISABLELED[] PROGMEM = { "AT+CNETLIGHT=0\r\n" };

// for sending SMS predefined text 
const char GOOGLELOC1[] PROGMEM = {"\r\n http://maps.google.com/maps?q="};
const char GOOGLELOC2[] PROGMEM = {","};
const char GOOGLELOC3[] PROGMEM = {"\r\n"};
const char LONG[] PROGMEM = {" UTC\n LONGTITUDE="};
const char LATT[] PROGMEM = {" LATITUDE="};
const char BATT[] PROGMEM = {"\nBATTERY[mV]="};

// definition of APN used for GPRS communication
// please put correct APN, USERNAME and PASSWORD here appropriate
// for your Mobile Network provider 
const char SAPBR1[] PROGMEM = {"AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n"};  
const char SAPBR2[] PROGMEM = {"AT+SAPBR=3,1,\"APN\",\"internet\"\r\n"};    // Put your mobile operator APN name here
const char SAPBR3[] PROGMEM = {"AT+SAPBR=3,1,\"USER\",\"internet\"\r\n"};   // Put your mobile operator APN username here
const char SAPBR4[] PROGMEM = {"AT+SAPBR=3,1,\"PWD\",\"internet\"\r\n"};    // Put your mobile operator APN password here 
// PDP context commands
const char SAPBROPEN[] PROGMEM = {"AT+SAPBR=1,1\r\n"};     // open IP bearer
const char SAPBRQUERY[] PROGMEM = {"AT+SAPBR=2,1\r\n"};    // query IP bearer
const char SAPBRCLOSE[] PROGMEM = {"AT+SAPBR=0,1\r\n"};    // close bearer 
const char SAPBRSUCC[] PROGMEM = {"+SAPBR: 1,1"};          // bearer was succesfull we are not checking IP assigned

// check statuses & cells
const char CHECKGPS[] PROGMEM = {"AT+CIPGSMLOC=1,1\r\n"};   // check GPS position of nearest GSM CELL via Google API
const char CHECKBATT[] PROGMEM = {"AT+CBC\r\n"};            // check battery voltage 


// buffers for number of phone, responses from modem, longtitude & latitude data
#define BUFFER_SIZE 80
volatile static uint8_t response[BUFFER_SIZE] = "12345678901234567890123456789012345678901234567890123456789012345678901234567890";
volatile static uint8_t response_pos = 0;
volatile static uint8_t phonenumber[20] = "12345678901234567890"; 
volatile static uint8_t phonenumber_pos = 0;
volatile static uint8_t latitude[20] = "12345678901234567890";
volatile static uint8_t latitude_pos = 0;
volatile static uint8_t longtitude[20] = "12345678901234567890";
volatile static uint8_t longtitude_pos = 0;
volatile static uint8_t buf[40];                                  // buffer to copy string from PROGMEM
volatile static uint8_t battery[10] = "1234567890";                     // for battery voltage checking
volatile static uint8_t battery_pos = 0;


// ----------------------------------------------------------------------------------------------
// init_uart
// ----------------------------------------------------------------------------------------------
void init_uart(void) {
  // double speed by U2X0 flag = 1 to have 0.2% error rate on 9600 baud
 UCSR0A = (1<<U2X0);
  // set baud rate from PRESCALER
 UBRR0H = (uint8_t)(MYUBBR>>8);
 UBRR0L = (uint8_t)(MYUBBR);
 UCSR0B|=(1<<TXEN0); //enable TX
 UCSR0B|=(1<<RXEN0); //enable RX
  // set frame format for SIM808 communication
 UCSR0C|=(1<<UCSZ00)|(1<<UCSZ01); // no parity, 1 stop bit, 8-bit data 
}



// ----------------------------------------------------------------------------------------------
// send_uart
// Sends a single char to UART without ISR
// ----------------------------------------------------------------------------------------------
void send_uart(uint8_t c) {
  // wait for empty data register
  while (!(UCSR0A & (1<<UDRE0)));
  // set data into data register
  UDR0 = c;
}



// ----------------------------------------------------------------------------------------------
// receive_uart
// Receives a single char without ISR
// ----------------------------------------------------------------------------------------------
uint8_t receive_uart() {
  while ( !(UCSR0A & (1<<RXC0)) ) 
    ; 
  return UDR0; 
}


// ----------------------------------------------------------------------------------------------
// function to search RX buffer for response  SUB IN RX_BUFFER STR
// ----------------------------------------------------------------------------------------------
uint8_t is_in_rx_buffer(char *str, char *sub) {
   uint8_t i, j=0, k;
    for(i=0; i<BUFFER_SIZE; i++)
    {
      if(str[i] == sub[j])
     {
       for(k=i, j=0; str[k] && sub[j]; j++, k++)  // if NOT NULL on each of strings
            if(str[k]!=sub[j])   break; // if different - start comparision with next char
       // if(!sub[j]) return 1; 
        if(j == strlen(sub)) return 1;  // full substring has been found        
      }
     }
     // substring not found
    return 0;
}

 

// ----------------------------------------------------------------------------------------------
// uart_puts
// Sends a string.
// ----------------------------------------------------------------------------------------------
void uart_puts(const char *s) {
  while (*s) {
    send_uart(*s);
    s++;
  }
}



// ----------------------------------------------------------------------------------------------
// uart_puts_P
// Sends a PROGMEM string.
// ----------------------------------------------------------------------------------------------
void uart_puts_P(const char *s) {
  while (pgm_read_byte(s) != 0x00) {
    send_uart(pgm_read_byte(s++));
  }
}


// ---------------------------------------------------------------------------------------------------------------
// READLINE from serial port that starts with CRLF and ends with CRLF and put to 'response' buffer what read
// ---------------------------------------------------------------------------------------------------------------
uint8_t readline()
{
  uint16_t char1, wholeline ;
  uint8_t i;

  // wait for first CR-LF or exit after timeout i cycles not to overload the char buffer
   i = 0;
   wholeline = 0;
   response_pos = 0;

  //
   do {
      // read chars in pairs to find combination CR LF
      char1 = receive_uart();
      // if CR-LF combination detected start to copy the response
      if   (  (char1 != 0x0a) && (char1 != 0x0d) && (i<80) ) 
         { response[response_pos] = char1; 
           response_pos++;
         };
      if    (  char1 == 0x0a || char1 == 0x0d )              
         {  
           // if the line was received and this is only CR/LF ending :
           if (response_pos > 0) // this is EoL
               { response[response_pos] = NULL;
                 response_pos = 0;
                  wholeline = 1;  };
          // just skip this CRLF character and wait for valuable one
               
         };
      // if buffer is empty exit from function there is nothing to read from
      i++;
      } while ( (wholeline == 0) && (i<80) );

return (1);
}


// --------------------------------------------------------------------------------------------------------
// READ CELL GPS from AT+CIPGSMLOC output and put output to 'lattitude' and 'longtitude' buffers
// --------------------------------------------------------------------------------------------------------
uint8_t readcellgps()
{
  uint16_t char1;
  uint8_t i;
  // wait for first COMMA, set positions to start of each buffer
   longtitude_pos = 0;
   latitude_pos = 0;
   response_pos = 0;

  // i counter of received chars... not to get deadlock. 80 chars max as a safe fuse 
   i = 0;

      // wait for first COMMA sign after +CIPGSMLOC: message
      do { 
           char1 = receive_uart();
           i++;
         } while ( (char1 != ',') && (i<20));
      
      // if 'i' fuse == 20 chars and no comma sign return with 0, probably module restarted itself
          if (i == 20) return(0);


          // if COMMA detected start to copy the response - LONGTITUDE first
      do  { 
           char1 = receive_uart();
           longtitude[longtitude_pos] = char1; 
           longtitude_pos++;
           i++;
         } while ( (char1 != ',') && (i<70) );
           longtitude[longtitude_pos-1] = NULL; 
           longtitude_pos=0;

      // if COMMA detected start to copy the response - LATITUDE second
      do  { 
           char1 = receive_uart();
          latitude[latitude_pos] = char1; 
          latitude_pos++;
          i++;
         } while ( (char1 != ',') && (i<70) );
           // put end of string to latitude
           latitude[latitude_pos-1] = NULL; 
           latitude_pos=0;

      // Now copy DATE & TIME UTC to response buffer and wait for CRLF to finish
        do  { 
           char1 = receive_uart();
           response[response_pos] = char1; 
           response_pos++;
           i++;
         } while ( (char1 != '\r') && (char1 != '\n') && (i<70) );       // WAIT FOR CR LF
           response[response_pos-1] = NULL; 
           response_pos=0;
           char1 = receive_uart();  // read last CR or LF and exit

return (1);
}

// ----------------------------------------------------------------------------------------
// read PHONE NUMBER from AT+CLIP output and copy it to buffer for SMS sending
// ----------------------------------------------------------------------------------------
uint8_t readphonenumber()
{
  uint16_t char1;
  uint8_t i;
  // wait for first quotation
   phonenumber_pos = 0;
   i = 0;

       // wait for "CLIP:" and space
      do { 
           char1 = receive_uart();
           i++;
         } while ( (char1 != ':') && (i<80) );

      // wait for first quotation sign, 'i' protection of buffer overload
      do { 
           char1 = receive_uart();
           i++;
         } while ( (char1 != '\"') && (i<80)  );
      // if quotation detected start to copy the response - phonenumber 
      do  { 
           char1 = receive_uart();
           phonenumber[phonenumber_pos] = char1; 
           phonenumber_pos++;
           i++;
         } while ( (char1 != '\"') && (i<80) );    // until end of quotation
     // put NULL to end the string phonenumber
           phonenumber[phonenumber_pos-1] = NULL; 
           phonenumber_pos=0;
     // wait for CRLF for new response to empty RX buffer
           do { 
           char1 = receive_uart(); 
           i++;
           } while ( (char1 != 0x0a)  && (char1 != 0x0d) && (i<80)  );
return (1);
}

// ----------------------------------------------------------------------------------------
// Read BATTERY VOLTAGE in milivolts from AT+CBC output and put results to 'battery' buffer
// ----------------------------------------------------------------------------------------

uint8_t readbattery()
{
  uint16_t char1;
  uint8_t i;

   i = 0; 
   battery_pos = 0;

      // wait for first COMMA sign
      do { 
           char1 = receive_uart();
           i++;
         } while ( (char1 != ',') && (i<70) );

      // wait for second COMMA sign
      do { 
           char1 = receive_uart();
           i++;
         } while ( (char1 != ',') && (i<70) );

      // if 2 COMMA detected start to copy battery voltage to buffer 
      do  { 
           char1 = receive_uart();
           battery[battery_pos] = char1; 
           battery_pos++;
           i++;
         } while ( (char1 != 0x0a) && (char1 != 0x0d) && (i<70)   );

          battery[battery_pos-1] = NULL; 
          battery_pos=0;

return (1);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// delay procedure ASM based because _delay_ms() is working bad for 1 MHz clock MCU
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// delay partucular number of i seconds,  i < 255

void delay_sec(uint8_t i)
{
while(i > 0)
{
// Delay 1 000 000 cycles
// 1s at 1 MHz

asm volatile (
    "    ldi  r18, 6"	"\n"
    "    ldi  r19, 19"	"\n"
    "    ldi  r20, 174"	"\n"
    "1:  dec  r20"	"\n"
    "    brne 1b"	"\n"
    "    dec  r19"	"\n"
    "    brne 1b"	"\n"
    "    dec  r18"	"\n"
    "    brne 1b"	"\n"
    "    rjmp 1f"	"\n"
    "1:"	"\n"
);

i--;  // decrease another second

};    // repeat until i not zero

}



//////////////////////////////////////////
// SIM800L initialization procedures
//////////////////////////////////////////

// -------------------------------------------------------------------------------
// wait for first AT in case SIM800L is starting up
// -------------------------------------------------------------------------------
uint8_t checkat()
{
  uint8_t initialized2;

// wait for first OK while sending AT - autosensing speed on SIM800L, but we are working 9600 bps
// SIM 800L can be set by AT+IPR=9600  to fix this speed
// which I do recommend by connecting SIM800L to PC using putty and FTD232 cable

                 initialized2 = 0;
              do { 
               uart_puts_P(AT);
                if (readline()>0)
                   {
                    memcpy_P(buf, ISOK, sizeof(ISOK));                     
                   if (is_in_rx_buffer(response, buf) == 1)  initialized2 = 1;                  
                   };
               delay_sec(1);
               } while (initialized2 == 0);

        // send ECHO OFF
                uart_puts_P(ECHO_OFF);
		delay_sec(1);

             return (1);
}

// -------------------------------------------------------------------------------
// check if PIN is needed and enter PIN 1111 
// -------------------------------------------------------------------------------
uint8_t checkpin()
{

  uint8_t initialized2;
     // readline and wait for PIN CODE STATUS if needed send PIN 1111 to SIM card if required
                  initialized2 = 0;
              do { 
		delay_sec(2);
               uart_puts_P(SHOW_PIN);
                if (readline()>0)
                   {
                    memcpy_P(buf, PIN_IS_READY, sizeof(PIN_IS_READY));
                  if (is_in_rx_buffer(response, buf ) == 1)       initialized2 = 1;                                         
                    memcpy_P(buf, PIN_MUST_BE_ENTERED, sizeof(PIN_MUST_BE_ENTERED));
                  if (is_in_rx_buffer(response, buf) == 1)     
                        {  
                           delay_sec(1);
                           uart_puts_P(ENTER_PIN);   // ENTER PIN 1111
                           delay_sec(1);
                        };                  
                    };
                  
              } while (initialized2 == 0);

   return (1);
}


// -------------------------------------------------------------------------------
// check if registered to the network
// -------------------------------------------------------------------------------
uint8_t checkregistration()
{
  uint8_t initialized2, attempt2, nbrminutes;
      initialized2 = 0;
      attempt2 = 0;
      nbrminutes = 0;

    // check if already registered first and quit immediately if true
     delay_sec(1);
     uart_puts_P(SHOW_REGISTRATION);
     if (readline()>0)
        {                                    
         memcpy_P(buf, ISREG1, sizeof(ISREG1));
         if (is_in_rx_buffer(response, buf) == 1)  return(1); 
         memcpy_P(buf, ISREG2, sizeof(ISREG2));
         if (is_in_rx_buffer(response, buf) == 1)  return(1); 
        } 

              do { 
                   // give reasonable time to look for 2G coverage and then check, maybe on the move...
                  delay_sec(1);
                  uart_puts_P(FLIGHTOFF);    // DISABLE airplane mode - just in case...
	          delay_sec(120);
                  uart_puts_P(SHOW_REGISTRATION);
                  // readline and wait for STATUS NETWORK REGISTRATION from SIM800L
                  // first 2 networks preferred from SIM list are OK

                if (readline()>0)
                   {			   
                    memcpy_P(buf, ISREG1, sizeof(ISREG1));
                   if (is_in_rx_buffer(response, buf) == 1)  initialized2 = 1; 
                    memcpy_P(buf, ISREG2, sizeof(ISREG2));
                   if (is_in_rx_buffer(response, buf) == 1)  initialized2 = 1; 

                   // if 2G network not found make backoff for 1 hour - maybe in underground garage or something...
                   if (initialized2 == 0)
                     {  
                      // if not registered or something wrong turn off RADIO for  minutes 
                      // this is not to drain battery in underground garage 
                      delay_sec(1);
                      uart_puts_P(FLIGHTON);    // enable airplane mode - turn off radio
                      delay_sec(1);
                     // enter SLEEP MODE of SIM800L for power saving when no coverage 
                      uart_puts_P(SLEEPON); 
                     // now wait XX min before turning on radio again, here XX = 30 min
                       for (nbrminutes = 0; nbrminutes<30; nbrminutes++) 
                          { 
                          delay_sec(60); 
                          };  
   
                      // send first dummy AT command
                       uart_puts_P(AT);
                       delay_sec(1); 

                       uart_puts_P(SLEEPOFF);  // switch off to SLEEPMODE = 0
                       delay_sec(1); 

                     };     // end of no-coverage IF


                  };    // end of READLINE-IF
             
                attempt2++; // increase number of attempts     
                // end of DO loop
                } while ( (initialized2 == 0) && (attempt2 < 48) );   // stop after 24 hours of searching 2G network

      return (1);
};


 
// ---------------------------------------------------------------------------------------------
// provision GPRS APNs and passwords - we are not checking if any error not to get deadlocks
// ---------------------------------------------------------------------------------------------
uint8_t provisiongprs()
{
     // connection to GPRS for AGPS basestation data - provision APN and username
               	delay_sec(1);
               uart_puts_P(SAPBR1);
		delay_sec(1);
               uart_puts_P(SAPBR2);
             // only if username password in APN is needed
		delay_sec(1);
               uart_puts_P(SAPBR3);
		delay_sec(1);
               uart_puts_P(SAPBR4);
        	delay_sec(1);   
  return 1;
}




// -------------------------------------------------------------------------------
// POWER SAVING mode handling to reduce the battery consumption
// Required connection between SIM800L RI/RING pin and ATMEGA328P INT0/D2 pin
// -------------------------------------------------------------------------------

void sleepnow(void)
{

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    sleep_enable();

    DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
    // PD2 (PCINT0 pin) is now an input

    PORTD |= (1 << PORTD2);    // turn On the Pull-up
    // PD2 is now an input with pull-up enabled

    // stop interrupts for configuration period
    cli(); 

    // update again INT0 conditions
    EICRA &= ~(1 << ISC01);    // set INT0 to trigger on low level
    EICRA &= ~(1 << ISC00);    // set INT0 to trigger on low level
    EIMSK |= (1 << INT0);     // Turns on INT0 (set bit)

    sei();                         //ensure interrupts enabled so we can wake up again

    sleep_cpu();                   //go to sleep

    // MCU ATTMEGA328P sleeps here until INT0 interrupt

    sleep_disable();               //wake up here

}

// when interrupt from INT0 disable next interrupts from RING pin of SIM800L and go back to main code
ISR(INT0_vect)
{

   EIMSK &= ~(1 << INT0);     // Turns off INT0 (clear bit)
}




// *********************************************************************************************************
//
//                                                    MAIN PROGRAM
//
// *********************************************************************************************************

int main(void) {

  uint8_t initialized, attempt = 0;
  uint8_t cellgpsavailable = 0;

  // initialize 9600 baud 8N1 RS232
  init_uart();

  // delay 10 seconds for safe SIM800L startup and network registration
  delay_sec(10);

  // try to communicate with SIM800L over AT
  checkat();
  delay_sec(2);

  // Fix UART speed to 9600 bps to disable autosensing
  uart_puts_P(SET9600); 
  delay_sec(1);

  // configure RI PIN activity for URC ( unsolicited messages like restart of the modem or battery low)
  uart_puts_P(CFGRIPIN);
  delay_sec(1);

  // disable reporting of 2G registration URC
  uart_puts_P(DISREGURC);
  delay_sec(1);

  // Save settings to SIM800L
  uart_puts_P(SAVECNF);
  delay_sec(3);

  // check pin status, registration status and provision APN settings
  checkpin();
  delay_sec(2);

  // disable flighmode
  uart_puts_P(FLIGHTOFF);
  delay_sec(120);

  // check if attached to 2G network
  checkregistration();
  provisiongprs();
 

  // neverending LOOP
     while (1) {

             do { 

                // WAIT FOR RING message - incoming voice call and send SMS or restart RADIO module if no signal
                   initialized = 0;
                   cellgpsavailable = 0;

                // delete all SMSes and SMS confirmation to keep SIM800L memory empty   
                   uart_puts_P(SMS1);
                   delay_sec(1); 
                   uart_puts_P(DELSMS);
                   delay_sec(2);


                // read phone number of incoming voice call by CLIP, will be needed for SMS sending 
                   uart_puts_P(CLIP); 
                   delay_sec(1);


                // Disable LED blinking on  SIM800L
                   uart_puts_P(DISABLELED);
                   delay_sec(1);

               // enter SLEEP MODE of SIM800L for power saving ( will be interrupted by incoming voice call or SMS ) 
                   uart_puts_P(SLEEPON); 
                   delay_sec(2);
     
               // enter SLEEP MODE on ATMEGA328P for power saving, requires RING/RI SIM800L pin connected to ATMEGA
                   sleepnow(); // sleep function called here 

               // THERE WAS RI / INT0 INTERRUPT AND SOMETHING WAS SEND OVER SERIAL WE NEED TO GET OFF SLEEPMODE AND READ SERIAL PORT
                if (readline()>0)
                   {
                    memcpy_P(buf, ISRING, sizeof(ISRING));  
                    if (is_in_rx_buffer(response, buf) == 1) 
                    { initialized = 1; 
                      readphonenumber(); 
                      // disable SLEEPMODE , hangup a call and proceed with sending SMS                  
                      uart_puts_P(AT);
                      delay_sec(1);
                      uart_puts_P(SLEEPOFF);
                      delay_sec(1);
                      uart_puts_P(HANGUP);
                      delay_sec(1);
                      } // end of IF


                     // if some other message than RING check if network is avaialble and SIM800L is operational  
                     else 
                      {
                      // disable SLEEPMODE                  
                       uart_puts_P(AT);
                       delay_sec(1);
                       uart_puts_P(SLEEPOFF);
                       delay_sec(1);
                      // check status of all functions 
                       checkpin();
                       checkregistration();
                      // read phone number of incoming voice call by CLIP, will be needed for SMS sending       
                    //and close the beare just in case it was open
                       uart_puts_P(SAPBRCLOSE);
                       delay_sec(2);
                    // there was something different than RING so we need to go back to the beginning - clear the flag 
                       initialized = 0;
                      }; // end of ELSE

                    }; // END of READLINE IF
                 
                } while ( initialized == 0);    // end od DO-WHILE, go to beggining and enter SLEEPMODE again 

           // clear the flag we gonna need it later
           initialized = 0;

           // Create connection to GPRS network - 3 attempts if needed, if not succesfull restart the modem 
           attempt = 0;
           do { 

            //and close the bearer first maybe there was an error or something
              delay_sec(1);
              uart_puts_P(SAPBRCLOSE);
           
           // now after this time network should be fine, provision APN settings
              delay_sec(1);
              provisiongprs();
              delay_sec(1);

           // make GPRS network attach and open IP bearer
              uart_puts_P(SAPBROPEN);

           // query PDP context for IP address after several seconds
           // check if GPRS attach was succesfull, do it several times if needed
              initialized = 0; 
              delay_sec(10);
              uart_puts_P(SAPBRQUERY);
              if (readline()>0)
                   {
                   // checking for properly attached
                    memcpy_P(buf, SAPBRSUCC, sizeof(SAPBRSUCC));                     
                   if (is_in_rx_buffer(response, buf) == 1)  initialized = 1;
                   // other responses simply ignored as there was no attach
                    };
            // increase attempt counter and repeat until not attached
            attempt++;
            } while ( (attempt < 3) && (initialized == 0) );
           

           // if GPRS was  succesfull the it is time send cell info to Google and query the GPS location
           if (initialized == 1)
           {

                // check battery voltage
        	delay_sec(1);
                uart_puts_P(CHECKBATT);
                readbattery();

               // GET CELL ID OF BASE STATION and query Google for coordinates then send over SMS with google map loc
               delay_sec(1);
               uart_puts_P(CHECKGPS);

               // parse GPS coordinates from the SIM808 answer to 'longtitude' & 'latitude' buffers
               // if possible otherwise some backup scenario
               cellgpsavailable=readcellgps();
               // if negative result please allow 60 sec fo SIM808 reboot
               if ( cellgpsavailable == 0 )  
                   { 
                     delay_sec(55);
                   }
               else     // proceed with SMS sending
                   {
                     delay_sec(1);
                     // send a SMS in plain text format
                     uart_puts_P(SMS1);
                     delay_sec(1); 
                     // compose an SMS from fragments - interactive mode CTRL Z at the end
                     uart_puts_P(SMS2);
	             uart_puts(phonenumber);              // send phone number received from CLIP
                     uart_puts_P(CRLF);   			   
                     delay_sec(1); 
                     // put info about DATE,TIME, LONG, LATITUDE
                     uart_puts(response);                 // send Date & Time info from AGPS cell info
                     uart_puts_P(LONG);                   // send LONGTITUDE
                     uart_puts(longtitude);               // from buffer
                     uart_puts_P(LATT);                   // send LATITUDE
                     uart_puts(latitude);                 // from buffer
                     // put battery info
                     uart_puts_P(BATT);                   // send BATTERY INFO
                     uart_puts(battery);                  // from buffer
                     // put link to GOOGLE MAPS
                     uart_puts_P(GOOGLELOC1);             // send http ****
                     uart_puts(latitude);                 // send latitude value
                     uart_puts_P(GOOGLELOC2);             // send comma
                     uart_puts(longtitude);               // send longtitude value
                     uart_puts_P(GOOGLELOC3);             // send CRLF
                     delay_sec(1); 
                     // end the SMS message
                     send_uart(26);   // ctrl Z to end SMS

                     }; // End of cellgpsavailable IF

              //and close the bearer 
              delay_sec(5);
              uart_puts_P(SAPBRCLOSE);

          } /// end of commands when GPRS is working
       
        // now go to the beginning and enter sleepmode on SIM800L and ATMEGA328P again for power saving
        delay_sec(10);
 
        // end of neverending loop
        };

 
    // end of MAIN code 
}

