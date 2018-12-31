/* ---------------------------------------------------------------------------
 * GPS car tracker on ATTINY2313 / ATTINY2313A + SIM800L module version 3.0
 * by Adam Loboda - adam.loboda@wp.pl
 * accuracy is nearest GSM cell around locator
 * baudrate 9600
 * please configure SIM800L to fixed 9600 first by AT+IPR=9600 command 
 * to ensure stability ans save config via AT&W command
 * in this program ATTINY never goes to sleep - there is no INT0 / RING interrupt
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
// internal RC oscillator with NO DIVISION, gives 0.2% error rate for 9600 bps UART speed
// for 8Meg : -U lfuse:w:0xe4:m -U hfuse:w:0xdf:m 
#define F_CPU 8000000UL

const char AT[] PROGMEM = { "  AT\n\r" }; // SPACE for wakeup from sleep mode
const char ISOK[] PROGMEM = { "OK" };
const char ISRING[] PROGMEM = { "RING" };
const char ISREG1[] PROGMEM = { "+CREG: 1,1" };  // registered in HPLMN
const char ISREG2[] PROGMEM = { "+CREG: 1,5" };  // registered in ROAMING NETWORK
const char SHOW_REGISTRATION[] PROGMEM = {"AT+CREG?\n\r"};
const char ENABLEREGREPORT[] PROGMEM = {"AT+CREG=1\n\r"};   //  enable reporting URC of losing 2G coverage by +CREG=0
const char NOCOVERAGE1[] PROGMEM =  {"+CREG= 1,0"};         //  network status - no network found  
const char PIN_IS_READY[] PROGMEM = {"+CPIN: READY"};
const char PIN_MUST_BE_ENTERED[] PROGMEM = {"+CPIN: SIM PIN"};

const char SHOW_PIN[] PROGMEM = {"AT+CPIN?\n\r"};
const char ECHO_OFF[] PROGMEM = {"ATE0\n\r"};
const char ENTER_PIN[] PROGMEM = {"AT+CPIN=\"1111\"\n\r"};
const char HANGUP[] PROGMEM = {"ATH\n\r"};
const char SMS1[] PROGMEM = {"AT+CMGF=1\r\n"};
const char SMS2[] PROGMEM = {"AT+CMGS=\""};        // for other networks if they show +XX in CLIP
const char DELSMS[] PROGMEM = {"AT+CMGDA=\"DEL ALL\"\r\n"};

// Disable SIM808 LED for further reduction of power consumption
const char DISABLELED[] PROGMEM = { "AT+CNETLIGHT=0\r\n" };

const char CRLF[] PROGMEM = {"\"\n\r"};
const char CLIP[] PROGMEM = {"AT+CLIP=1\r\n"};

// Flightmode ON OFF - for saving battery in underground garage
const char FLIGHTON[] PROGMEM = { "AT+CFUN=4\r\n" };
const char FLIGHTOFF[] PROGMEM = { "AT+CFUN=1\r\n" };

// Sleepmode ON OFF
const char SLEEPON[] PROGMEM = { "AT+CSCLK=2\r\n" };
const char SLEEPOFF[] PROGMEM = { "AT+CSCLK=0\r\n" };

// Fix UART speed to 9600 bps
const char SET9600[] PROGMEM = { "AT+IPR=9600\r\n" };

// Save settings to SIM800L
const char SAVECNF[] PROGMEM = { "AT&W\r\n" };

// for sending SMS predefined text 
const char GOOGLELOC1[] PROGMEM = {"\r\n http://maps.google.com/maps?q="};
const char GOOGLELOC2[] PROGMEM = {","};
const char GOOGLELOC3[] PROGMEM = {"\r\n"};
const char LONG[] PROGMEM = {" UTC\n LONG="};
const char LATT[] PROGMEM = {" LATT="};

// definition of APN used for GPRS communication
// please put correct APN, USERNAME and PASSWORD here appropriate
// for your Mobile Network provider 
const char SAPBR1[] PROGMEM = {"AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n"};  
const char SAPBR2[] PROGMEM = {"AT+SAPBR=3,1,\"APN\",\"internet\"\r\n"};    // Put your mobile operator APN name here
const char SAPBR3[] PROGMEM = {"AT+SAPBR=3,1,\"USER\",\"internet\"\r\n"};   // Put your mobile operator APN username here
const char SAPBR4[] PROGMEM = {"AT+SAPBR=3,1,\"PWD\",\"internet\"\r\n"};    // Put your mobile operator APN password here 
// PDP context commands
const char SAPBROPEN[] PROGMEM = {"AT+SAPBR=1,1\r\n"};  // open IP bearer
const char SAPBRQUERY[] PROGMEM = {"AT+SAPBR=2,1\r\n"};  // query IP bearer
const char SAPBRCLOSE[] PROGMEM = {"AT+SAPBR=0,1\r\n"};   // close bearer 
const char SAPBRSUCC[] PROGMEM = {"+SAPBR: 1,1"}; // bearer was succesfull we are not checking IP assigned
const char CHECKGPS[] PROGMEM = {"AT+CIPGSMLOC=1,1\r\n"};  // check AGPS position of nearest GSM CELL 


// Initialize UART to 9600 baud with 8N1. 
void init_uart(void);

// Send and receive functions, that run without ISRs
uint8_t receive_uart();
void send_uart(uint8_t c);

// Send a string
void uart_puts(const char *s);
// Send a PROGMEM string
void uart_puts_P(const char *s);


#ifndef TRUE
#define TRUE 1
#define FALSE 0 
#endif

//#define BAUD 19200
#define BAUD 9600
// 4.096MHz
//  4800: 52.3333333
//  9600: 25.6666667
// 14400: 16.7777778
// 19600: 12.06
// 28800: 7.8889
// 38400: 5.6667

#define MYUBBR ((F_CPU / (BAUD * 16L)) - 1)
#define BUFFER_SIZE 40

// buffers for number of phone, responses from modem, longtitude & latitude data
volatile static uint8_t response[BUFFER_SIZE] = "1234567890123456789012345678901234567890";
volatile static uint8_t response_pos = 0;
volatile static uint8_t phonenumber[15] = "123456789012345";
volatile static uint8_t phonenumber_pos = 0;
volatile static uint8_t latitude[10] = "1234567890";
volatile static uint8_t latitude_pos = 0;
volatile static uint8_t longtitude[10] = "1234567890";
volatile static uint8_t longtitude_pos = 0;
volatile static uint8_t buf[20];  // buffer to copy string from PROGMEM


/*
 * init_uart
 */
void init_uart(void) {
  // set baud rate
  UBRRH = (uint8_t)(MYUBBR >> 8); 
  UBRRL = (uint8_t)(MYUBBR);
  // enable receive and transmit and NO INTERRUPT
  UCSRB = (1 << RXEN) | (1 << TXEN) ;
  // set frame format
  UCSRC = (0 << USBS) | (3 << UCSZ0); // asynchron 8n1
  // UCSRC = (1 << USBS) | (3 << UCSZ0);
  
}



/*
 * send_uart
 * Sends a single char to UART without ISR
 */
void send_uart(uint8_t c) {
  // wait for empty data register
  while (!(UCSRA & (1<<UDRE)));
  // set data into data register
  UDR = c;
}



/*
 * receive_uart
 * Receives a single char without ISR
 */
uint8_t receive_uart() {
  while ( !(UCSRA & (1<<RXC)) ) 
    ; 
  return UDR; 
}



// function to search RX buffer for response  SUB IN RX_BUFFER STR
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

 

/*
 * uart_puts
 * Sends a string.
 */
void uart_puts(const char *s) {
  while (*s) {
    send_uart(*s);
    s++;
  }
}



/*
 * uart_puts_P
 * Sends a PROGMEM string.
 */
void uart_puts_P(const char *s) {
  while (pgm_read_byte(s) != 0x00) {
    send_uart(pgm_read_byte(s++));
  }
}


// --------------------------------------------------------------------------------------------
// READLINE from serial port that starts with CRLF and ends with CRLF and put to 'response' buffer what read
// --------------------------------------------------------------------------------------------
uint8_t readline()
{
  uint16_t char1, i , wholeline ;
  // wait for first CR-LF or exit after timeout i cycles
   i = 0;
   wholeline = 0;
   response_pos = 0;
  //
   do {
      // read chars in pairs to find combination CR LF
      char1 = receive_uart();
      // if CR-LF combination detected start to copy the response
      if   (  char1 != 0x0a && char1 != 0x0d ) 
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
      } while ( wholeline == 0);
return(1);
}


// --------------------------------------------------------------------------------------------
// READ CELL GPS from AT+CIPGSMLOC output and put output to 'lattitude' and 'longtitude' buffers
// --------------------------------------------------------------------------------------------
uint8_t readcellgps()
{
  uint16_t char1;
  uint8_t i;
  // wait for first COMMA, set positions to start of each buffer
   longtitude_pos = 0;
   latitude_pos = 0;
   response_pos = 0;

      // wait for first COMMA sign
      do { 
           char1 = receive_uart();
         } while ( char1 != ',' );

          // if COMMA detected start to copy the response - LONGTITUDE first
      do  { 
           char1 = receive_uart();
           longtitude[longtitude_pos] = char1; 
           longtitude_pos++;
         } while ( char1 != ',' );
           longtitude[longtitude_pos-1] = NULL; 
           longtitude_pos=0;

      // if COMMA detected start to copy the response - LATITUDE second
      do  { 
           char1 = receive_uart();
          latitude[latitude_pos] = char1; 
          latitude_pos++;
         } while ( char1 != ',' );
           // put end of string to latitude
           latitude[latitude_pos-1] = NULL; 
           latitude_pos=0;

      // Now copy DATE & TIME UTC to response buffer and wait for CRLF to finish
        do  { 
           char1 = receive_uart();
           response[response_pos] = char1; 
           response_pos++;
         } while ( (char1 != '\r') && (char1 != '\n') );       // WAIT FOR CR LF
           response[response_pos-1] = NULL; 
           response_pos=0;
           char1 = receive_uart();  // read last CR or LF and exit

return (1);
}

// --------------------------------------------------------------------------------------------
// read PHONE NUMBER from AT+CLIP output and copy it to buffer for SMS sending
// --------------------------------------------------------------------------------------------
uint8_t readphonenumber()
{
  uint16_t char1;
  // wait for first quotation
   phonenumber_pos = 0;
      // wait for first quotation sign
      do { 
           char1 = receive_uart();
         } while ( char1 != '\"' );
      // if quotation detected start to copy the response - phonenumber 
      do  { 
           char1 = receive_uart();
           phonenumber[phonenumber_pos] = char1; 
           phonenumber_pos++;
         } while ( char1 != '\"' );    // until end of quotation
     // put NULL to end the string phonenumber
           phonenumber[phonenumber_pos-1] = NULL; 
           phonenumber_pos=0;
     // wait for CRLF for new response to empty RX buffer
           do { 
           char1 = receive_uart(); 
           } while ( char1 != 0x0a && char1 != 0x0d  );
return (1);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// delay procedure ASM based because _delay_ms() is working bad for 8 MHz clock MCU
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// --------------------------------------------------------------------------------------------
// delay partucular number of seconds  (up to UINT8 = 255 seconds)
// --------------------------------------------------------------------------------------------

void delay_sec(uint8_t i)
{
while(i > 0)
{
// Generated by delay loop calculator
// at http://www.bretmulvey.com/avrdelay.html
//
// Delay 8 000 000 cycles
// 1s at 8.0 MHz

asm volatile (
    "    ldi  r18, 41"	"\n"
    "    ldi  r19, 150"	"\n"
    "    ldi  r20, 128"	"\n"
    "1:  dec  r20"	"\n"
    "    brne 1b"	"\n"
    "    dec  r19"	"\n"
    "    brne 1b"	"\n"
    "    dec  r18"	"\n"
    "    brne 1b"	"\n"
);

i--;  // decrease another second

};    // repeat until i not zero

}


//////////////////////////////////////////
// SIM800L initialization procedures
//////////////////////////////////////////

// --------------------------------------------------------------------------------------------
// wait for first AT in case SIM800L is starting up
// --------------------------------------------------------------------------------------------
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
		delay_sec(2);
                uart_puts_P(ECHO_OFF);

             return initialized2;
}

// --------------------------------------------------------------------------------------------
// check if PIN is needed and enter PIN 1111 
// --------------------------------------------------------------------------------------------
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
                        {  uart_puts_P(ENTER_PIN);   // ENTER PIN 1111
                           delay_sec(1);
                        };                  
                    };
                  
              } while (initialized2 == 0);
   return initialized2;
}

// --------------------------------------------------------------------------------------------
// check if registered to the network
// --------------------------------------------------------------------------------------------
uint8_t checkregistration()
{
  uint8_t initialized2;
     // readline and wait for STATUS NETWORK REGISTRATION from SIM800L
     // first 2 networks preferred from SIM list are OK
                  initialized2 = 0;
              do { 
		delay_sec(5);
                 uart_puts_P(SHOW_REGISTRATION);
                if (readline()>0)
                 {			   
                    memcpy_P(buf, ISREG1, sizeof(ISREG1));
                   if (is_in_rx_buffer(response, buf) == 1)  initialized2 = 1; 
                    memcpy_P(buf, ISREG2, sizeof(ISREG2));
                   if (is_in_rx_buffer(response, buf) == 1)  initialized2 = 1; 
                    memcpy_P(buf, NOCOVERAGE1, sizeof(NOCOVERAGE1));
                   if (is_in_rx_buffer(response, buf) == 1) 
                    { 
                      // if not registered or something wrong turn off RADIO for 4 minutes 
                      // this is not to drain battery in underground garage 
                      uart_puts_P(FLIGHTON);    // enable airplane mode - turn off radio
                      delay_sec(240);
                      uart_puts_P(FLIGHTOFF);  // disable airplane mode - turn on radio and start to search for networks
                      delay_sec(60);           // give reasonable time to search for 2G network
                   };
                  }; // end of READLINE IF
                // end of DO loop
                } while (initialized2 == 0);

      return initialized2;
};
 
// --------------------------------------------------------------------------------------------
// provision GPRS APNs and passwords - we are not checking if any error not to get deadlocks
// --------------------------------------------------------------------------------------------
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


// *********************************************************************************************************
//
//                                                    MAIN PROGRAM
//
// *********************************************************************************************************

int main(void) {

  uint8_t initialized, attempt = 0;

  //char buf[20];  // buffer to copy string from PROGMEM

 
  // initialize 9600 baud 8N1 RS232
  init_uart();

  // delay 10 seconds for safe SIM800L startup and network registration
  delay_sec(10);

          
       // try to communicate with SIM800L over AT
        checkat();
	delay_sec(2);

       // Fix UART speed to 9600 bps to disable autosensing
        uart_puts_P(SET9600); 
	delay_sec(2);

       // configure reporting of 2G COVERAGE status
        uart_puts_P(ENABLEREGREPORT);
        delay_sec(2);

       // Save settings to SIM800L
        uart_puts_P(SAVECNF);
        delay_sec(3);

       // check pin status, registration status and provision APN settings
        checkpin();
        checkregistration();
        provisiongprs();
	delay_sec(2);
 
       // read phone number of incoming voice call by CLIP, will be needed for SMS sending 
        uart_puts_P(CLIP); 
        delay_sec(2);
       //and close the beare just in case it was open
        uart_puts_P(SAPBRCLOSE);
        delay_sec(2);
 
     // neverending LOOP

       while (1) {

             do { 

                // delete all SMSes and SMS confirmation to keep SIM800L memory empty   
                   uart_puts_P(SMS1);
                   delay_sec(2); 
                   uart_puts_P(DELSMS);
                   delay_sec(3);

                // WAIT FOR RING message - incoming voice call and send SMS or restart RADIO module if no signal
                   initialized = 0;

                // disable LEDs on SIM800L module
                   uart_puts_P(DISABLELED);
                   delay_sec(1);

               // enter SLEEP MODE of SIM800L for power saving ( will be interrupted by incoming voice call or SMS ) 
                   uart_puts_P(SLEEPON); 
                   delay_sec(2);
     

               // THERE WAS URC message : incoming CALL / SMS or 2G coverage was lost
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
                      // check PIN status maybe SIM800 restarted itself
                       checkpin();
                       // maybe there was NO COVERAGE URC...
                       checkregistration();
                       provisiongprs();
                      // read phone number of incoming voice call by CLIP, will be needed for SMS sending       
                      uart_puts_P(CLIP);
                      delay_sec(2);
                    //and close the beare just in case it was open
                      uart_puts_P(SAPBRCLOSE);
                      delay_sec(2);
                    // there was something different than RING so we need to go back to the beginning 
                      initialized = 0;
                      }; // end of ELSE

                    }; // END of READLINE IF
                 
                } while ( initialized == 0);    // end od DO-WHILE, go to beggining and enter SLEEPMODE again 

               initialized = 0;

           // Create connection to GPRS network - 3 attempts if needed, if not succesfull restart the modem 
           attempt = 0;
           do { 

            //and close the bearer first maybe there was an error or something
              delay_sec(2);
              uart_puts_P(SAPBRCLOSE);
           
           // make GPRS network attach and open IP bearer
              delay_sec(2);
              uart_puts_P(SAPBROPEN);

           // query PDP context for IP address after several seconds
           // check if GPRS attach was succesfull, do it several times if needed
              initialized = 0; 
              delay_sec(5);
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
               // GET CELL ID OF BASE STATION and query Google for coordinates then send over SMS with google map loc
        	delay_sec(1);
               uart_puts_P(CHECKGPS);
               // parse GPS coordinates from the answer to SMS buffer
               readcellgps();
                // send a SMS in plain text format
               uart_puts_P(SMS1);
               delay_sec(1); 
               // compose an SMS from fragments - interactive mode CTRL Z at the end
               uart_puts_P(SMS2);
			   uart_puts(phonenumber);  // send phone number received from CLIP
			   uart_puts_P(CRLF);   			   
               delay_sec(1); 
               // put info about DATE,TIME, LONG, LATT
               uart_puts(response);  // send Date & Time info from AGPS cell info
               uart_puts_P(LONG);  // send LONGTITUDE
               uart_puts(longtitude); // from buffer
               uart_puts_P(LATT);  // send LATITUDE
               uart_puts(latitude);  // from buffer
               // put link to GOOGLE MAPS
               uart_puts_P(GOOGLELOC1); // send http ****
               uart_puts(latitude);   // send Cellinfo
               uart_puts_P(GOOGLELOC2); // send comma
               uart_puts(longtitude);   // send Cellinfo
               uart_puts_P(GOOGLELOC3); // send CRLF
               delay_sec(1); 
               send_uart(26);   // ctrl Z to end SMS

              //and close the bearer 
              delay_sec(5);
              uart_puts_P(SAPBRCLOSE);
          } /// end of commands when GPRS is working

       
        delay_sec(10);
        // go to the beginning and enter sleepmode on SIM800L and ATTINY2313 again for power saving

        // end of neverending loop
        };

 
    // end of MAIN code 
}

