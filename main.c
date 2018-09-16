/* -----------------------------------------------------------------------
 * AGPS car tracker on ATTINY2313 / ATTINY2313A + SIM800L module
 * by Adam Loboda - adam.loboda@wp.pl
 * baudrate 9600
 * please configure SIM800L to fixed 9600 first by AT+IPR=9600 command
 * to ensure stability
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

#define UART_NO_DATA 0x0100
// internal RC oscillator with NO DIVISION, gives 0.2% error rate for 9600 bps UART speed
// for 8Meg : -U lfuse:w:0xe4:m -U hfuse:w:0xdf:m 
#define F_CPU 8000000UL

const char AT[] PROGMEM = { "AT\n\r" };
const char ISOK[] PROGMEM = { "OK" };
const char ISERROR[] PROGMEM = { "ERROR" };
const char ISRING[] PROGMEM = { "RING" };
const char ISREG1[] PROGMEM = { "+CREG: 0,1" };
const char ISREG2[] PROGMEM = { "+CREG: 0,2" };
const char ISREG3[] PROGMEM = { "+CREG: 0,3" };
const char ISSABR[] PROGMEM = { "+SABR" };
const char SHOW_REGISTRATION[] PROGMEM = {"AT+CREG?\n\r"};
const char PIN_IS_READY[] PROGMEM = {"+CPIN: READY"};
const char PIN_MUST_BE_ENTERED[] PROGMEM = {"+CPIN: SIM PIN"};

const char SHOW_PIN[] PROGMEM = {"AT+CPIN?\n\r"};
const char ECHO_OFF[] PROGMEM = {"ATE0\n\r"};
const char ENTER_PIN[] PROGMEM = {"AT+CPIN=\"1111\"\n\r"};
const char HANGUP[] PROGMEM = {"ATH\n\r"};
const char SMS1[] PROGMEM = {"AT+CMGF=1\r\n"};
const char SMS2[] PROGMEM = {"AT+CMGS=\"+48"};   // no ONLY FOR ORANGE PL because it does not +48 in CLIP !
//const char SMS2[] PROGMEM = {"AT+CMGS=\""};    // for other networks if they show +XX in CLIP

const char CRLF[] PROGMEM = {"\"\n\r"};
const char CLIP[] PROGMEM = {"AT+CLIP=1\r\n"};

// Flightmode ON OFF
const char FLIGHTON[] PROGMEM = { "AT+CFUN=4\r\n" };
const char FLIGHTOFF[] PROGMEM = { "AT+CFUN=1\r\n" };


// for sending SMS predefined text 
const char GOOGLELOC1[] PROGMEM = {"\r\n http://maps.google.com/maps?q="};
const char GOOGLELOC2[] PROGMEM = {","};
const char GOOGLELOC3[] PROGMEM = {"\r\n"};
const char LONG[] PROGMEM = {" UTC\n LONG="};
const char LATT[] PROGMEM = {" LATT="};

const char SAPBR1[] PROGMEM = {"AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n"};
const char SAPBR2[] PROGMEM = {"AT+SAPBR=3,1,\"APN\",\"internet\"\r\n"};
const char SAPBR3[] PROGMEM = {"AT+SAPBR=3,1,\"USER\",\"internet\"\r\n"};
const char SAPBR4[] PROGMEM = {"AT+SAPBR=3,1,\"PWD\",\"internet\"\r\n"};
const char SAPBROPEN[] PROGMEM = {"AT+SAPBR=1,1\r\n"};  // open IP bearer
const char SAPBRQUERY[] PROGMEM = {"AT+SAPBR=2,1\r\n"};  // query IP bearer
const char SAPBRCLOSE[] PROGMEM = {"AT+SAPBR=0,1\r\n"};   // close bearer 
const char SAPBRNOTSUCC[] PROGMEM = {"+SAPBR: 1,3,\"0.0.0.0\""}; // bearer was not succesfull
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



// READLINE from serial port that starts with CRLF and ends with CRLF and put to 'response' buffer what read
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

return (i-1);
}



// READ CELL GPS from GPRS APGS function
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


// read PHONE NUMBER from CLIP identification and copy it to buffer for SMS sending
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

// WAIT FOR RESPONSE OK or ERROR - whatever
uint8_t waitforresponse()
{ 
  uint8_t initialized2 = 0;
  char buf2[10];  // buffer to copy string from PROGMEM

               initialized2 = 0; 
               do {  
                if (readline()>0)
                   {
                    memcpy_P(buf2, ISOK, sizeof(ISOK));                     
                   if (is_in_rx_buffer(response, buf2) == 1)  initialized2 = 1; 
                    memcpy_P(buf2, ISERROR, sizeof(ISERROR));                     
                   if (is_in_rx_buffer(response, buf2) == 1)  initialized2 = 0; 
                   };
               } while (initialized2 == 0);
return (initialized2);
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// delay procedures ASM based because _delay_ms() is working bad for 8 MHz clock MCU
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// delay 2 seconds
void delay_2s()
{
// Generated by delay loop calculator
// at http://www.bretmulvey.com/avrdelay.html
//
// Delay 16 000 000 cycles
// 2s at 8.0 MHz

asm volatile (
    "    ldi  r18, 82"	"\n"
    "    ldi  r19, 43"	"\n"
    "    ldi  r20, 0"	"\n"
    "1:  dec  r20"	"\n"
    "    brne 1b"	"\n"
    "    dec  r19"	"\n"
    "    brne 1b"	"\n"
    "    dec  r18"	"\n"
    "    brne 1b"	"\n"
    "    lpm"	"\n"
    "    nop"	"\n"
);
}


// delay 10 seconds
void delay_10s ()
{

asm volatile (
    "    ldi  r18, 2"	"\n"
    "    ldi  r19, 150"	"\n"
    "    ldi  r20, 216"	"\n"
    "    ldi  r21, 9"	"\n"
    "1:  dec  r21"	"\n"
    "    brne 1b"	"\n"
    "    dec  r20"	"\n"
    "    brne 1b"	"\n"
    "    dec  r19"	"\n"
    "    brne 1b"	"\n"
    "    dec  r18"	"\n"
    "    brne 1b"	"\n"
    "    rjmp 1f"	"\n"
    "1:"	"\n"
);
}


//delay 5 seconds

void delay_5s()
{
// Generated by delay loop calculator
// at http://www.bretmulvey.com/avrdelay.html
//
// Delay 40 000 000 cycles
// 5s at 8.0 MHz

asm volatile (
    "    ldi  r18, 203"	"\n"
    "    ldi  r19, 236"	"\n"
    "    ldi  r20, 133"	"\n"
    "1:  dec  r20"	"\n"
    "    brne 1b"	"\n"
    "    dec  r19"	"\n"
    "    brne 1b"	"\n"
    "    dec  r18"	"\n"
    "    brne 1b"	"\n"
    "    nop"	"\n"
);
}


// delay 1 second
void delay_1s()
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
}

// SIM800L initialization procedures


// wait for first AT in case SIM800L is starting up
uint8_t checkat()
{
  uint8_t initialized2;

// wait for first OK while sending AT - autosensing speed on SIM800L, but we are working 9600 bps
// SIM 800L can be set by AT+IPR=9600  to fix this speed
// which I do recommend

                 initialized2 = 0;
              do { 
		delay_2s();
               uart_puts_P(AT);
                if (readline()>0)
                   {
                    memcpy_P(buf, ISOK, sizeof(ISOK));                     
                   if (is_in_rx_buffer(response, buf) == 1)  initialized2 = 1; 
                   // just debug
               // uart_puts_P(response);                   
                   };
               } while (initialized2 == 0);

        // send ECHO OFF
		delay_2s();
                 uart_puts_P(ECHO_OFF);

             return initialized2;
}

// check if PIN is needed and enter PIN 1111 
uint8_t checkpin()
{

  uint8_t initialized2;
     // readline and wait for PIN CODE STATUS if needed send PIN 1111 to SIM card if required
                  initialized2 = 0;
              do { 
		delay_2s();
               uart_puts_P(SHOW_PIN);
                if (readline()>0)
                   {
        	   delay_1s();   

                    memcpy_P(buf, PIN_IS_READY, sizeof(PIN_IS_READY));
                  if (is_in_rx_buffer(response, buf ) == 1)       initialized2 = 1;                                         
                    memcpy_P(buf, PIN_MUST_BE_ENTERED, sizeof(PIN_MUST_BE_ENTERED));
                  if (is_in_rx_buffer(response, buf) == 1)     uart_puts_P(ENTER_PIN);   // ENTER PIN 1111                                      
                    };
                  
              } while (initialized2 == 0);
   return initialized2;
}

// check if registered to the network
uint8_t checkregistration()
{
  uint8_t initialized2, attempt2;
     // readline and wait for STATUS NETWORK REGISTRATION from SIM800L
     // first 3 networks preferred from SIM list are OK
                  initialized2 = 0;
                  attempt2 = 0;
              do { 
		delay_2s();
                 uart_puts_P(SHOW_REGISTRATION);
                if (readline()>0)
                   {			   
                    memcpy_P(buf, ISREG1, sizeof(ISREG1));
                   if (is_in_rx_buffer(response, buf) == 1)  initialized2 = 1; 
                    memcpy_P(buf, ISREG2, sizeof(ISREG2));
                   if (is_in_rx_buffer(response, buf) == 1)  initialized2 = 1; 
                    memcpy_P(buf, ISREG3, sizeof(ISREG3));
                   if (is_in_rx_buffer(response, buf) == 1)  initialized2 = 1; 
                   };
                    // check if we need to restart the RADIO
                if (initialized2 == 0)  // put to flight mode for 10sec and back to register the network
                   { uart_puts_P(FLIGHTON);
                     attempt2 = waitforresponse();
                     delay_10s();
                     uart_puts_P(FLIGHTOFF);
                     attempt2 = waitforresponse();
                     delay_10s();
                     attempt2 = 0;
                   }
                } while (initialized2 == 0);
      return initialized2;
};
 

// provision GPRS APNs and passwords
uint8_t provisiongprs()
{
     // connection to GPRS for AGPS basestation data - provision APN and username
               	delay_1s();
               uart_puts_P(SAPBR1);
		delay_1s();
               uart_puts_P(SAPBR2);
             // only if username password in APN is needed
		delay_1s();
               uart_puts_P(SAPBR3);
		delay_1s();
               uart_puts_P(SAPBR4);
        	delay_1s();   
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
  delay_10s();
           
  // check status of all functions
        initialized = checkat();
        initialized = checkpin();
        initialized = checkregistration();
        initialized = provisiongprs();

 
       // read phone number of incoming voice call by CLIP, will be needed for SMS sending 
		delay_2s();
                uart_puts_P(CLIP); 
		delay_2s();
    

     // neverending LOOP

       while (1) {
     
              // WAIT FOR RING message - incoming voice call, restart RADIO module if no signal
                initialized = 0;
             do { 
                if (readline()>0)
                   {
                    memcpy_P(buf, ISRING, sizeof(ISRING));  
                    if (is_in_rx_buffer(response, buf) == 1) 
                     { initialized = 1; 
                        readphonenumber(); 
                        delay_1s();
                      // hangup a call and proceed with sending SMS                  
                       uart_puts_P(HANGUP); 
                       }
                     // if some other message than RING check if network is avaialble and SIM800L is operational  
                     else 
                      {
                      // check if network is available or there is a need to restart RADIO
                       initialized = checkat();    // maybe SIM800L  restarted itself ?
                      // check status of all functions 
                       initialized = checkpin();
                       initialized = checkregistration();
                       initialized = provisiongprs();
                      // read phone number of incoming voice call by CLIP, will be needed for SMS sending       
                      uart_puts_P(CLIP);
                      delay_2s();
                      initialized = 0;
                      };
                    };
                  } while ( initialized == 0);
               initialized = 0;

           // Create connection to GPRS network - 3 attempts if needed, if not succesfull restart the modem 
           attempt = 0;
           do { 

           // make GPRS network attach and open IP bearer
              delay_2s();
              uart_puts_P(SAPBROPEN);
              initialized = waitforresponse();

           // query PDP context for IP address after several seconds
           // check if GPRS attach was succesfull, do it several times if needed
              delay_5s();
              uart_puts_P(SAPBRQUERY);
              initialized = 0;  
               if (readline()>0)
                   {
                   // checking for not attached
                    memcpy_P(buf, SAPBRNOTSUCC, sizeof(SAPBRNOTSUCC));                     
                   if (is_in_rx_buffer(response, buf) == 1)  initialized = 0;
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
        	delay_2s();
               uart_puts_P(CHECKGPS);
               // parse GPS coordinates from the answer to SMS buffer
               readcellgps();
                // send a SMS in plain text format
               uart_puts_P(SMS1);
               delay_1s(); 
               // compose an SMS from fragments - interactive mode CTRL Z at the end
               uart_puts_P(SMS2);
			   uart_puts(phonenumber);  // send phone number received from CLIP
			   uart_puts_P(CRLF);   			   
               delay_1s(); 
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
               delay_1s(); 
               send_uart(26);   // ctrl Z to end SMS

          //and close the bearer 
              delay_5s();
              uart_puts_P(SAPBRCLOSE);
              attempt = waitforresponse();
          } /// end of commands when GPRS is working

		delay_2s();
           
        // end of neverending loop
        };

 
    // end of MAIN code 
}

