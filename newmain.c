/* 
 * File:   main.c
 * Author: oli4gate
 *
 * Created on 26 février 2015, 17:39
 * wat niet is gecovered in deze software is als the master leest en de slave antwoordt met data
 * in het geval dat de master antwoordt met notacknowledge dan zou de slave de laatste data opnieuw moeten verzenden, dit is niet opgenomen.
 * 
 * acts like and based on CAT24C256 eeprom
 * 
 * general call is not supported by I2C module
 * 
 */


// PIC16F88 Configuration Bit Settings
// 'C' source line config statements
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <pic16f88.h>

//#pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
// CONFIG1
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTRC oscillator; port I/O function on both RA6/OSC2/CLKO pin and RA7/OSC1/CLKI pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is MCLR)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB3/PGM pin has PGM function, Low-Voltage Programming enabled)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off)
#pragma config CCPMX = RB0      // CCP1 Pin Selection bit (CCP1 function on RB0)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// CONFIG2
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode enabled)


#define ELEMENTS  64
#define I2C_SLAVE_ADDRESS 0x46


unsigned int  current_state  = 0;
unsigned int  last_state = 0;
unsigned char stop_bit = 0;
unsigned int  global_call = 0;

unsigned char offset[2]={0x0A,0x0B};            //Start address to read from or write to (2 bytes) 
unsigned char* p_offset = offset;               //pointer to offset array. note: offset is a constant that holds the addrress of the first element of the offset array
unsigned char i2c_buf[ELEMENTS];
unsigned char* p_i2c_buf = &i2c_buf;            //pointer p_i2c_rx_buf holds the address of to the first element of i2c_rx-buf array of unsigned char type

unsigned char i2c_register_addr = 0;
unsigned char read_SSPBUF_to_clear_BF = 0;


void i2c_setup(void)
{
    OSCCON  = 0b01111110;               //8MHz, internal RC for system clock
    ANSEL   = 0b00000000;               //All digital inputs
    TRISA   = 0b11111000;               //porta   
    PORTA   = 0b00000000;
    //timer instellen voor watchdogtimer
    INTCONbits.TMR0IE = 0;              //Timer0 interrupt
    OPTION_REG = 0b00000111;            //Set the TMR0 rate to 1:256 (trigger after 256k ticks)

    PIR1            = 0x00;
    PIE1            = 0x00;
    INTCON          = 0x00;             //Disable all interrupts
    TRISB           = 0b00010010;       //RB1 = SDA + RB4 = SCL - as inputs
    PORTB           = 0b00000000;
    SSPCON          = 0b00110110;       //(SSPEN)SDA+SCL are serial port source pins ; Enable clock ; I2C mode 7-bits address ; Start stoP interrupts disabled    
    PIR1bits.SSPIF  = 0;                //Clear the serial port interrupt flag
    PIE1bits.SSPIE  = 1;                //enable Synchronious Serial Port interrupts
    INTCONbits.PEIE = 1;
    INTCONbits.GIE  = 1;                //enable global and peripheral interrupt
    SSPADD          = I2C_SLAVE_ADDRESS;
}    



/*************************** INITIALIZE ROUTINE *******************************/
void interrupt i2c_int(void)
{
    if (SSPIF)  // check to see if SSP interrupt, an SSP interrupt is generated for each data transfer byte
    {           // the R_nW flag stays valid for the whole message, until the slave sends START,STOP or NACK
                // the D_nA indicates that the last received byte was Data (1) or an address (0)
                // acknowledges are automatically handled by the hardware
        
           if (!SSPSTATbits.R_nW)                               // Master wants to WRITE (R_nW = 0)                                                                
           {
               //State 1: S=1, R_nW=0, D_nA=0, BF=1
               if (!SSPSTATbits.D_nA)                           // Last byte was an address (D_nA = 0)
               {                                   
                   for(char i = 0; i!=2 ;i++)                    //loop to RAZ the offset array, when the address is received we will put the start register address the master wants to start
                   {                                            //reading from or writing to in this array
                       *(p_offset+i)=0x00;                         //put to zero th address offset points to (witch is the offset[] array)                        
                   }
                    p_offset = offset;                          //set offset pointer at first the address of the first element of the offset array to store the start address later on
                    p_i2c_buf = i2c_buf;                        //set pointer at first address location of the array, data storage later on, will start from here (+offset) on.
                    SSPBUF;                                     //dummy read to clear BF flag
               }
               //State 2: S=1, R_nW=0, D_nA=1, BF=1               
               if (SSPSTATbits.D_nA)                            // Last byte was data (D_nA = 1)
               {   
                   if(p_offset < (&offset+2))                   //are we still receiving the offset? if yes store it in the offset array
                   {                                            //because this will be the start address the i2c master wants to read form or write to
                       *p_offset = SSPBUF;                      
                        p_offset++;
                   }
                   else
                   {   
                       if(p_offset == &offset+2) p_i2c_buf += offset[1];     //if the two offset bytes are received,(p_offset will be at &offset plus 2) then point to the start address
                       if(p_i2c_buf  == (&i2c_buf+ELEMENTS)) p_i2c_buf = i2c_buf;               //if we reach the end of the buffer we have to start at the beginning again
                       
                           *p_i2c_buf = SSPBUF;                 //store the data in the location the pointer point starting from element 0 of the array because we exceeded the max buffer length
                           p_i2c_buf++;                         //next array address
                   }
                    
               }
               
               SSPCONbits.CKP = 1;                              // Release CLK, the CLK is held down until the slave is ready again and the CKP is set(clock streching)
           }
                      
           if (SSPSTATbits.R_nW)                                // Master READ (R_nW = 1)
           {
               //State 3: S=1, R_nW=1, D_nA=0               
               if (!SSPSTATbits.D_nA)                           //Received byte was an address, switch to send modus set the pointer to the start address (offset)
               {                     
                    if(p_offset == &offset+2)
                    {
                        p_i2c_buf += offset[1];                                         //point to the first element to be transmitted to the master, this is the value stored in offset[1]                       
                    }                                                                   //else in case of sequential read (didn't receive the dummy write) continue in the offset array from where we were at.    
                    SSPBUF;                                                             //dummy read to clear BF (buffer full) flag
               }
               if (SSPSTATbits.D_nA)                                                    //Received byte was data store it in an array (the master sends the start address to read from)
               {
                    if(p_i2c_buf  == (&i2c_buf+ELEMENTS)) p_i2c_buf = i2c_buf;          //if we reach the end of the buffer we have to start at the beginning again
                        SSPBUF = *p_i2c_buf;                                            //send the data in the location the pointer point starting from element 0 of the array because we exceeded the max buffer length
                        p_i2c_buf++;                                                    //next array address
               }          
               
                if(SSPCONbits.WCOL)                                                     //Write collision occurs when the SSBUF was written to when the previously word was still transmitting
                    {
                        SSPBUF = *p_i2c_buf;                                            //try again to send the data bit in case WCOL occurred
                        SSPCONbits.WCOL = 0;
                    }
               
               SSPCONbits.CKP = 1;                              //the SSPBUF must be written to before the CKP bit can be set, the CLK is held down until CKP is set(clock streching) 
                                                                //in other words, the CKP stays low until the slave is ready preparing tranmit data. 
           }

       SSPIF = 0;           //In case the slave is transmitting data back to the master, this will be the point that the sending wil start. oli4 see table 10-4 in datasheet 16f88
   }
}
/******************************************************************************/



/*************************** MAIN ROUTINE *************************************/
void main(void){

    i2c_setup();
    while (1)                   // main while() loop
    {
        asm("CLRWDT");          // while here just clear WDT and wait for
      
    }         
}
/******************************************************************************/

