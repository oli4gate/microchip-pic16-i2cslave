# microchip-pic16-i2cslave
I2C slave code in c++ and MPLABX for 8-bit PIC16F microcontroller

Hi,

I'm oli4gate, I've discovered PIC microchip from the time I worked at Velleman in 1999 and ever working with it since.
First I started with MikroBasic, but have moved  on to c-c++. Because I also love to work with Linux and because it is build in c I tried to find a coding language that can be commonly used among both. So I ended up with c, c++ that I'm trying to leanr profonldy. 

This project is a I2C slave code for a small 8 bit microcontroller, an 16F88. It is nog tested yet and still in development.
In this code I have tried to us as much pointers as possible, to speed the runtime and to get a better understanding how pointers work.

The code uses a circular buffer with ofsset net with a tail and front pointer.
