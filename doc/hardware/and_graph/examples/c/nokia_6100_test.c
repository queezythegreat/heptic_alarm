/*

  Nokia 6100 Display Test
  Copyright 2005 Thomas Pfeifer (http://thomaspfeifer.net)


  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


  Target: AVR-ATMega8
  Compiler: AVRGcc

  Note: This will only work with Philips-Controller-Displays (brown flexible
  PCB). The ones with Epson-Controller (green PCB) will NOT work.

GND
+5V
BL
CS
CLK
DATA
RESET

*/

//#### CONFIG ####

#define F_CPU 16000000UL  // 8 MHz

#define SPIPORT PORTA
#define SPIDDR DDRA
#define CS 3
#define CLK 4
#define SDA 5
#define RESET 6


#define USR UCSRA
#define UCR UCSRB
#define UBRR UBRRL
#define BAUD_RATE 38400
#define MODE565

//#################


#include <math.h>
#include <avr/io.h>
#include <avr/delay.h>

#define cbi(reg, bit) (reg&=~(1<<bit))
#define sbi(reg, bit) (reg|= (1<<bit))

#define CS0 cbi(SPIPORT,CS);
#define CS1 sbi(SPIPORT,CS);
#define CLK0 cbi(SPIPORT,CLK);
#define CLK1 sbi(SPIPORT,CLK);
#define SDA0 cbi(SPIPORT,SDA);
#define SDA1 sbi(SPIPORT,SDA);
#define RESET0 cbi(SPIPORT,RESET);
#define RESET1 sbi(SPIPORT,RESET);

const unsigned char FONT6x8[97][8];


// Font sizes 
#define SMALL  0 
#define MEDIUM 1 
#define LARGE  2 

// 12-bit color definitions 
#define WHITE    0xFFF 
#define BLACK    0x000 
#define RED    0xF00 
#define GREEN    0x0F0 
#define BLUE    0x00F 
#define CYAN    0x0FF 
#define MAGENTA  0xF0F 
#define YELLOW   0xFF0 
#define BROWN    0xB22 
#define ORANGE   0xFA0 
#define PINK   0xF6A   
 
// Philips PCF8833 LCD controller command codes 
#define NOP    0x00  // nop 
#define SWRESET    0x01  // software reset 
#define BSTROFF    0x02  // booster voltage OFF 
#define BSTRON     0x03  // booster voltage ON 
#define RDDIDIF    0x04  // read display identification 
#define RDDST      0x09  // read display status 
#define SLEEPIN    0x10  // sleep in 
#define SLEEPOUT    0x11  // sleep out 
#define PTLON      0x12  // partial display mode 
#define NORON      0x13  // display normal mode 
#define INVOFF     0x20  // inversion OFF 
#define INVON      0x21  // inversion ON 
#define DALO      0x22  // all pixel OFF 
#define DAL      0x23  // all pixel ON 
#define SETCON   0x25  // write contrast 
#define DISPOFF    0x28  // display OFF 
#define DISPON     0x29  // display ON 
#define CASET      0x2A  // column address set 
#define PASET      0x2B  // page address set 
#define RAMWR      0x2C  // memory write 
#define RGBSET     0x2D  // colour set 
#define PTLAR      0x30  // partial area 
#define VSCRDEF    0x33  // vertical scrolling definition 
#define TEOFF      0x34  // test mode 
#define TEON      0x35  // test mode 
#define MADCTL     0x36  // memory access control 
#define SEP      0x37  // vertical scrolling start address 
#define IDMOFF     0x38  // idle mode OFF 
#define IDMON      0x39  // idle mode ON 
#define COLMOD     0x3A  // interface pixel format 
#define SETVOP     0xB0  // set Vop   
#define BRS      0xB4  // bottom row swap 
#define TRS      0xB6  // top row swap 
#define DISCTR     0xB9  // display control 
#define DOR      0xBA  // data order 
#define TCDFE      0xBD  // enable/disable DF temperature compensation 
#define TCVOPE     0xBF  // enable/disable Vop temp comp 
#define EC      0xC0  // internal or external  oscillator 
#define SETmul     0xC2  // set multiplication factor 
#define TCVOPAB    0xC3  // set TCVOP slopes A and B 
#define TCVOPCD    0xC4  // set TCVOP slopes c and d 
#define TCDF      0xC5  // set divider frequency 
#define DF8COLOR    0xC6  // set divider frequency 8-color mode 
#define SETBS      0xC7  // set bias system 
#define RDTEMP     0xC8  // temperature read back 
#define NLI      0xC9  // n-line inversion 

// Pyramide
#define PYR_A    00,+00,-10
#define PYR_B   -15,-8,+10
#define PYR_C   +15,-8,+10
#define PYR_D   +00,17,+10
#define CNT_PYR  6
const int PYR[CNT_PYR][6]=
//              ^^^ Was CNT_UFO. Too large ;)
       {
		{ PYR_A,PYR_B},
		{ PYR_A,PYR_D},
		{ PYR_A,PYR_C},
		{ PYR_B,PYR_C},
		{ PYR_B,PYR_D},
		{ PYR_D,PYR_C}
	   };
	   
#define byte unsigned char
byte n=0;
byte s1,s2;
byte r,g,b;

void sendCMD(byte cmd);
void sendData(byte cmd);
void shiftBits(byte b);
void setPixel(byte r,byte g,byte b);
void LCDClearScreen(void);
void LCDSetXY(int x, int y); 
void LCDSetPixel(int  x, int  y, int  color); 
void LCDSetLine(int x1, int y1, int x2, int y2, int color); 
void LCDSetRect(int x0, int y0, int x1, int y1, unsigned char fill, int color); 
void LCDSetCircle(int x0, int y0, int radius, int color); 
void LCDPutChar(char c, int  x, int  y, int size, int fcolor, int bcolor); 
void LCDPutString (char *lcd_string, const char *font_style, unsigned char x, unsigned char y, unsigned char fcolor, unsigned char bcolor);

//void RotatingObject(int W1,int W2,int W3, int x_Pos,int y_Pos,const int OBJ[][6],int cnt,int size);

void waitms(int ms) {
  int i;
  for (i=0;i<ms;i++) _delay_ms(1);
}

int main (void) {

  SPIDDR=(1<<SDA)|(1<<CLK)|(1<<CS)|(1<<RESET); //Port-Direction Setup


  //Init Uart and send OK
  UCR = (1<<RXEN)|(1<<TXEN);
  UBRR=(F_CPU / (BAUD_RATE * 16L) - 1);
  loop_until_bit_is_set(USR, UDRE);
  UDR = 'O';
  loop_until_bit_is_set(USR, UDRE);
  UDR = 'K';


  CS0
  SDA0
  CLK1

  RESET1
  RESET0
  RESET1

  CLK1
  SDA1
  CLK1

  waitms(10);

  //Software Reset
  sendCMD(0x01);

  //Sleep Out
  sendCMD(0x11);

  //Booster ON
  sendCMD(0x03);

  waitms(10);

  //Display On
  sendCMD(0x29);

  //Normal display mode
  sendCMD(0x13);

  //Display inversion on
  sendCMD(0x21);

  //Data order
  sendCMD(0xBA);

  //Memory data access control
  sendCMD(0x36);

  sendData(8);   //rgb + MirrorX
 //sendData(8|64);   //rgb + MirrorX
 // sendData(8|128);   //rgb + MirrorY

#ifdef MODE565
  sendCMD(0x3A);
  sendData(5);   //16-Bit per Pixel
#else
  sendCMD(0x3A);
  sendData(3);   //12-Bit per Pixel (default)
#endif


  //Set Constrast
  sendCMD(0x25);
  sendData(55);


LCDClearScreen();

 LCDSetRect(50,0,100,50,1,ORANGE);
 unsigned char i=30;
 
 while (i>5){
 LCDSetCircle(100,100,i,BLACK);
 i = i-5;
 }
 
 LCDPutChar('A', 20, 20, SMALL, RED, BLUE);
 LCDPutChar('N', 20, 26, SMALL, RED, BLUE);
 LCDPutChar('D', 20, 32, 0, RED, BLUE);
 LCDPutChar('-', 20, 38, SMALL, RED, BLUE);
 LCDPutChar('T', 20, 42, SMALL, RED, BLUE);
 LCDPutChar('E', 20, 48, 0, RED, BLUE);
 LCDPutChar('C', 20, 54, SMALL, RED, BLUE);
 LCDPutChar('H', 20, 60, SMALL, RED, BLUE);
 LCDPutChar('.', 20, 66, 0, RED, BLUE);
 LCDPutChar('P', 20, 72, SMALL, RED, BLUE);
 LCDPutChar('L', 20, 78, 0, RED, BLUE);
 //LCDPutChar("Hello World!", 20, 20, SMALL, WHITE, BLACK);
 
 LCDSetLine(30,120,70,90,GREEN);
 LCDSetLine(70,90,30,70,GREEN);
 LCDSetLine(30,70,30,120,GREEN);
}

//********************* DEFINICJE FUNKCJI *******************************

void shiftBits(byte b) {

  CLK0
  if ((b&128)!=0) SDA1 else SDA0
  CLK1

  CLK0
  if ((b&64)!=0) SDA1 else SDA0
  CLK1

  CLK0
  if ((b&32)!=0) SDA1 else SDA0
  CLK1

  CLK0
  if ((b&16)!=0) SDA1 else SDA0
  CLK1

  CLK0
  if ((b&8)!=0) SDA1 else SDA0
  CLK1

  CLK0
  if ((b&4)!=0) SDA1 else SDA0
  CLK1

  CLK0
  if ((b&2)!=0) SDA1 else SDA0
  CLK1

  CLK0
  if ((b&1)!=0) SDA1 else SDA0
  CLK1

}




//send data
void sendData(byte data) {

  CLK0
  SDA1                                                 //1 for param
  CLK1

  shiftBits(data);
}

//send cmd
void sendCMD(byte data) {

  CLK0
  SDA0                                                 //1 for cmd
  CLK1

  shiftBits(data);
}

//converts a 3*8Bit-RGB-Pixel to the 2-Byte-RGBRGB Format of the Display
void setPixel(byte r,byte g,byte b) {
#ifdef MODE565
   sendData((r&248)|g>>5);
   sendData((g&7)<<5|b>>3);
#else
  if (n==0) {
    s1=(r & 240) | (g>>4);
    s2=(b & 240);
    n=1;
  } else {
    n=0;
    sendData(s1);
    sendData(s2|(r>>4));
    sendData((g&240) | (b>>4));
  }
#endif
}

  
void LCDClearScreen(void) { 
  
 long  i;    // loop counter 
 
  // Row address set  (command 0x2B) 
 sendCMD(0x2B); 
 sendData(0); 
 sendData(132); 
 
  // Column address set  (command 0x2A) 
 sendCMD(0x2A); 
 sendData(0); 
 sendData(132); 
 
  // set the display memory to BLACK 
 sendCMD(0x2C); 
  for(i = 0; i < ((132 * 132) / 2); i++) 
  { 
  sendData((BLUE  >> 4) & 0xFF); 
  sendData(((BLUE  & 0xF) << 4) | ((BLUE  >> 8) & 0xF)); 
  sendData(BLUE  & 0xFF);  } 
  } 

  
//  ***************************************************************************** 
//          LCDSetXY.c 
//  
//     Sets the Row and Column addresses 
//      
//     Inputs:    x  =   row address (0 .. 131) 
//        y  =   column address  (0 .. 131) 
// 
// 
//     Returns:   nothing   
// 
//    Author:  James P Lynch     July 7, 2007 
//  *****************************************************************************    
void LCDSetXY(int  x, int  y) { 
  
  // Row address set  (command 0x2B) 
 sendCMD(PASET); 
 sendData(x); 
 sendData(x); 
 
  // Column address set  (command 0x2A) 
 sendCMD(CASET); 
 sendData(y); 
 sendData(y); 
} 


//  ************************************************************************************* 
//          LCDSetPixel.c 
//  
//  Lights a single pixel in the specified color at the specified x and y addresses 
//      
// Inputs:    x     =   row address (0 .. 131) 
//      y     =   column address  (0 .. 131) 
//      color =   12-bit color value  rrrrggggbbbb 
//     rrrr = 1111 full red 
//          : 
//      0000 red is off 
// 
//     gggg = 1111 full green 
//          : 
//      0000 green is off 
// 
//     bbbb = 1111 full blue 
//          : 
//      0000 blue is off 
// 
//     Returns:   nothing  
// 
//    Note: see lcd.h for some sample color settings  
// 
//    Author:  James P Lynch     July 7, 2007 
//  *************************************************************************************  
void LCDSetPixel(int  x, int  y, int  color) { 
 
 LCDSetXY(x, y); 
 sendCMD(RAMWR); 
  sendData((unsigned char)((color >> 4) & 0xFFFF)); 
  sendData((unsigned char)(((color & 0x0F) << 4) | 0x00)); 
 sendCMD(NOP); 
} 
 
 
//  ************************************************************************************************* 
//          LCDSetLine.c 
//  
//  Draws a line in the specified color from (x0,y0) to (x1,y1)  
//      
// Inputs:    x     =   row address (0 .. 131) 
//      y     =   column address  (0 .. 131) 
//      color =   12-bit color value  rrrrggggbbbb 
//     rrrr = 1111 full red 
//          : 
//      0000 red is off 
// 
//     gggg = 1111 full green 
//          : 
//      0000 green is off 
// 
//     bbbb = 1111 full blue 
//          : 
//      0000 blue is off 
// 
//  Returns:   nothing  
// 
//  Note:  good write-up on this algorithm in Wikipedia (search for Bresenham's line algorithm) 
//    see lcd.h for some sample color settings   
// 
//  Authors:   Dr. Leonard McMillan, Associate Professor UNC 
//      Jack Bresenham IBM, Winthrop University (Father of this algorithm, 1962) 
// 
//      Note: taken verbatim from Professor McMillan's presentation:  
//            http://www.cs.unc.edu/~mcmillan/comp136/Lecture6/Lines.html 
// 
//  *************************************************************************************************  
void LCDSetLine(int x0, int y0, int x1, int y1, int color) { 
 
        int dy = y1 - y0; 
        int dx = x1 - x0; 
        int stepx, stepy; 
		 
         
if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; } 
        if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; } 
        dy <<= 1;        // dy is now 2*dy 
        dx <<= 1;        // dx is now 2*dx 
 
  LCDSetPixel(x0, y0, color); 
 
        if (dx > dy) { 
            int fraction = dy - (dx >> 1);  // same as 2*dy - dx 
            while (x0 != x1) { 
                if (fraction >= 0) { 
                    y0 += stepy; 
                    fraction -= dx;    // same as fraction -= 2*dx 
                } 
                x0 += stepx; 
                fraction += dy;    // same as fraction -= 2*dy 
                LCDSetPixel(x0, y0, color); 
            } 
        } else { 
            int fraction = dx - (dy >> 1); 
            while (y0 != y1) { 
                if (fraction >= 0) { 
                    x0 += stepx; 
                    fraction -= dy; 
                } 
                y0 += stepy; 
                fraction += dx; 
                LCDSetPixel(x0, y0, color); 
            } 
        } 
} 

 
 
//  ***************************************************************************************** 
//          LCDSetRect.c 
//  
//  Draws a rectangle in the specified color from (x1,y1) to (x2,y2) 
//  Rectangle can be filled with a color if desired  
//      
// Inputs:    x     =   row address (0 .. 131) 
//      y     =   column address  (0 .. 131) 
//      fill  =   0=no fill, 1-fill entire rectangle  
//      color =   12-bit color value for lines  rrrrggggbbbb 
//     rrrr = 1111 full red 
//      : 
//     0000 red is off 
// 
//     gggg = 1111 full green 
//      : 
//     0000 green is off 
// 
//     bbbb = 1111 full blue 
//      : 
//     0000 blue is off 
//   Returns:   nothing  
// 
//   Notes: 
// 
//    The best way to fill a rectangle is to take advantage of the "wrap-around" featute 
//    built into the Philips PCF8833 controller. By defining a drawing box, the memory can 
//    be simply filled by successive memory writes until all pixels have been illuminated. 
// 
//      1.  Given the coordinates of two opposing corners (x0, y0) (x1, y1) 
//          calculate the minimums and maximums of the coordinates 
// 
//        xmin = (x0 <= x1) ? x0 : x1; 
//        xmax = (x0 > x1) ? x0 : x1; 
//        ymin = (y0 <= y1) ? y0 : y1; 
//        ymax = (y0 > y1) ? y0 : y1; 
//
//      2. Now set up the drawing box to be the desired rectangle 
// 
//     sendCMD(PASET);   // set the row boundaries 
//     sendData(xmin); 
//     sendData(xmax); 
//     sendCMD(CASET);   // set the column boundaries 
//     sendData(ymin); 
//     sendData(ymax); 
// 
//      3. Calculate the number of pixels to be written divided by 2 
// 
//        NumPixels = ((((xmax - xmin + 1) * (ymax - ymin + 1)) / 2) + 1) 
// 
//        You may notice that I added one pixel to the formula.  
//        This covers the case where the number of pixels is odd and we  
//        would lose one pixel due to rounding error. In the case of 
//        odd pixels, the number of pixels is exact.  
//        in the case of even pixels, we have one more pixel than 
//        needed, but it cannot be displayed because it is outside 
//     the drawing box. 
// 
//        We divide by 2 because two pixels are represented by three bytes. 
//        So we work through the rectangle two pixels at a time. 
// 
//      4.  Now a simple memory write loop will fill the rectangle 
// 
//        for (i = 0; i < ((((xmax - xmin + 1) * (ymax - ymin + 1)) / 2) + 1); i++) { 
//      sendData((color >> 4) & 0xFF); 
//          sendData(((color & 0xF) << 4) | ((color >> 8) & 0xF)); 
//      sendData(color & 0xFF); 
//     } 
// 
//    In the case of an unfilled rectangle, drawing four lines with the Bresenham line 
//    drawing algorithm is reasonably efficient. 
// 
//    Author:  James P Lynch      July 7, 2007 
//  *****************************************************************************************  
void LCDSetRect(int x0, int y0, int x1, int y1, unsigned char fill, int color) { 
  int   xmin, xmax, ymin, ymax; 
 int   i; 
  
  // check if the rectangle is to be filled 
  if (fill == 1) { 
   
    // best way to create a filled rectangle is to define a drawing box 
    // and loop two pixels at a time 
   
    // calculate the min and max for x and y directions 
    xmin = (x0 <= x1) ? x0 : x1; 
    xmax = (x0 > x1) ? x0 : x1; 
    ymin = (y0 <= y1) ? y0 : y1; 
    ymax = (y0 > y1) ? y0 : y1; 
   
    // specify the controller drawing box according to those limits 
    // Row address set  (command 0x2B) 
  sendCMD(PASET); 
  sendData(xmin); 
  sendData(xmax); 
     
    // Column address set  (command 0x2A) 
  sendCMD(CASET); 
  sendData(ymin); 
  sendData(ymax); 
   
  // WRITE MEMORY 
  sendCMD(RAMWR); 
   
    // loop on total number of pixels / 2 
    for (i = 0; i < ((((xmax - xmin + 1) * (ymax - ymin + 1)) / 2) + 1); i++) { 
    
      // use the color value to output three data bytes covering two pixels 
      sendData((color >> 4) & 0xFF); 
      sendData(((color & 0xF) << 4) | ((color >> 8) & 0xF)); 
   sendData(color & 0xFF); 
  }
 
  } else { 
      
       // best way to draw un unfilled rectangle is to draw four lines 
    LCDSetLine(x0, y0, x1, y0, color); 
    LCDSetLine(x0, y1, x1, y1, color); 
    LCDSetLine(x0, y0, x0, y1, color); 
    LCDSetLine(x1, y0, x1, y1, color); 
    } 
} 
 
//  ************************************************************************************* 
//          LCDSetCircle.c 
//  
//  Draws a line in the specified color at center (x0,y0) with radius  
//      
// Inputs:    x0     =   row address (0 .. 131) 
//      y0     =   column address  (0 .. 131) 
//      radius =   radius in pixels 
//      color  =   12-bit color value  rrrrggggbbbb 
// 
//  Returns:   nothing   
// 
//  Author:    Jack Bresenham IBM, Winthrop University (Father of this algorithm, 1962) 
// 
//         Note: taken verbatim Wikipedia article on Bresenham's line algorithm   
//          http://www.wikipedia.org 
// 
//  *************************************************************************************  
 
void LCDSetCircle(int x0, int y0, int radius, int color) { 
  int f = 1 - radius; 
  int ddF_x = 0; 
  int ddF_y = -2 * radius; 
  int x = 0; 
  int y = radius; 
  
  LCDSetPixel(x0, y0 + radius, color); 
  LCDSetPixel(x0, y0 - radius, color); 
  LCDSetPixel(x0 + radius, y0, color); 
  LCDSetPixel(x0 - radius, y0, color); 
  
  while(x < y)
 { 
  if(f >= 0)
  { 
	y--; 
	ddF_y += 2; 
	f += ddF_y; 
  } 
  
  x++; 
  ddF_x += 2; 
    f += ddF_x + 1;     
    LCDSetPixel(x0 + x, y0 + y, color); 
    LCDSetPixel(x0 - x, y0 + y, color); 
    LCDSetPixel(x0 + x, y0 - y, color); 
    LCDSetPixel(x0 - x, y0 - y, color); 
    LCDSetPixel(x0 + y, y0 + x, color); 
    LCDSetPixel(x0 - y, y0 + x, color); 
    LCDSetPixel(x0 + y, y0 - x, color); 
    LCDSetPixel(x0 - y, y0 - x, color); 
 } 
}

//  ***************************************************************************** 
//          LCDPutChar.c 
//  
//     Draws an ASCII character at the specified (x,y) address and color 
//      
//     Inputs:    c       =   character to be displayed        
//        x       =   row address (0 .. 131) 
//        y       =   column address  (0 .. 131) 
//        size    =   font pitch (SMALL, MEDIUM, LARGE) 
//             fcolor  =   12-bit foreground color value    rrrrggggbbbb 
//             bcolor  =   12-bit background color value    rrrrggggbbbb 
// 
// 
//     Returns:   nothing   
// 
// 
//        Notes:  Here's an example to display "E" at address (20,20) 
// 
//      LCDPutChar('E', 20, 20, MEDIUM, WHITE, BLACK); 
// 
//            (27,20)        (27,27) 
//          |             | 
//          |             | 
//             ^  V             V 
//             :  _ # # # # # # #   0x7F 
//             :  _ _ # # _ _ _ #   0x31 
//             :  _ _ # # _ # _ _   0x34 
//             x  _ _ # # # # _ _   0x3C 
//             :  _ _ # # _ # _ _   0x34 
//             :  _ _ # # _ _ _ #   0x31 
//             :  _ # # # # # # #   0x7F 
//             :  _ _ _ _ _ _ _ _   0x00 
// 
//      ------y-------> 
//          ^             ^ 
//          |             | 
//          |             | 
//            (20,20)       (20,27) 
// 
// 
//  The most efficient way to display a character is to make use of the "wrap-around" feature 
//  of the Philips PCF8833 LCD controller chip.   
// 
//  Assume that we position the character at (20, 20)  that's a  (row, col) specification. 
//  With the row and column address set commands, you can specify an 8x8 box for the SMALL and MEDIUM 
//  characters or a 16x8 box for the LARGE characters. 
// 
//      sendCMD(PASET);    // set the row drawing limits 
//    sendData(20);    // 
//    sendData(27);    // limit rows to (20, 27) 
//  
//      sendCMD(CASET);    // set the column drawing limits 
//    sendData(20);    //  
//    sendData(27);    // limit columns to (20,27) 
// 
//  When the algorithm completes col 27, the column address wraps back to 20 
//  At the same time, the row address increases by one (this is done by the controller) 
// 
//  We walk through each row, two pixels at a time. The purpose is to create three 
//  data bytes representing these two pixels in the following format (as specified by Philips 
//  for RGB 4 : 4 : 4 format (see page 62 of PCF8833 controller manual). 
// 
//      Data for pixel 0:  RRRRGGGGBBBB 
//      Data for Pixel 1:  RRRRGGGGBBBB 
// 
//      sendCMD(RAMWR);    // start a memory write (96 data bytes to follow) 
// 
//      sendData(RRRRGGGG);    // first pixel, red and green data 
//      sendData(BBBBRRRR);    // first pixel, blue data; second pixel, red data 
//      sendData(GGGGBBBB);    // second pixel, green and blue data 
//     : 
//      and so on until all pixels displayed! 
//     : 
//      sendCMD(NOP);      // this will terminate the RAMWR command 
// 
// 
//    Author:  James P Lynch    July 7, 2007 
//  *****************************************************************************
 
void LCDPutChar(char c, int  x, int  y, int size, int fColor, int bColor) { 
  
  extern const unsigned char FONT6x8[97][8]; 
  //extern const unsigned char FONT8x8[97][8]; 
  //extern const unsigned char FONT8x16[97][16]; 
   
 int    i,j; 
 unsigned int   nCols; 
 unsigned int   nRows; 
 unsigned int   nBytes; 
 unsigned char   PixelRow; 
 unsigned char   Mask; 
 unsigned int   Word0; 
 unsigned int   Word1; 
 unsigned char   *pFont; 
  
 unsigned char   *pChar; 
  unsigned char    *FontTable[] = {(unsigned char *)FONT6x8,  
         //(unsigned char *)FONT8x8,  
         //(unsigned char *)FONT8x16
};  
  
  // get pointer to the beginning of the selected font table 
  pFont = (unsigned char *)FontTable[size];   
  
  // get the nColumns, nRows and nBytes 
  nCols = *pFont; 
  nRows = *(pFont + 1); 
  nBytes = *(pFont + 2); 
 
  // get pointer to the last byte of the desired character 
  pChar = pFont + (nBytes * (c - 0x1F)) + nBytes - 1; 
  
  // Row address set  (command 0x2B) 
 sendCMD(PASET); 
 sendData(x); 
  sendData(x + nRows - 1); 
   
  // Column address set  (command 0x2A) 
 sendCMD(CASET); 
 sendData(y); 
  sendData(y + nCols - 1); 
  
 // WRITE MEMORY 
 sendCMD(RAMWR); 
  
  // loop on each row, working backwards from the bottom to the top 
  for (i = nRows - 1; i >= 0; i--) { 
   
    // copy pixel row from font table and then decrement row 
  PixelRow = *pChar--; 
  
    // loop on each pixel in the row (left to right) 
    // Note: we do two pixels each loop 
  Mask = 0x80; 
    for (j = 0; j < nCols; j += 2) { 
   
      // if pixel bit set, use foreground color; else use the background color 
      // now get the pixel color for two successive pixels 
      if ((PixelRow & Mask) == 0) 
    Word0 = bColor; 
   else 
    Word0 = fColor; 
   Mask = Mask >> 1; 
      if ((PixelRow & Mask) == 0) 
    Word1 = bColor; 
   else 
    Word1 = fColor; 
   Mask = Mask >> 1; 
    
      // use this information to output three data bytes 
      sendData((Word0 >> 4) & 0xFF); 
      sendData(((Word0 & 0xF) << 4) | ((Word1 >> 8) & 0xF)); 
   sendData(Word1 & 0xFF); 
  }   
 } 
  // terminate the Write Memory command 
 sendCMD(NOP);   
}


//  ************************************************************************************************* 
//          LCDPutStr.c 
//  
//     Draws a null-terminates character string at the specified (x,y) address, size and color 
//      
//     Inputs:    pString =   pointer to character string to be displayed        
//        x       =   row address (0 .. 131) 
//        y       =   column address  (0 .. 131) 
//        Size    =   font pitch (SMALL, MEDIUM, LARGE) 
//        fColor  =   12-bit foreground color value   rrrrggggbbbb 
//        bColor  =   12-bit background color value   rrrrggggbbbb 
// 
// 
//     Returns:   nothing   
// 
//     Notes:  Here's an example to display "Hello World!" at address (20,20) 
// 
//          LCDPutChar("Hello World!", 20, 20, LARGE, WHITE, BLACK); 
// 
// 
//    Author:  James P Lynch    July 7, 2007 
//  *************************************************************************************************  
void LCDPutStr(char *pString, int  x, int  y, int Size, int fColor, int bColor) { 
       
  // loop until null-terminator is seen 
  while (*pString != 0x00) { 
 
  // draw the character 
    LCDPutChar(*pString++, x, y, Size, fColor, bColor);   
 
    // advance the y position 
  if (Size == SMALL) 
      y = y + 6; 
  else if (Size == MEDIUM) 
      y = y + 8; 
  else 
      y = y + 8; 
   
    // bail out if y exceeds 131 
    if (y > 131) break; 
 } 
}  

//  ********************************************************************************* 
//    Font tables for Nokia 6610 LCD Display Driver  (PCF8833 Controller) 
// 
//   FONT6x8    -  SMALL font (mostly 5x7) 
//   FONT8x8    -  MEDIUM font (8x8 characters, a bit thicker) 
//    FONT8x16  -  LARGE font  (8x16 characters, thicker) 
//                                                                        
//    Note:   ASCII characters 0x00 through 0x1F are not included in these fonts. 
//      First row of each font contains the number of columns, the 
//      number of rows and the number of bytes per character. 
// 
//  Author:  Jim Parise, James P Lynch  July 7, 2007 
//  *********************************************************************************  
const unsigned char FONT6x8[97][8] = 
{0x06,0x08,0x08,0x00,0x00,0x00,0x00,0x00,  //  columns, rows, num_bytes_per_char 
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //  space  0x20 
0x20,0x20,0x20,0x20,0x20,0x00,0x20,0x00, //  ! 
0x50,0x50,0x50,0x00,0x00,0x00,0x00,0x00, //  " 
0x50,0x50,0xF8,0x50,0xF8,0x50,0x50,0x00, //  #   
0x20,0x78,0xA0,0x70,0x28,0xF0,0x20,0x00, //  $ 
0xC0,0xC8,0x10,0x20,0x40,0x98,0x18,0x00, //  % 
0x40,0xA0,0xA0,0x40,0xA8,0x90,0x68,0x00, //  & 
0x30,0x30,0x20,0x40,0x00,0x00,0x00,0x00, //  ' 
0x10,0x20,0x40,0x40,0x40,0x20,0x10,0x00, //  ( 
0x40,0x20,0x10,0x10,0x10,0x20,0x40,0x00, //  ) 
0x00,0x20,0xA8,0x70,0x70,0xA8,0x20,0x00, //  * 
0x00,0x20,0x20,0xF8,0x20,0x20,0x00,0x00, //  + 
0x00,0x00,0x00,0x00,0x30,0x30,0x20,0x40, //  , 
0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00, //  - 
0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00, //  . 
0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,  //  / (forward slash) 
0x70,0x88,0x88,0xA8,0x88,0x88,0x70,0x00,  //  0  0x30 
0x20,0x60,0x20,0x20,0x20,0x20,0x70,0x00, //  1 
0x70,0x88,0x08,0x70,0x80,0x80,0xF8,0x00, //  2 
0xF8,0x08,0x10,0x30,0x08,0x88,0x70,0x00, //  3 
0x10,0x30,0x50,0x90,0xF8,0x10,0x10,0x00, //  4 
0xF8,0x80,0xF0,0x08,0x08,0x88,0x70,0x00, //  5 
0x38,0x40,0x80,0xF0,0x88,0x88,0x70,0x00, //  6 
0xF8,0x08,0x08,0x10,0x20,0x40,0x80,0x00, //  7 
0x70,0x88,0x88,0x70,0x88,0x88,0x70,0x00, //  8 
0x70,0x88,0x88,0x78,0x08,0x10,0xE0,0x00, //  9 
0x00,0x00,0x20,0x00,0x20,0x00,0x00,0x00, //  : 
0x00,0x00,0x20,0x00,0x20,0x20,0x40,0x00, //  ; 
0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x00, //  < 
0x00,0x00,0xF8,0x00,0xF8,0x00,0x00,0x00, //  = 
0x40,0x20,0x10,0x08,0x10,0x20,0x40,0x00, //  > 
0x70,0x88,0x08,0x30,0x20,0x00,0x20,0x00, //  ? 
0x70,0x88,0xA8,0xB8,0xB0,0x80,0x78,0x00,  //  @  0x40 
0x20,0x50,0x88,0x88,0xF8,0x88,0x88,0x00, //  A 
0xF0,0x88,0x88,0xF0,0x88,0x88,0xF0,0x00, //  B 
0x70,0x88,0x80,0x80,0x80,0x88,0x70,0x00, //  C 
0xF0,0x88,0x88,0x88,0x88,0x88,0xF0,0x00, //  D 
0xF8,0x80,0x80,0xF0,0x80,0x80,0xF8,0x00, //  E 
0xF8,0x80,0x80,0xF0,0x80,0x80,0x80,0x00, //  F 
0x78,0x88,0x80,0x80,0x98,0x88,0x78,0x00, //  G 
0x88,0x88,0x88,0xF8,0x88,0x88,0x88,0x00, //  H 
0x70,0x20,0x20,0x20,0x20,0x20,0x70,0x00, //  I 
0x38,0x10,0x10,0x10,0x10,0x90,0x60,0x00, //  J 
0x88,0x90,0xA0,0xC0,0xA0,0x90,0x88,0x00, //  K 
0x80,0x80,0x80,0x80,0x80,0x80,0xF8,0x00, //  L 
0x88,0xD8,0xA8,0xA8,0xA8,0x88,0x88,0x00, //  M 
0x88,0x88,0xC8,0xA8,0x98,0x88,0x88,0x00, //  N 
0x70,0x88,0x88,0x88,0x88,0x88,0x70,0x00, //  O 
0xF0,0x88,0x88,0xF0,0x80,0x80,0x80,0x00,  //  P  0x50 
0x70,0x88,0x88,0x88,0xA8,0x90,0x68,0x00, //  Q 
0xF0,0x88,0x88,0xF0,0xA0,0x90,0x88,0x00, //  R 
0x70,0x88,0x80,0x70,0x08,0x88,0x70,0x00, //  S 
0xF8,0xA8,0x20,0x20,0x20,0x20,0x20,0x00, //  T 
0x88,0x88,0x88,0x88,0x88,0x88,0x70,0x00, //  U 
0x88,0x88,0x88,0x88,0x88,0x50,0x20,0x00, //  V 
0x88,0x88,0x88,0xA8,0xA8,0xA8,0x50,0x00, //  W 
0x88,0x88,0x50,0x20,0x50,0x88,0x88,0x00, //  X 
0x88,0x88,0x50,0x20,0x20,0x20,0x20,0x00, //  Y 
0xF8,0x08,0x10,0x70,0x40,0x80,0xF8,0x00, //  Z 
0x78,0x40,0x40,0x40,0x40,0x40,0x78,0x00, //  [ 
0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,  //  \  (back slash) 
0x78,0x08,0x08,0x08,0x08,0x08,0x78,0x00, //  ]   
0x20,0x50,0x88,0x00,0x00,0x00,0x00,0x00, //  ^ 
0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x00, //  _ 
0x60,0x60,0x20,0x10,0x00,0x00,0x00,0x00,  //  `  0x60 
0x00,0x00,0x60,0x10,0x70,0x90,0x78,0x00, //  a 
0x80,0x80,0xB0,0xC8,0x88,0xC8,0xB0,0x00, //  b 
0x00,0x00,0x70,0x88,0x80,0x88,0x70,0x00, //  c 
0x08,0x08,0x68,0x98,0x88,0x98,0x68,0x00, //  d 
0x00,0x00,0x70,0x88,0xF8,0x80,0x70,0x00, //  e 
0x10,0x28,0x20,0x70,0x20,0x20,0x20,0x00, //  f 
0x00,0x00,0x70,0x98,0x98,0x68,0x08,0x70, //  g 
0x80,0x80,0xB0,0xC8,0x88,0x88,0x88,0x00, //  h 
0x20,0x00,0x60,0x20,0x20,0x20,0x70,0x00, //  i 
0x10,0x00,0x10,0x10,0x10,0x90,0x60,0x00, //  j 
0x80,0x80,0x90,0xA0,0xC0,0xA0,0x90,0x00, //  k 
0x60,0x20,0x20,0x20,0x20,0x20,0x70,0x00, //  l 
0x00,0x00,0xD0,0xA8,0xA8,0xA8,0xA8,0x00, //  m 
0x00,0x00,0xB0,0xC8,0x88,0x88,0x88,0x00, //  n 
0x00,0x00,0x70,0x88,0x88,0x88,0x70,0x00, //  o 
0x00,0x00,0xB0,0xC8,0xC8,0xB0,0x80,0x80,  //  p  0x70 
0x00,0x00,0x68,0x98,0x98,0x68,0x08,0x08, //  q 
0x00,0x00,0xB0,0xC8,0x80,0x80,0x80,0x00, //  r 
0x00,0x00,0x78,0x80,0x70,0x08,0xF0,0x00, //  s 
0x20,0x20,0xF8,0x20,0x20,0x28,0x10,0x00, //  t 
0x00,0x00,0x88,0x88,0x88,0x98,0x68,0x00, //  u 
0x00,0x00,0x88,0x88,0x88,0x50,0x20,0x00, //  v 
0x00,0x00,0x88,0x88,0xA8,0xA8,0x50,0x00, //  w 
0x00,0x00,0x88,0x50,0x20,0x50,0x88,0x00, //  x 
0x00,0x00,0x88,0x88,0x78,0x08,0x88,0x70, //  y 
0x00,0x00,0xF8,0x10,0x20,0x40,0xF8,0x00, //  z 
0x10,0x20,0x20,0x40,0x20,0x20,0x10,0x00, //  { 
0x20,0x20,0x20,0x00,0x20,0x20,0x20,0x00, //  | 
0x40,0x20,0x20,0x10,0x20,0x20,0x40,0x00, //  } 
0x40,0xA8,0x10,0x00,0x00,0x00,0x00,0x00, //  ~ 
0x70,0xD8,0xD8,0x70,0x00,0x00,0x00,0x00}; //  DEL 

/*
#define OFFSETX 64              	// offset for screen wont change unless
#define OFFSETY 64              	// i use different screen! so its kinda fixed
#define OFFSETZ 30

const signed int aa[8]={10,-10,-10,10,   10,-10,-10,10};	// x data for shape vertex
const signed int bb[8]={10,10,-10,-10,   10,10,-10,-10};	// y data for shape vertex
const signed int cc[8]={-10,-10,-10,-10, 10,10,10,10};		// z data for shape vertex

const int ff[12]={1,2,3,4,  5,6,7,8, 1,2,3,4};    		// start vertex for lines
const int gg[12]={2,3,4,1,  6,7,8,5, 5,6,7,8};			// end vertex for lines

int sx,sy,ex,ey;    

void cube()                     	// routine to draw and calc 3d cube
 {
 int newx[8];                   	// translated screen x co-ordinates for vertex
 int newy[8];                   	// translated screen y co-ordinates for vertex
 int i,loop;                    	// temp variable for loops
 float xt,yt,zt,x,y,z,sinax,cosax,sinay,cosay,sinaz,cosaz,vertex;  // lots of work variables
 float xpos=0;				// position for object in 3d space, in x
 float ypos=0;				// y
 float zpos=0;				// and z values
 float rotx=0;                  	// starting amount of x rotation
 float roty=0;                 		// starting amount of y rotation
 float rotz=0;                		// starting amount of z rotation

 for (loop=0; loop<=100; loop++)	// rotate the cube 100 times
  {
  xpos=xpos+0.0;			// move the object
  ypos=ypos+0.0;			// it would wander off screen
  zpos=zpos+0.0;			// really quick, so leave it centered

  rotx=rotx+0.5;                	// rotate the cube on X axis
  roty=roty+0.5;                	// and on its y axis
  rotz=rotz+0.0;                	// dont bother with z or it gets confusing

  sinax=sin(rotx);			// precalculate the sin and cos values
  cosax=cos(rotx);			// for the rotation as this saves a 
  
  sinay=sin(roty);			// little time when running as we
  cosay=cos(roty);			// call sin and cos less often
  
  sinaz=sin(rotz);			// they are slow routines
  cosaz=cos(rotz);			// and we dont want slow!

  for (i=0; i<8; i++)           	// translate 3d vertex position to 2d screen position
        {
        x=aa[i];                	// get x for vertex i
        y=bb[i];                	// get y for vertex i
        z=cc[i];                	// get z for vertex i

        yt = y * cosax - z * sinax;	// rotate around the x axis
        zt = y * sinax + z * cosax;	// using the Y and Z for the rotation
        y = yt;
        z = zt;

        xt = x * cosay - z * sinay;	// rotate around the Y axis
        zt = x * sinay + z * cosay;	// using X and Z
        x = xt;
        z = zt;

        xt = x * cosaz - y * sinaz;	// finaly rotate around the Z axis
        yt = x * sinaz + y * cosaz;	// using X and Y
        x = xt;
        y = yt;

        x=x+xpos;			// add the object position offset
        y=y+ypos;			// for both x and y
        z=z+OFFSETZ-zpos;		// as well as Z

        newx[i]=(x*64/z)+OFFSETX;	// translate 3d to 2d coordinates for screen
        newy[i]=(y*64/z)+OFFSETY;	// drawing so we can see the cube
        }

  delay_ms(20);			// delay for a while to allow looking at the cube
  clearscreen();		// clear the screen to remove old cube

  for (i=0; i<12; i++)		// draw the lines that make up the object
        {
        vertex=ff[i]-1;         // temp = start vertex for this line
        sx=newx[vertex];        // set line start x to vertex[i] x position
        sy=newy[vertex];        // set line start y to vertex[i] y position
        vertex=gg[i]-1;         // temp = end vertex for this line
        ex=newx[vertex];	// set line end x to vertex[i+1] x position
        ey=newy[vertex];	// set line end y to vertex[i+1] y position
        LCDSetLine(sx,sy,ex,ey,BLACK);
		//lcdline();		// draw the line between these 2 vertex
        }
  }
 }
*/