/***************************************************
  This is our library for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_ILI9341.h"
//#include <avr/pgmspace.h>
#include "stm32l4xx_hal.h"
//#include <limits.h>
//#include "pins_arduino.h"
//#include "wiring_private.h"
//#include <SPI.h>

// Digital 10 Display CS (PA2)
#define DISP_CS_LOW HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
#define DISP_CS_HIGH HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
// Digital 9 Display DC (PA15)
#define DISP_DC_LOW HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
#define DISP_DC_HIGH CSDISP_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

// Constructor when using software SPI.  All output pins are configurable.

void Adafruit_ILI9341::writecommand(uint8_t c) {
  //*dcport &=  ~dcpinmask;
  //*csport &= ~cspinmask;
  DISP_DC_LOW;
  DISP_CS_LOW;
  SPI.write(c);
  //*csport |= cspinmask;
  DISP_CS_HIGH;
}


void Adafruit_ILI9341::writedata(uint8_t c) {
  //*dcport |=  dcpinmask;
  //*csport &= ~cspinmask;
  DISP_DC_HIGH;
  DISP_CS_LOW;
  SPI.write(c);
  //*csport |= cspinmask;
  DISP_CS_LOW;
} 

// If the SPI library has transaction support, these functions
// establish settings and protect from interference from other
// libraries.  Otherwise, they simply do nothing.
/*#ifdef SPI_HAS_TRANSACTION
static inline void spi_begin(void) __attribute__((always_inline));
static inline void spi_begin(void) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
}
static inline void spi_end(void) __attribute__((always_inline));
static inline void spi_end(void) {
  SPI.endTransaction();
}
#else*/
#define spi_begin()
#define spi_end()
//#endif

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.


void ILI9341_init(void) {
  /*if (_rst > 0) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
  }*/

  // toggle RST low to reset
 /* if (_rst > 0) {
    digitalWrite(_rst, HIGH);
    delay(5);
    digitalWrite(_rst, LOW);
    delay(20);
    digitalWrite(_rst, HIGH);
    delay(150);
  }*/

  
  //if (hwSPI) spi_begin();
  writecommand(0xEF);
  writedata(0x03);
  writedata(0x80);
  writedata(0x02);

  writecommand(0xCF);  
  writedata(0x00); 
  writedata(0XC1); 
  writedata(0X30); 

  writecommand(0xED);  
  writedata(0x64); 
  writedata(0x03); 
  writedata(0X12); 
  writedata(0X81); 
 
  writecommand(0xE8);  
  writedata(0x85); 
  writedata(0x00); 
  writedata(0x78); 

  writecommand(0xCB);  
  writedata(0x39); 
  writedata(0x2C); 
  writedata(0x00); 
  writedata(0x34); 
  writedata(0x02); 
 
  writecommand(0xF7);  
  writedata(0x20); 

  writecommand(0xEA);  
  writedata(0x00); 
  writedata(0x00); 
 
  writecommand(ILI9341_PWCTR1);    //Power control 
  writedata(0x23);   //VRH[5:0] 
 
  writecommand(ILI9341_PWCTR2);    //Power control 
  writedata(0x10);   //SAP[2:0];BT[3:0] 
 
  writecommand(ILI9341_VMCTR1);    //VCM control 
  writedata(0x3e); //???????
  writedata(0x28); 
  
  writecommand(ILI9341_VMCTR2);    //VCM control2 
  writedata(0x86);  //--
 
  writecommand(ILI9341_MADCTL);    // Memory Access Control 
  writedata(0x48);

  writecommand(ILI9341_PIXFMT);    
  writedata(0x55); 
  
  writecommand(ILI9341_FRMCTR1);    
  writedata(0x00);  
  writedata(0x18); 
 
  writecommand(ILI9341_DFUNCTR);    // Display Function Control 
  writedata(0x08); 
  writedata(0x82);
  writedata(0x27);  
 
  writecommand(0xF2);    // 3Gamma Function Disable 
  writedata(0x00); 
 
  writecommand(ILI9341_GAMMASET);    //Gamma curve selected 
  writedata(0x01); 
 
  writecommand(ILI9341_GMCTRP1);    //Set Gamma 
  writedata(0x0F); 
  writedata(0x31); 
  writedata(0x2B); 
  writedata(0x0C); 
  writedata(0x0E); 
  writedata(0x08); 
  writedata(0x4E); 
  writedata(0xF1); 
  writedata(0x37); 
  writedata(0x07); 
  writedata(0x10); 
  writedata(0x03); 
  writedata(0x0E); 
  writedata(0x09); 
  writedata(0x00); 
  
  writecommand(ILI9341_GMCTRN1);    //Set Gamma 
  writedata(0x00); 
  writedata(0x0E); 
  writedata(0x14); 
  writedata(0x03); 
  writedata(0x11); 
  writedata(0x07); 
  writedata(0x31); 
  writedata(0xC1); 
  writedata(0x48); 
  writedata(0x08); 
  writedata(0x0F); 
  writedata(0x0C); 
  writedata(0x31); 
  writedata(0x36); 
  writedata(0x0F); 

  writecommand(ILI9341_SLPOUT);    //Exit Sleep 
  //if (hwSPI) spi_end();
  delay(120); 		
  //if (hwSPI) spi_begin();
  writecommand(ILI9341_DISPON);    //Display on 
  //if (hwSPI) spi_end();

}


void ILI9341_setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {

  writecommand(ILI9341_CASET); // Column addr set
  DISP_DC_HIGH;
  DISP_CS_LOW;
  //*dcport |=  dcpinmask;
  //*csport &= ~cspinmask;
  SPI.write(x0 >> 8);
  SPI.write(x0 & 0xFF);     // XSTART 
  SPI.write(x1 >> 8);
  SPI.write(x1 & 0xFF);     // XEND

  writecommand(ILI9341_PASET); // Row addr set
  //*dcport |=  dcpinmask;
  //*csport &= ~cspinmask;
  DISP_DC_HIGH;
  DISP_CS_LOW;
  SPI.write(y0>>8);
  SPI.write(y0);     // YSTART
  SPI.write(y1>>8);
  SPI.write(y1);     // YEND

  writecommand(ILI9341_RAMWR); // write to RAM
}


void ILI9341_pushColor(uint16_t color) {
  //if (hwSPI) spi_begin();

  //*dcport |=  dcpinmask;
  //*csport &= ~cspinmask;
  DISP_DC_HIGH;
  DISP_CS_LOW;

  SPI.write(color >> 8);
  SPI.write(color);

  //*csport |= cspinmask;
  DISP_CS_HIGH;
  //digitalWrite(_cs, HIGH);
  //if (hwSPI) spi_end();
}

void ILI9341_drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  //if (hwSPI) spi_begin();
  setAddrWindow(x,y,x+1,y+1);

  //*dcport |=  dcpinmask;
  //*csport &= ~cspinmask;
  DISP_DC_HIGH;
  DISP_CS_LOW;

  SPI.write(color >> 8);
  SPI.write(color);

  //*csport |= cspinmask;
  DISP_CS_HIGH;
  //if (hwSPI) spi_end();
}


void ILI9341_drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;

  if((y+h-1) >= _height) 
  {
    h = _height-y;
  }
  
  //if (hwSPI) spi_begin();
  setAddrWindow(x, y, x, y+h-1);
  uint8_t hi = color >> 8, lo = color;
  //*dcport |=  dcpinmask;
  //*csport &= ~cspinmask;
  DISP_DC_HIGH;
  DISP_CS_LOW;

  while (h--) 
  {
    SPI.write(hi);
    SPI.write(lo);
  }
  *csport |= cspinmask;

  //if (hwSPI) spi_end();
}


void ILI9341_drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  //if (hwSPI) spi_begin();
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  //*dcport |=  dcpinmask;
  //*csport &= ~cspinmask;
  DISP_DC_HIGH;
  DISP_CS_LOW;
  while (w--) {
    SPI.write(hi);
    SPI.write(lo);
  }
  //*csport |= cspinmask;
  DISP_CS_HIGH;
  //if (hwSPI) spi_end();
}

void ILI9341_fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void ILI9341_fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {
	int numPixels;
  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  //if (hwSPI) spi_begin();
  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  //*dcport |=  dcpinmask;
  //*csport &= ~cspinmask;
  DISP_DC_HIGH;
  DISP_CS_LOW;
  for(y=h; y>0; y--) 
  {
    for(x=w; x>0; x--)
	{
      SPI.write(hi);
      SPI.write(lo);
    }
  }
  
  //if (hwSPI) spi_end();
  //*csport |= cspinmask;
  DISP_CS_HIGH;
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t ILI9341_color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void ILI9341_setRotation(uint8_t m) {

  //if (hwSPI) spi_begin();
  writecommand(ILI9341_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(MADCTL_MX | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
     break;
   case 1:
     writedata(MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  case 2:
    writedata(MADCTL_MY | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
    break;
   case 3:
     writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  }
  //if (hwSPI) spi_end();
}


void ILI9341_invertDisplay(boolean i) {
  //if (hwSPI) spi_begin();
  writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
  //if (hwSPI) spi_end();
}



