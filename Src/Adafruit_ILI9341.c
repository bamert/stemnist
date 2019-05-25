/***************************************************
  This is our library for the Adafruit  ILI9341 Breakout and Shield
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

#include "font/font8x8_basic.h"
#include "Adafruit/Adafruit_ILI9341.h"
#include <inttypes.h>

int _width, _height;
// Enables SPI1 
void ILI9341_SPI1_Init() {
  /* full duplex master, 8 bit transfer, default phase and polarity */
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI;
  /* Disable receive FIFO, it'd complicate things when there is an odd number of bytes to transfer */
  SPI1->CR2 = SPI_CR2_FRXTH;
}
void ILI9341_SPI1_SetSpeed(){
  SPI1->CR1 &=~( SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0); //switch off clock divdivider l
}

// Circumvent STM's slow HAL
void ILI9341_fastSpiSend(uint8_t* dat, int len){ 
  ILI9341_SPI1_SetSpeed();
  while(len--) {
      while(!(SPI1->SR & SPI_SR_TXE)) ;
      *(volatile uint8_t *)&SPI1->DR = *dat++;
      while(!(SPI1->SR & SPI_SR_RXNE)) ;
  }
}
void ILI9341_writecommand(uint8_t c){
  DISP_DC_LOW;
  DISP_CS_LOW;
  ILI9341_fastSpiSend(&c,1);
  DISP_CS_HIGH;
}
void ILI9341_writedata(uint8_t c){
  DISP_DC_HIGH;
  DISP_CS_LOW;
  ILI9341_fastSpiSend(&c,1);
  DISP_CS_HIGH;
}
void ILI9341_Init(){
  _width = ILI9341_TFTWIDTH;
  _height = ILI9341_TFTHEIGHT;
  ILI9341_SPI1_Init();
  ILI9341_writecommand(0xEF);
  ILI9341_writedata(0x03);
  ILI9341_writedata(0x80);
  ILI9341_writedata(0x02);

  ILI9341_writecommand(0xCF);  
  ILI9341_writedata(0x00); 
  ILI9341_writedata(0XC1); 
  ILI9341_writedata(0X30); 

  ILI9341_writecommand(0xED);  
  ILI9341_writedata(0x64); 
  ILI9341_writedata(0x03); 
  ILI9341_writedata(0X12); 
  ILI9341_writedata(0X81); 
 
  ILI9341_writecommand(0xE8);  
  ILI9341_writedata(0x85); 
  ILI9341_writedata(0x00); 
  ILI9341_writedata(0x78); 

  ILI9341_writecommand(0xCB);  
  ILI9341_writedata(0x39); 
  ILI9341_writedata(0x2C); 
  ILI9341_writedata(0x00); 
  ILI9341_writedata(0x34); 
  ILI9341_writedata(0x02); 
 
  ILI9341_writecommand(0xF7);  
  ILI9341_writedata(0x20); 

  ILI9341_writecommand(0xEA);  
  ILI9341_writedata(0x00); 
  ILI9341_writedata(0x00); 
 
  ILI9341_writecommand(ILI9341_PWCTR1);    //Power control 
  ILI9341_writedata(0x23);   //VRH[5:0] 
 
  ILI9341_writecommand(ILI9341_PWCTR2);    //Power control 
  ILI9341_writedata(0x10);   //SAP[2:0];BT[3:0] 
 
  ILI9341_writecommand(ILI9341_VMCTR1);    //VCM control 
  ILI9341_writedata(0x3e); //???????
  ILI9341_writedata(0x28); 
  
  ILI9341_writecommand(ILI9341_VMCTR2);    //VCM control2 
  ILI9341_writedata(0x86);  //--
 
  ILI9341_writecommand(ILI9341_MADCTL);    // Memory Access Control 
  ILI9341_writedata(0x48);

  ILI9341_writecommand(ILI9341_PIXFMT);    
  ILI9341_writedata(0x55); 
  
  ILI9341_writecommand(ILI9341_FRMCTR1);    
  ILI9341_writedata(0x00);  
  ILI9341_writedata(0x18); 
 
  ILI9341_writecommand(ILI9341_DFUNCTR);    // Display Function Control 
  ILI9341_writedata(0x08); 
  ILI9341_writedata(0x82);
  ILI9341_writedata(0x27);  
 
  ILI9341_writecommand(0xF2);    // 3Gamma Function Disable 
  ILI9341_writedata(0x00); 
 
  ILI9341_writecommand(ILI9341_GAMMASET);    //Gamma curve selected 
  ILI9341_writedata(0x01); 
 
  ILI9341_writecommand(ILI9341_GMCTRP1);    //Set Gamma 
  ILI9341_writedata(0x0F); 
  ILI9341_writedata(0x31); 
  ILI9341_writedata(0x2B); 
  ILI9341_writedata(0x0C); 
  ILI9341_writedata(0x0E); 
  ILI9341_writedata(0x08); 
  ILI9341_writedata(0x4E); 
  ILI9341_writedata(0xF1); 
  ILI9341_writedata(0x37); 
  ILI9341_writedata(0x07); 
  ILI9341_writedata(0x10); 
  ILI9341_writedata(0x03); 
  ILI9341_writedata(0x0E); 
  ILI9341_writedata(0x09); 
  ILI9341_writedata(0x00); 
  
  ILI9341_writecommand(ILI9341_GMCTRN1);    //Set Gamma 
  ILI9341_writedata(0x00); 
  ILI9341_writedata(0x0E); 
  ILI9341_writedata(0x14); 
  ILI9341_writedata(0x03); 
  ILI9341_writedata(0x11); 
  ILI9341_writedata(0x07); 
  ILI9341_writedata(0x31); 
  ILI9341_writedata(0xC1); 
  ILI9341_writedata(0x48); 
  ILI9341_writedata(0x08); 
  ILI9341_writedata(0x0F); 
  ILI9341_writedata(0x0C); 
  ILI9341_writedata(0x31); 
  ILI9341_writedata(0x36); 
  ILI9341_writedata(0x0F); 

  ILI9341_writecommand(ILI9341_SLPOUT);    //Exit Sleep 
  HAL_Delay(120);
  ILI9341_writecommand(ILI9341_DISPON);    //Display on 

}
void ILI9341_setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {

  ILI9341_writecommand(ILI9341_CASET); // Column addr set
  DISP_DC_HIGH;
  DISP_CS_LOW;

  uint8_t buf[4];
  buf[0] = x0 >> 8;
  buf[1] = x0 & 0xFF;
  buf[2] = x1 >> 8;
  buf[3] = x1 & 0xFF;
  ILI9341_fastSpiSend(buf,4);

  ILI9341_writecommand(ILI9341_PASET); // Row addr set
  DISP_DC_HIGH;
  DISP_CS_LOW;
  
  buf[0] = y0 >> 8;
  buf[1] = y0 & 0xFF;
  buf[2] = y1 >> 8;
  buf[3] = y1 & 0xFF;
  ILI9341_fastSpiSend(buf,4);
  ILI9341_writecommand(ILI9341_RAMWR); // write to RAM
  DISP_CS_HIGH;
}


void ILI9341_pushColor(uint16_t color) {
  DISP_DC_HIGH;
  DISP_CS_LOW;


  uint8_t buf[2];
  buf[0] = color >> 8;
  buf[1] = color;
  ILI9341_fastSpiSend(buf,2);
  DISP_CS_HIGH;
}

void ILI9341_drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  ILI9341_setAddrWindow(x,y,x+1,y+1);

  DISP_DC_HIGH;
  DISP_CS_LOW;


  uint8_t buf[2];
  buf[0] = color >> 8;
  buf[1] = color;
  ILI9341_fastSpiSend(buf,2);
  DISP_CS_HIGH;
}
void ILI9341_drawGUI(void){
  //Rectangle for drawing area
  //(x,y): (128,8)x(232,8
  for(uint8_t x=8;x<232;x++)
    ILI9341_drawPixel(x,128, ILI9341_BLACK);
  for(uint8_t x=8;x<232;x++)
    ILI9341_drawPixel(x,128, ILI9341_BLACK);
}
void ILI9341_putstr(int x, int y, const char* str){
  //Lower left corner of where we draw a character
  int dx=x;
  int dy=y;
  while(*str != 0x00){

    //Draw 64 pixels
    for(uint8_t cx=0;cx<8;cx++){
      char charline = font8x8_basic[*str][7-cx]; 
      for(uint8_t cy=0;cy<8;cy++){
        if(charline&(1<<(7-cy)))
          ILI9341_drawPixel(dx+cx,dy+7-cy,ILI9341_BLACK);
        else
          ILI9341_drawPixel(dx+cx,dy+7-cy,ILI9341_WHITE);
      }
    }
    dy+=8;
    str++;
  }
}



void ILI9341_drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;

  if((y+h-1) >= _height) 
  {
    h = _height-y;
  }
  
  ILI9341_setAddrWindow(x, y, x, y+h-1);
  uint8_t hi = color >> 8, lo = color;
  DISP_DC_HIGH;
  DISP_CS_LOW;

  while (h--) 
  {
    ILI9341_fastSpiSend(&hi,1);
    ILI9341_fastSpiSend(&lo,1);
  }
  DISP_CS_HIGH;

}


void ILI9341_drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  //if (hwSPI) spi_begin();
  ILI9341_setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  DISP_DC_HIGH;
  DISP_CS_LOW;
  while (w--) {
    ILI9341_fastSpiSend(&hi,1);
    ILI9341_fastSpiSend(&lo,1);
  }
  DISP_CS_HIGH;
}

void ILI9341_fillScreen(uint16_t color) {
  ILI9341_fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void ILI9341_fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {
  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  ILI9341_setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  DISP_DC_HIGH;
  DISP_CS_LOW;
  for(y=h; y>0; y--) 
  {
    for(x=w; x>0; x--)
    {
      ILI9341_fastSpiSend(&hi,1);
      ILI9341_fastSpiSend(&lo,1);
    }
  }
  
  DISP_CS_HIGH;
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t ILI9341_color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


void ILI9341_setRotation(uint8_t m) {

  ILI9341_writecommand(ILI9341_MADCTL);
  int rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     ILI9341_writedata(MADCTL_MX | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
     break;
   case 1:
     ILI9341_writedata(MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  case 2:
    ILI9341_writedata(MADCTL_MY | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
    break;
   case 3:
     ILI9341_writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  }
}


void ILI9341_invertDisplay(uint8_t i) {
  writecommand(i == 1 ? ILI9341_INVON : ILI9341_INVOFF);
}


