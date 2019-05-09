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

#ifndef _ADAFRUIT_ILI9341H_
#define _ADAFRUIT_ILI9341H_

// Digital 10 Display CS (PA2)
#define DISP_CS_LOW HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
#define DISP_CS_HIGH HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
// Digital 9 Display DC (PA15)
#define DISP_DC_LOW HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
#define DISP_DC_HIGH HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
/*
#define ILI9341_PWCTR6  0xFC

*/

// Color definitions
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

class ILI9341  {
  private:
    SPI_HandleTypeDef* hspi1;
    int _width, _height;
    // Enables SPI1 
    void SPI1_Init() {
      //SPI1->CR1 &= ~SPI_CR1_SPE;
      /* full duplex master, 8 bit transfer, default phase and polarity */
      SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI;
      /* Disable receive FIFO, it'd complicate things when there is an odd number of bytes to transfer */
      SPI1->CR2 = SPI_CR2_FRXTH;
    }
    /*void SPI1_Transfer(uint8_t *outp, uint8_t *inp, int count) {
      while(count--) {
          while(!(SPI1->SR & SPI_SR_TXE))
              ;
          *(volatile uint8_t *)&SPI1->DR = *outp++;
          while(!(SPI1->SR & SPI_SR_RXNE))
              ;
          *inp++ = *(volatile uint8_t *)&SPI1->DR;
      }
    }*/

    // Circumvent STM's slow HAL
    void fastSpiSend(uint8_t* dat, int len){ 
      SPI1_Init();
      while(len--) {
          while(!(SPI1->SR & SPI_SR_TXE)) ;
          *(volatile uint8_t *)&SPI1->DR = *dat++;
          while(!(SPI1->SR & SPI_SR_RXNE)) ;
      }
    }
  public:
    ILI9341(SPI_HandleTypeDef* hspi1) :hspi1(hspi1){
      _width = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      SPI1_Init();
    }
    void writecommand(uint8_t c){
      DISP_DC_LOW;
      DISP_CS_LOW;
      //HAL_SPI_Transmit(hspi1, &c, 1, 10);
      fastSpiSend(&c,1);
      DISP_CS_HIGH;
    }
    void writedata(uint8_t c){
      DISP_DC_HIGH;
      DISP_CS_LOW;
      //HAL_SPI_Transmit(hspi1, &c, 1, 10);
      fastSpiSend(&c,1);
      DISP_CS_HIGH;
    }
    void init(){
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
      HAL_Delay(120);
      writecommand(ILI9341_DISPON);    //Display on 

    }
    void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
     uint16_t y1) {

      writecommand(ILI9341_CASET); // Column addr set
      DISP_DC_HIGH;
      DISP_CS_LOW;

      uint8_t buf[4];
      buf[0] = x0 >> 8;
      buf[1] = x0 & 0xFF;
      buf[2] = x1 >> 8;
      buf[3] = x1 & 0xFF;
      //HAL_SPI_Transmit(hspi1, buf, 4, HAL_MAX_DELAY);
      fastSpiSend(buf,4);

      writecommand(ILI9341_PASET); // Row addr set
      DISP_DC_HIGH;
      DISP_CS_LOW;
      
      buf[0] = y0 >> 8;
      buf[1] = y0 & 0xFF;
      buf[2] = y1 >> 8;
      buf[3] = y1 & 0xFF;
      //HAL_SPI_Transmit(hspi1, buf, 4, HAL_MAX_DELAY);
      fastSpiSend(buf,4);
      writecommand(ILI9341_RAMWR); // write to RAM
      DISP_CS_HIGH;
    }


    void pushColor(uint16_t color) {
      DISP_DC_HIGH;
      DISP_CS_LOW;


      uint8_t buf[2];
      buf[0] = color >> 8;
      buf[1] = color;
      //HAL_SPI_Transmit(hspi1, buf, 2, HAL_MAX_DELAY);
      fastSpiSend(buf,2);
      DISP_CS_HIGH;
    }

    void drawPixel(int16_t x, int16_t y, uint16_t color) {

      if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

      setAddrWindow(x,y,x+1,y+1);

      DISP_DC_HIGH;
      DISP_CS_LOW;


      uint8_t buf[2];
      buf[0] = color >> 8;
      buf[1] = color;
      //HAL_SPI_Transmit(hspi1, buf, 2, HAL_MAX_DELAY);
      fastSpiSend(buf,2);
      DISP_CS_HIGH;
    }
    void drawGUI(void){
      //Rectangle for drawing area
      //(x,y): (128,8)x(232,8
      for(uint8_t x=8;x<232;x++)
        drawPixel(x,128, ILI9341_BLACK);
      for(uint8_t x=8;x<232;x++)
        drawPixel(x,128, ILI9341_BLACK);
      

    }


    void drawFastVLine(int16_t x, int16_t y, int16_t h,
     uint16_t color) {

      // Rudimentary clipping
      if((x >= _width) || (y >= _height)) return;

      if((y+h-1) >= _height) 
      {
        h = _height-y;
      }
      
      setAddrWindow(x, y, x, y+h-1);
      uint8_t hi = color >> 8, lo = color;
      DISP_DC_HIGH;
      DISP_CS_LOW;

      while (h--) 
      {
        //HAL_SPI_Transmit(hspi1, &hi, 1, HAL_MAX_DELAY);
        //HAL_SPI_Transmit(hspi1, &lo, 1, HAL_MAX_DELAY);
        fastSpiSend(&hi,1);
        fastSpiSend(&lo,1);
      }
      DISP_CS_HIGH;

    }


    void drawFastHLine(int16_t x, int16_t y, int16_t w,
      uint16_t color) {

      // Rudimentary clipping
      if((x >= _width) || (y >= _height)) return;
      if((x+w-1) >= _width)  w = _width-x;
      //if (hwSPI) spi_begin();
      setAddrWindow(x, y, x+w-1, y);

      uint8_t hi = color >> 8, lo = color;
      DISP_DC_HIGH;
      DISP_CS_LOW;
      while (w--) {
        //HAL_SPI_Transmit(hspi1, &hi, 1, HAL_MAX_DELAY);
        //HAL_SPI_Transmit(hspi1, &lo, 1, HAL_MAX_DELAY);
        fastSpiSend(&hi,1);
        fastSpiSend(&lo,1);
      }
      DISP_CS_HIGH;
    }

    void fillScreen(uint16_t color) {
      fillRect(0, 0,  _width, _height, color);
    }

    // fill a rectangle
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
      uint16_t color) {
      int numPixels;
      // rudimentary clipping (drawChar w/big text requires this)
      if((x >= _width) || (y >= _height)) return;
      if((x + w - 1) >= _width)  w = _width  - x;
      if((y + h - 1) >= _height) h = _height - y;

      setAddrWindow(x, y, x+w-1, y+h-1);

      uint8_t hi = color >> 8, lo = color;

      DISP_DC_HIGH;
      DISP_CS_LOW;
      for(y=h; y>0; y--) 
      {
        for(x=w; x>0; x--)
        {
          //HAL_SPI_Transmit(hspi1, &hi, 1, 1);//$HAL_MAX_DELAY);
          //HAL_SPI_Transmit(hspi1, &lo, 1, 1);//$HAL_MAX_DELAY);
          fastSpiSend(&hi,1);
          fastSpiSend(&lo,1);
        }
      }
      
      DISP_CS_HIGH;
    }


    // Pass 8-bit (each) R,G,B, get back 16-bit packed color
    uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
      return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

    void setRotation(uint8_t m) {

      writecommand(ILI9341_MADCTL);
      int rotation = m % 4; // can't be higher than 3
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
    }


    void ILI9341_invertDisplay(bool i) {
      writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
    }

   
};

#endif

