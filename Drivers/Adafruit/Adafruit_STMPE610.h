/*!
 *
 *  @file Adafruit_STMPE610.cpp
 *
 *  @mainpage Adafruit STMPE610 Resistive Touch Screen Controller
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the Adafruit STMPE610 Resistive
 *  touch screen controller breakout
 *  ----> http://www.adafruit.com/products/1571
 *
 *  Check out the links above for our tutorials and wiring diagrams
 *  These breakouts use SPI or I2C to communicate
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 *  @section author Author
 *
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */


/** STMPE610 Address **/
#define STMPE_ADDR 0x41

/** Reset Control **/
#define STMPE_SYS_CTRL1 0x03
#define STMPE_SYS_CTRL1_RESET 0x02

/** Clock Contrl **/
#define STMPE_SYS_CTRL2 0x04

/** Touchscreen controller setup **/
#define STMPE_TSC_CTRL 0x40
#define STMPE_TSC_CTRL_EN 0x01
#define STMPE_TSC_CTRL_XYZ 0x00
#define STMPE_TSC_CTRL_XY 0x02

/** Interrupt control **/
#define STMPE_INT_CTRL 0x09
#define STMPE_INT_CTRL_POL_HIGH 0x04
#define STMPE_INT_CTRL_POL_LOW 0x00
#define STMPE_INT_CTRL_EDGE 0x02
#define STMPE_INT_CTRL_LEVEL 0x00
#define STMPE_INT_CTRL_ENABLE 0x01
#define STMPE_INT_CTRL_DISABLE 0x00

/** Interrupt enable **/
#define STMPE_INT_EN 0x0A
#define STMPE_INT_EN_TOUCHDET 0x01
#define STMPE_INT_EN_FIFOTH 0x02
#define STMPE_INT_EN_FIFOOF 0x04
#define STMPE_INT_EN_FIFOFULL 0x08
#define STMPE_INT_EN_FIFOEMPTY 0x10
#define STMPE_INT_EN_ADC 0x40
#define STMPE_INT_EN_GPIO 0x80

/** Interrupt status **/
#define STMPE_INT_STA 0x0B
#define STMPE_INT_STA_TOUCHDET 0x01

/** ADC control **/
#define STMPE_ADC_CTRL1 0x20
#define STMPE_ADC_CTRL1_12BIT 0x08
#define STMPE_ADC_CTRL1_10BIT 0x00

/** ADC control **/
#define STMPE_ADC_CTRL2 0x21
#define STMPE_ADC_CTRL2_1_625MHZ 0x00
#define STMPE_ADC_CTRL2_3_25MHZ 0x01
#define STMPE_ADC_CTRL2_6_5MHZ 0x02

/** Touchscreen controller configuration **/
#define STMPE_TSC_CFG 0x41
#define STMPE_TSC_CFG_1SAMPLE 0x00
#define STMPE_TSC_CFG_2SAMPLE 0x40
#define STMPE_TSC_CFG_4SAMPLE 0x80
#define STMPE_TSC_CFG_8SAMPLE 0xC0
#define STMPE_TSC_CFG_DELAY_10US 0x00
#define STMPE_TSC_CFG_DELAY_50US 0x08
#define STMPE_TSC_CFG_DELAY_100US 0x10
#define STMPE_TSC_CFG_DELAY_500US 0x18
#define STMPE_TSC_CFG_DELAY_1MS 0x20
#define STMPE_TSC_CFG_DELAY_5MS 0x28
#define STMPE_TSC_CFG_DELAY_10MS 0x30
#define STMPE_TSC_CFG_DELAY_50MS 0x38
#define STMPE_TSC_CFG_SETTLE_10US 0x00
#define STMPE_TSC_CFG_SETTLE_100US 0x01
#define STMPE_TSC_CFG_SETTLE_500US 0x02
#define STMPE_TSC_CFG_SETTLE_1MS 0x03
#define STMPE_TSC_CFG_SETTLE_5MS 0x04
#define STMPE_TSC_CFG_SETTLE_10MS 0x05
#define STMPE_TSC_CFG_SETTLE_50MS 0x06
#define STMPE_TSC_CFG_SETTLE_100MS 0x07

/** FIFO level to generate interrupt **/
#define STMPE_FIFO_TH 0x4A

/** Current filled level of FIFO **/
#define STMPE_FIFO_SIZE 0x4C

/** Current status of FIFO **/
#define STMPE_FIFO_STA 0x4B
#define STMPE_FIFO_STA_RESET 0x01
#define STMPE_FIFO_STA_OFLOW 0x80
#define STMPE_FIFO_STA_FULL 0x40
#define STMPE_FIFO_STA_EMPTY 0x20
#define STMPE_FIFO_STA_THTRIG 0x10

/** Touchscreen controller drive I **/
#define STMPE_TSC_I_DRIVE 0x58
#define STMPE_TSC_I_DRIVE_20MA 0x00
#define STMPE_TSC_I_DRIVE_50MA 0x01

/** Data port for TSC data address **/
#define STMPE_TSC_DATA_X 0x4D
#define STMPE_TSC_DATA_Y 0x4F
#define STMPE_TSC_FRACTION_Z 0x56

/** GPIO **/
#define STMPE_GPIO_SET_PIN 0x10
#define STMPE_GPIO_CLR_PIN 0x11
#define STMPE_GPIO_DIR 0x13
#define STMPE_GPIO_ALT_FUNCT 0x17

// STMPE610 CS Digital 8 (PB2)
#define STMP_CS_LOW  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
#define STMP_CS_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
/*!
 *  @brief  Class for working with points
 */
class TS_Point {
public:
  TS_Point(){x=y=0;}
  TS_Point(int16_t x0, int16_t y0, int16_t z0){
    x=x0;y=y0;z=z0;
  };

  bool operator==(TS_Point p1){
    return ((p1.x == x) && (p1.y == y) && (p1.z == z));
  }
  bool operator!=(TS_Point p1){
    return ((p1.x != x) || (p1.y != y) || (p1.z != z));
  }

  int16_t x; /**< x coordinate **/
  int16_t y; /**< y coordinate **/
  int16_t z; /**< z coordinate **/
};

class STMPE610{
  private:
  public:
    STMPE610(){
      SPI1_Init();
    }
    void SPI1_Init() {
      /* full duplex master, 8 bit transfer, default phase and polarity */
      SPI1->CR1 = SPI_CR1_BR_2 | SPI_CR1_BR_0; //64 divider
      //SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA;
      SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | SPI_CR1_CPOL; // mode 3
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

    uint8_t spiIn(void){
      SPI1_Init();
      uint8_t out=0x00,in=0x00;
      while(!(SPI1->SR & SPI_SR_TXE)) ;
      *(volatile uint8_t *)&SPI1->DR = out;
      while(!(SPI1->SR & SPI_SR_RXNE)) ;
      in = *(volatile uint8_t *)&SPI1->DR;
      return in;
    }
    void spiOut(uint8_t x){
      SPI1_Init();
      uint8_t in;
      while(!(SPI1->SR & SPI_SR_TXE)) ;
      *(volatile uint8_t *)&SPI1->DR = x;
      while(!(SPI1->SR & SPI_SR_RXNE)) ;
      in = *(volatile uint8_t *)&SPI1->DR;
    }
    uint8_t readRegister8(uint8_t reg){
      uint8_t x;
      //digitalWrite(_CS, LOW);
      STMP_CS_LOW;
      spiOut(0x80 | reg);
      spiOut(0x00);
      x = spiIn();
      //digitalWrite(_CS, HIGH);
      STMP_CS_HIGH;
      return x;
    }
    uint16_t readRegister16(uint8_t reg){
      uint16_t x;
      //digitalWrite(_CS, LOW);
      STMP_CS_LOW;
      spiOut(0x80 | reg);
      spiOut(0x00);
      x = spiIn();
      x <<= 8;
      x |= spiIn();
      //digitalWrite(_CS, HIGH);
      STMP_CS_HIGH;
      return x;
    }

    void writeRegister8(uint8_t reg, uint8_t val){
      //digitalWrite(_CS, LOW);
      STMP_CS_LOW;
      spiOut(reg);
      spiOut(val);
      //digitalWrite(_CS, HIGH);
      STMP_CS_HIGH;
    }
    void readData(uint16_t *x, uint16_t* y, uint8_t* z){
      uint8_t data[4];

      for (uint8_t i = 0; i < 4; i++) {
        data[i] = readRegister8(0xD7); // _spi->transfer(0x00);
      }
      *x = data[0];
      *x <<= 4;
      *x |= (data[1] >> 4);
      *y = data[1] & 0x0F;
      *y <<= 8;
      *y |= data[2];
      *z = data[3];
    }
    bool init(void){
      if (getVersion() != 0x811) 
        return false;
          //mySPISettings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
          //m_spiMode = SPI_MODE1;
      

      writeRegister8(STMPE_SYS_CTRL1, STMPE_SYS_CTRL1_RESET);
      //delay(10);
      HAL_Delay(10);

      for (uint8_t i = 0; i < 65; i++) {
        readRegister8(i);
      }

      writeRegister8(STMPE_SYS_CTRL2, 0x0); // turn on clocks!
      writeRegister8(STMPE_TSC_CTRL,
                     STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN); // XYZ and enable!
      // Serial.println(readRegister8(STMPE_TSC_CTRL), HEX);
      writeRegister8(STMPE_INT_EN, STMPE_INT_EN_TOUCHDET);
      writeRegister8(STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_12BIT |
                                          (0x6 << 4)); // 96 clocks per conversion
      writeRegister8(STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ);
      writeRegister8(STMPE_TSC_CFG, STMPE_TSC_CFG_4SAMPLE |
                                        STMPE_TSC_CFG_DELAY_1MS |
                                        STMPE_TSC_CFG_SETTLE_50MS);
      writeRegister8(STMPE_TSC_FRACTION_Z, 0x6);
      writeRegister8(STMPE_FIFO_TH, 1);
      writeRegister8(STMPE_FIFO_STA, STMPE_FIFO_STA_RESET);
      writeRegister8(STMPE_FIFO_STA, 0); // unreset
      writeRegister8(STMPE_TSC_I_DRIVE, STMPE_TSC_I_DRIVE_20MA);
      writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
      writeRegister8(STMPE_INT_CTRL,
                     STMPE_INT_CTRL_POL_HIGH | STMPE_INT_CTRL_ENABLE);
    
      return true;
    }
    bool touched(){
      return (readRegister8(STMPE_TSC_CTRL) & 0x80);
    }
    bool bufferEmpty(){
      return (readRegister8(STMPE_FIFO_STA) & STMPE_FIFO_STA_EMPTY);
    }
    uint8_t bufferSize(){
      return readRegister8(STMPE_FIFO_SIZE);
    }
    uint16_t getVersion(){
      uint16_t v;
      // Serial.print("get version");
      v = readRegister8(0);
      v <<= 8;
      v |= readRegister8(1);
      // Serial.print("Version: 0x"); Serial.println(v, HEX);
      return v;
    }
    
    TS_Point getPoint(){
      uint16_t x, y;
      uint8_t z;

      /* Making sure that we are reading all data before leaving */
      while (!bufferEmpty()) {
        readData(&x, &y, &z);
      }

      if (bufferEmpty())
        writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints

      return TS_Point(x, y, z);
    }
};


