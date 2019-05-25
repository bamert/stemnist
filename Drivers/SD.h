#ifndef _NDB_SDCARD
#define _NDB_SDCARD

#define SD_CS_LOW  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
#define SD_CS_HIGH HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND (MMC) */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */
#define	CMDREAD				17
#define	CMDWRITE			24
#define	CMDREADCSD   	9
uint32_t CardType;
class SDCard{
  private:
  void SPI1_Init() {
    //SPI1->CR1 &= ~SPI_CR1_SPE;
    /* full duplex master, 8 bit transfer, default phase and polarity */
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0; //64 divider
    /* Disable receive FIFO, it'd complicate things when there is an odd number of bytes to transfer */
    SPI1->CR2 = SPI_CR2_FRXTH;
  }
  void SPI1_SetInitSpeed(){
    SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0; //64 divider
    //SPI1->CR1 &=~(SPI_CR1_BR_1 ); //switch off clock divdivider l
  }
  // /div8: 10Mhz
  void SPI1_SetOpSpeed(){ 
    SPI1->CR1 |= SPI_CR1_BR_1; 
    SPI1->CR1 &=~(SPI_CR1_BR_2 | SPI_CR1_BR_0 ); //switch off clock div l
  }
  uint8_t spix(uint8_t out){
      uint8_t in=0x00;
      while(!(SPI1->SR & SPI_SR_TXE)) ;
      *(volatile uint8_t *)&SPI1->DR = out;
      while(!(SPI1->SR & SPI_SR_RXNE)) ;
      in = *(volatile uint8_t *)&SPI1->DR;
      return in;
 }

  public:
  SDCard() {
    SPI1_Init();
    SPI1_SetInitSpeed();
  }
  uint8_t sdCommand(uint8_t cmd, uint32_t param) {
    uint8_t n,res;
    if (cmd & 0x80){
      cmd &= 0x7F;
      res = sdCommand(CMD55, 0);
      if (res > 1) return res;
    }

    spix(0xff);
    spix(0x40 | cmd);
    spix((uint8_t) (param >> 24));
    spix((uint8_t) (param >> 16));
    spix((uint8_t) (param >> 8));
    spix((uint8_t) (param));
    n = 0x01;							/* Dummy CRC + Stop */
    if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
    if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
    spix(n);

    /* Receive command response */
    if (cmd == CMD12) spix(0xff);		/* Skip a byte when stop reading */
    n = 50;								/* Wait for a valid response in timeout of 10 attempts */
    do
      res = spix(0xff);
    while ((res & 0x80) && --n);

    spix(0x95); // Checksum (should be only valid for first command (0) 
    spix(0xff); // eat empty command - response 
    return res;
  }
  uint8_t  init_card(void) {
    uint8_t res;
    uint32_t i;
    uint8_t resp;
    uint8_t n, cmd, ty, ocr[4];
    uint16_t dummy;
    SPI1_SetInitSpeed();
    SD_CS_HIGH
    for (i=0; i<11; i++) { //Pulse 80+ clocks to reset MMC
      spix(0xFF);
    }
    SD_CS_LOW

    ty=0;
    i=200;
    do
    {
      resp = sdCommand(CMD0,0);
    } while(resp!=1 && i--);

    if(resp==1) {
      i =50;
      do {
        resp = sdCommand(CMD8,0x1AA);
	    } while(resp!=1 && i--);
      if (resp == 1) {	// SDv2? /
        for (n = 0; n < 4; n++) ocr[n] = spix(0xff);		// Get trailing return value of R7 resp /
          if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				// The card can work at vdd range of 2.7-3.6V /
            while (i && sdCommand(ACMD41, 1UL << 30));	// Wait for leaving idle state (ACMD41 with HCS bit) /
          if (i && sdCommand(CMD58, 0) == 0) {		// Check CCS bit in the OCR /
            for (n = 0; n < 4; n++) ocr[n] = spix(0xff);
            ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	// SDv2 /
					//dogPuts(1,11,1,"SD v2");
					//if(ocr[0] & 0x40) dogPuts(30,11,1,"HC");
					//dogPuts(1,11,1,"SD v2");
				}//end CSS check
			}//end voltage check
		} else {							// SDv1 or MMCv3 /
			if (sdCommand(ACMD41, 0) <= 1) 	{
				ty = CT_SD1; cmd = ACMD41;	// SDv1 /
				//printf("SDv1");
				//dogPuts(1,11,1,"SD v1");
			} else {
				ty = CT_MMC; cmd = CMD1;	// MMCv3 /
				//printf("MMCv3");
				//dogPuts(1,11,1,"MMC v3");
			}
			while (i && sdCommand(cmd, 0));			// Wait for leaving idle state /
			if (!i || sdCommand(CMD16, 512) != 0)	// Set R/W block length to 512 /
				ty = 0;
		}
	}
  SD_CS_HIGH
	CardType = ty;
	if(ty)return(0); else return(1);
}
uint8_t sd_write(uint32_t adr, uint8_t* buf) {
	uint16_t i;
	uint16_t t=0;
	uint16_t dummy,fb_timeout;
	uint8_t resp;

	dummy=dummy;

  SD_CS_LOW
	if (!(CardType & CT_BLOCK)) adr *=512;
	fb_timeout = 5000;
	if(sdCommand(CMDWRITE,adr) ==0){
	  do {
      resp=spix(0xff);
    }while(resp!=0xff && fb_timeout--);

	  spix(0xfe); /* Start block */
	  for(i=0;i<512;i++) spix(buf[i]); /* Send data */
	  spix(0xff); /* Checksum part 1 */
	  spix(0xff); /* Checksum part 2 */

	  if ((spix(0xff) & 0x1F) != 0x05) return(1);
	}

	while((spix(0xff)!=0xff) && t<10000) {
		t++;
	}
	dummy = spix(0xff);
  SD_CS_HIGH
	return(0);
}
uint8_t sd_read(uint32_t adr, uint8_t* buf) {

	uint8_t cardresp;
	uint8_t firstblock;
	uint8_t c;
	uint16_t fb_timeout=5000;
	uint16_t len;
	uint32_t i;

  len=512;

  SPI1_SetOpSpeed();
  SD_CS_LOW
	if (!(CardType & CT_BLOCK)) adr *=512;
	if(sdCommand(CMDREAD,adr)==0) {
	  do {
      firstblock=spix(0xff);
    }while(firstblock==0xff && fb_timeout--);

	  if(firstblock!=0xfe) {
		//printf("error reading sector");
	 	  return(1);
  	}
	  for(i=0;i<512;i++){
      c = spix(0xff);
      buf[i] = c;
	  }
	}

	spix(0xff);
	spix(0xff);
  SD_CS_HIGH

	return(0);
}


};
#endif
