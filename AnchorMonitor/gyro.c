/************************************************
 * @gyro.c 
 * 
 * LSM6DSL gyro file for Anchor monitor 
 * 
 * AUTHOR: Blaz Bogataj, 2021 
 ************************************************/
#include "gyro.h"




// tx rx functions 
void bb_transmitSPI(SPI_TypeDef *SPIx, uint16_t data) {
    while ((SPIx->SR & SPI_SR_BSY)); // BSY=1: SPI is busy in communication
    while ((SPIx->SR & SPI_SR_TXE) == 0); // TXE=0: Tx buffer not empty

    SPIx->DR = data;

    while ((SPIx->SR & SPI_SR_TXE) == 0);
    while ((SPIx->SR & SPI_SR_BSY));
}
uint8_t LSM6DSL_SendByte(uint8_t byte)
{
  uint8_t ch = 0x00;
  /* Loop while DR register in not empty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
  {
  }
  
  /* Send a Byte through the SPI peripheral */
  SPI_I2S_SendData(SPI1, (uint16_t)byte);

  /* Wait to receive a Byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
  {
  }

  // Delay(200);
  
  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}
void LSM6DSL_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite){
  /* Configure the MS bit: 
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  __SPI_CS_ENABLE__();
  
  /* Send the Address of the indexed register */
  LSM6DSL_SendByte(WriteAddr); // Velikosti prever!

  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    LSM6DSL_SendByte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }

  __SPI_CS_DISABLE__(); // pazi ta dvopicje

}
void LSM6DSL_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  __SPI_CS_ENABLE__(); // zasteka 
  
  /* Send the Address of the indexed register */
  LSM6DSL_SendByte(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to L3GD20 (Slave device) */
    *pBuffer = LSM6DSL_SendByte(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }

  __SPI_CS_DISABLE__();
}

// init functions 
void bb_initSPI(SPI_TypeDef *SPIx) {

    char wrval = 0x00;

    /* SPI clock enabled */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    
    /* GPIOA port clock enable */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    /* PA4 software slave select */
    GPIOA->CRL &= ~GPIO_CRL_CNF4; // 00: General purpose output push-pull
    GPIOA->CRL |= GPIO_CRL_MODE4; // 11: Output mode, max speed 50 MHz.

    /* PA5 SPI clock */
    GPIOA->CRL &= ~GPIO_CRL_CNF5;
    GPIOA->CRL |= GPIO_CRL_CNF5_1; // 10: Alternate function output Push-pull
    GPIOA->CRL |= GPIO_CRL_MODE5; // 11: Output mode, max speed 50 MHz.

    /* PA6 SPI MISO */
    GPIOA->CRL &= ~GPIO_CRL_CNF6;
    GPIOA->CRL |= GPIO_CRL_CNF6_1; // 10: Alternate function output Push-pull
    GPIOA->CRL |= GPIO_CRL_MODE6; // 11: Output mode, max speed 50 MHz.

    /* PA7 SPI MOSI */
    GPIOA->CRL &= ~GPIO_CRL_CNF7;
    GPIOA->CRL |= GPIO_CRL_CNF7_1; // 10: Alternate function output Push-pull
    GPIOA->CRL |= GPIO_CRL_MODE7; // 11: Output mode, max speed 50 MHz.

    /* Remap AFIO */
    AFIO->MAPR &= ~AFIO_MAPR_SPI1_REMAP; // 0: No remap (NSS/PA4, SCK/PA5, MISO/PA6, MOSI/PA7)


    // CR2 reg bit 2(SSOE): 0- SS output is disabled in master mode and the cell can work in multimaster configuration
    //                      1- SS output is enabled in master mode and when the cell is enabled, cell can NOT work in multimaster configuration
    SPIx->CR2 |= SPI_CR2_SSOE; // SS output enable

    // CLEAR CR1
    //SPIx->CR1 = 0; // Clear unwanted values from register

    // CR1 reg DFF bit 11: 0-8bit frame format, 1-16bit frame format
    SPIx->CR1 |= 0x0000; // 8-bit data frame format, SPI_CR1_DFF;  // 16-bit data frame format
    
    // CR1 reg bit 2(MSTR): 0-Slave
    //                      1-Master
    SPIx->CR1 |= SPI_CR1_MSTR; // Master device

    // CR1 reg bit 1 (CPOL): 0-clk to 0 when idle
    //                       1-clk to 1 when idle 
    SPIx->CR1 |= SPI_CR1_CPOL; // Clock polarity: high when idle

    //CR1 reg bit 0 (CPHA): 0-The first clock transition is the first data capture edge
    //                      1-The second clock transition is the first data capture edge
    SPIx->CR1 |= SPI_CR1_CPHA; // Capture data on 2nd edge (rising edge)

    // Bits 5:3 BR[2:0]: Baud rate control
    // 000: fPCLK/2
    // 001: fPCLK/4
    // 010: fPCLK/8
    // 011: fPCLK/16
    // 100: fPCLK/32
    // 101: fPCLK/64
    // 110: fPCLK/128
    // 111: fPCLK/256

    //SPIx->CR1 |= 0 << SPI_CR1_BR_1|SPI_CR1_BR_; // precekiri
    SPIx->CR1 |= SPI_CR1_BR;
    SPIx->CR1 |= SPI_CR1_SPE; // Peripheral enabled


    // Give a little time 
    Delay(10000);

    // Gyro ini sequence 
     wrval = 0x40; // Angular rate [CTRL2_G] reg ODR bits, 104Hz
     LSM6DSL_Write(&wrval, 0x10,1); // angular rate 
     LSM6DSL_Write(&wrval, 0x11,1); // g 

        
    
}

// data functions
void getAxis()
      { 
        
        // sidenote: float = ...[2^3][2^2][2^1][2^0],[2^-1][2^-2]...

        // linear acc
        //     msb   lsb
        // X: 0x29, 0x28
        // Y: 0x2b, 0x2a
        // Z: 0x2d, 0x2c

        Delay(10);

        LSM6DSL_Read(&gyro.msb[X_AXIS], 0x29, 1); // X DIR 
        LSM6DSL_Read(&gyro.lsb[X_AXIS], 0x28, 1); //          
        gyro.data[X_AXIS] = (gyro.msb[X_AXIS] << 8) + gyro.lsb[X_AXIS];  
        // gyro.data[X_AXIS] = (~gyro.data[X_AXIS]) + 1;
        
        Delay(10);

        LSM6DSL_Read(&gyro.msb[Y_AXIS], 0x2b, 1); // Y DIR 
        LSM6DSL_Read(&gyro.lsb[Y_AXIS], 0x2a, 1); //          
        gyro.data[Y_AXIS] = (gyro.msb[Y_AXIS] << 8) + gyro.lsb[Y_AXIS];  
        // gyro.data[Y_AXIS] = (~gyro.data[Y_AXIS]) + 1;

        Delay(10);

        LSM6DSL_Read(&gyro.msb[Z_AXIS], 0x2d, 1); // Z DIR 
        LSM6DSL_Read(&gyro.lsb[Z_AXIS], 0x2c, 1); //          
        gyro.data[Z_AXIS] = (gyro.msb[Z_AXIS] << 8) + gyro.lsb[Z_AXIS];  
        // gyro.data[Z_AXIS] = (~gyro.data[Z_AXIS]) + 1;

      }
void convertAxis(){

  __itoa(gyro.data[X_AXIS]*0.061, &gyro.itoaBuffer[X_AXIS][0], 10);
  __itoa(gyro.data[Y_AXIS]*0.061, &gyro.itoaBuffer[Y_AXIS][0], 10);
  __itoa(gyro.data[Z_AXIS]*0.061, &gyro.itoaBuffer[Z_AXIS][0], 10);

}
void printAxis(){

USART1_PutString(USART2, "X:");
USART1_PutString(USART2, &gyro.itoaBuffer[X_AXIS][0]);
USART1_PutString(USART2, " Y:");
USART1_PutString(USART2, &gyro.itoaBuffer[Y_AXIS][0]);
USART1_PutString(USART2, " Z:");
USART1_PutString(USART2, &gyro.itoaBuffer[Z_AXIS][0]);
USART1_PutString(USART2, "\n\r");

}


