/************************************************
 * @gyro.h 
 * 
 * LSM6DSL gyro file for Anchor monitor 
 * 
 * 
 * AUTHOR: Blaz Bogataj, 2021 
 ************************************************/
#include "stm32f10x.h"
#include "stdlib.h"


#define DIMENSIONS 3
#define X_AXIS 0
#define Y_AXIS 1 
#define Z_AXIS 2 

#define READWRITE_CMD              ((uint8_t)0x80) 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
#define DUMMY_BYTE                 ((uint8_t)0x00)

#define __SPI_CS_ENABLE__() GPIOA->BSRR |= GPIO_BSRR_BR4;
#define __SPI_CS_DISABLE__() GPIOA->BSRR |= GPIO_BSRR_BS4;
// #define __SPI_CS_ENABLE__() do { GPIOA->BSRR |= GPIO_BSRR_BR4; } while(0)
// #define __SPI_CS_DISABLE__() do { GPIOA->BSRR |= GPIO_BSRR_BS4; } while(0)


/* Accelero Full_ScaleSelection */
#define LSM6DSL_ACC_FULLSCALE_2G          ((uint8_t)0x00) /*!< ±2 g */
#define LSM6DSL_ACC_FULLSCALE_4G          ((uint8_t)0x08) /*!< ±4 g */
#define LSM6DSL_ACC_FULLSCALE_8G          ((uint8_t)0x0C) /*!< ±8 g */
#define LSM6DSL_ACC_FULLSCALE_16G         ((uint8_t)0x04) /*!< ±16 g */

/* Accelero Full Scale Sensitivity */
#define LSM6DSL_ACC_SENSITIVITY_2G     ((float)0.061f)  /*!< accelerometer sensitivity with 2 g full scale  [mgauss/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_4G     ((float)0.122f)  /*!< accelerometer sensitivity with 4 g full scale  [mgauss/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_8G     ((float)0.244f)  /*!< accelerometer sensitivity with 8 g full scale  [mgauss/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_16G    ((float)0.488f)  /*!< accelerometer sensitivity with 12 g full scale [mgauss/LSB] */

#define LSM6DSL_ACC_GYRO_CTRL1_XL 0x10




struct {
    char itoaBuffer[DIMENSIONS][8]; 
    uint8_t msb[DIMENSIONS];
    uint8_t lsb[DIMENSIONS];
    uint16_t data[DIMENSIONS];
    uint8_t LSM6DSL_reg;
} gyro;




// tx rx functions 
void bb_transmitSPI(SPI_TypeDef *SPIx, uint16_t data);
uint8_t LSM6DSL_SendByte(uint8_t byte);
void LSM6DSL_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void LSM6DSL_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

// init functions 
void bb_initSPI(SPI_TypeDef *SPIx);

// data functions
void getAxis();
void convertAxis();
void printAxis();



/******************* LSM6DSL  datasheet ************************

                         (Y) roll
                            |  
                        _________
                        |*       |
                        |        |
                        |  (Z)   | -- (X) pitch
                        | (yawn) |
                        |________|



    Pin name     |    Pin Description
    -------------+-------------------------------------------------------
    
    1 - SDO/SAO           [SPI SDO]* / I2C less significant bit of dev addr 
    2 - SDA/SDI/SDO       I2C serial data / SPI data input / 3-Wire Serial data out
    3 - SCL/SPC           I2C clk / SPI clk
    4 - INT1              Programmable interrupt
    5 - Vdd_io            Power suplly for pins (3V, 3,6V MAX!)
    6 - GND
    7 - GND
    8 - VDD               Power supply (3V, 3,6V MAX!)
    9 - INT2              Data enabled interrupt 
    10 - Not connected
    11 - Not connected
    12 - CS               [SPI CS]*
    13 - SCL              [SPI CLK]*
    14 - SDA              [SPI SDI]*

     - Adapter Board: Jumper 2: 22:SDO, 21:SDA, 20:SCL, 19:CS   
     
    ////////// SPI READ ////////// 
    bit 0: READ bit. The value is 1.
    bit 1-7: address AD(6:0). This is the address field of the indexed register.
    bit 8-15: data DO(7:0) (read mode). This is the data that will be read from the device (MSb
    first).
    bit 16-...: data DO(...-8). Further data in multiple byte reads.
    @ test register WHO AM I at 0x0F, Default value 0x6A 01101010


    ////////// SPI WRITE //////////
    bit 0: WRITE bit. The value is 0.
    bit 1 -7: address AD(6:0). This is the address field of the indexed register.
    bit 8-15: data DI(7:0) (write mode). This is the data that is written inside the device (MSb
    first).
    bit 16-... : data DI(...-8). Further data in multiple byte writes.
    

*/ 