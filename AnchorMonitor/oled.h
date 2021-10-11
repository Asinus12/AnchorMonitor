/************************************************
 * @oled.c 
 * 
 * Oled file for Anchor monitor 
 * 
 * AUTHOR: Blaz Bogataj, 2021 
 ************************************************/
#include "stdint.h"
#include "fonts.h"
//#include "gyro.h"

/******************************************
  Touch Controller I2C Address : 0x26
  SSD1306 Driver I2C Address: 0x3C 
 
  Touch Controller Connector 
  1 - GND       2 - VDD
  3 - RST       4 - INT
  5 - SCL       6 - SDA 


  Touch Controller Packet
  B0: 0x52
  B1: position1_x [11:8] position1_y[11:8]
  B2: position1_x [7:0]
  B3: position1_y [7:0]
  B4: distance_x [11:8] position_y[11:8]
  B5: distance_x [7:0]
  B6: distance_y [7:0]
  B7: checksum[7:0]

*********************************************/



#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_BUFF_SIZE ((SSD1306_WIDTH * SSD1306_HEIGHT) / 8 )
 uint8_t SSD1306_Buffer[SSD1306_BUFF_SIZE];

#define SSD1306_I2C_ADDR     0x3C //    0x78 = 0x3c << 1
#define SSD1306_WRITECOMMAND(command)      ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, (command))
#define SSD1306_WRITEDATA(data)            ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x40, (data))
#define I2C_ERR_Ok         0
#define I2C_ERR_NotConnect -1
#define I2C_ERR_BadChksum  -2
#define I2C_ERR_HWerr      -3

#define WHITE 0xFF
#define BLACK 0x00



struct {
  int refresh_sig;
  int refresh_enable;
} display;


/* I2C on PB6-SCL PB7-SDA */ 
void i2cm_init(I2C_TypeDef* I2Cx, uint32_t i2c_clock);
int8_t i2cm_Start(I2C_TypeDef* I2Cx, uint8_t slave_addr, uint8_t IsRead, uint16_t TimeOut);
int8_t i2cm_Stop(I2C_TypeDef* I2Cx, uint16_t TimeOut);
int8_t i2cm_WriteBuff(I2C_TypeDef* I2Cx, uint8_t *pbuf, uint16_t len, uint16_t TimeOut);
int8_t i2cm_ReadBuffAndStop(I2C_TypeDef* I2Cx, uint8_t *pbuf, uint16_t len, uint16_t TimeOut);
void ssd1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count);
void ssd1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t data);
void SSD1306_ON();
void SSD1306_OFF();
void SSD1306_ClearScreen(uint8_t color);
void SSD1306_UpdateScreen();
void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color);
char SSD1306_Putc(char ch, FontDef_t* Font, uint8_t color, int x, int y);
void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t c);
void SSD1306_init_seq();
void SSD1306_DrawLogo(int x, int y );
void wireframe();
void SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color, int x, int y);
uint8_t TouchChecksum (uint8_t* msg, uint32_t length);
void processTouch(uint8_t* chp);
void SSD1306_DisplayAxis();