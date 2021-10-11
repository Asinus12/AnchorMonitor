/************************************************
 * @oled.c 
 * 
 * OLED file for Anchor monitor 
 * 
 * AUTHOR: Blaz Bogataj, 2021 
 ************************************************/
#include "oled.h"
#include "gyro.h"



/* I2C on PB6-SCL PB7-SDA */ 
void i2cm_init(I2C_TypeDef* I2Cx, uint32_t i2c_clock)
{
  if (I2Cx == I2C1)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  else
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  I2C_Cmd(I2Cx, DISABLE);
  I2C_DeInit(I2Cx);
  I2C_InitTypeDef i2c_InitStruct;
  i2c_InitStruct.I2C_Mode = I2C_Mode_I2C;
  i2c_InitStruct.I2C_ClockSpeed = i2c_clock;
  i2c_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  i2c_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  i2c_InitStruct.I2C_Ack = I2C_Ack_Disable;
  i2c_InitStruct.I2C_OwnAddress1 = 0;
  I2C_Cmd(I2Cx, ENABLE);
  I2C_Init(I2Cx, &i2c_InitStruct);

  GPIO_InitTypeDef InitStruct;
  InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
  InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

  if (I2Cx == I2C1)
    InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  else
    InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;

  GPIO_Init(GPIOB, &InitStruct);

 

}

int8_t i2cm_Start(I2C_TypeDef* I2Cx, uint8_t slave_addr, uint8_t IsRead, uint16_t TimeOut)
{

  uint16_t TOcntr;

  I2C_GenerateSTART(I2Cx, ENABLE);
  TOcntr = TimeOut;
  while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) && TOcntr) {TOcntr--;}
  if (!TOcntr)
    return I2C_ERR_HWerr;

  if (IsRead)
  {
    I2C_Send7bitAddress(I2Cx, slave_addr << 1, I2C_Direction_Receiver);
    TOcntr = TimeOut;
    while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && TOcntr) {TOcntr--;}
  }
  else
  {
    I2C_Send7bitAddress(I2Cx, slave_addr << 1, I2C_Direction_Transmitter);
    TOcntr = TimeOut;
    while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && TOcntr) {TOcntr--;}
  }

  if (!TOcntr)
	{
      return I2C_ERR_NotConnect;
	}

  return I2C_ERR_Ok;
}

int8_t i2cm_Stop(I2C_TypeDef* I2Cx, uint16_t TimeOut)
{
  I2C_GenerateSTOP(I2Cx, ENABLE);
  uint16_t TOcntr = TimeOut;
  while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) && TOcntr);
  if (!TOcntr)
	{
    return I2C_ERR_HWerr;
	}

  return I2C_ERR_Ok;
}

int8_t i2cm_WriteBuff(I2C_TypeDef* I2Cx, uint8_t *pbuf, uint16_t len, uint16_t TimeOut)
{
  uint16_t TOcntr;

  while (len--)
  {
    I2C_SendData(I2Cx, *(pbuf++));
    TOcntr = TimeOut;
    while((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && TOcntr) {TOcntr--;}
    if (!TOcntr)
      return I2C_ERR_NotConnect;
  }

  return I2C_ERR_Ok;
}

int8_t i2cm_ReadBuffAndStop(I2C_TypeDef* I2Cx, uint8_t *pbuf, uint16_t len, uint16_t TimeOut)
{
  uint16_t TOcntr;

  I2C_AcknowledgeConfig(I2Cx, ENABLE);

  while (len-- != 1)
  {
    TOcntr = TimeOut;
    while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ) && TOcntr) {TOcntr--;}
    *pbuf++ = I2C_ReceiveData(I2Cx);
  }

  I2C_AcknowledgeConfig(I2Cx, DISABLE);
  I2C_GenerateSTOP(I2Cx,ENABLE);

  TOcntr = TimeOut;
  while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ) && TOcntr) {TOcntr--;}
  *pbuf++ = I2C_ReceiveData(I2Cx);

  i2cm_Stop(I2Cx, TimeOut);

  return I2C_ERR_Ok;
}

void ssd1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	uint8_t tempBuff = 0x40;

	int8_t err = i2cm_Start(I2C1 , address, 0 , 1000);
  if (!err)
  {
    i2cm_WriteBuff(I2C1, &tempBuff , 1, 1000);
    i2cm_WriteBuff(I2C1 , data , count , 1000);
  }
  i2cm_Stop(I2C1 , 1000);
}

void ssd1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t data) {

  int8_t err = i2cm_Start(I2C1 , address, 0 , 1000);
  if (!err)
  {
    i2cm_WriteBuff(I2C1, &reg , 1, 1000);
    i2cm_WriteBuff(I2C1 , &data , 1 , 1000);
  }
  i2cm_Stop(I2C1 , 1000);
}

void SSD1306_ON(){
	SSD1306_WRITECOMMAND(0xAF); 
}

void SSD1306_OFF(){
	SSD1306_WRITECOMMAND(0xAE); 
}

void SSD1306_ClearScreen( uint8_t color){
	  memset(SSD1306_Buffer, BLACK, sizeof(SSD1306_Buffer));

}

void SSD1306_UpdateScreen() {
	uint8_t m;
    uint8_t i;

	for (m = 0; m < 8; m++) {
		SSD1306_WRITECOMMAND(0xB0 + m);
		SSD1306_WRITECOMMAND(0x00);
		SSD1306_WRITECOMMAND(0x10);

		/* Write multi data */
		 ssd1306_I2C_WriteMulti(SSD1306_I2C_ADDR, 0x40, &SSD1306_Buffer[SSD1306_WIDTH * m], SSD1306_WIDTH);

	}
}

void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color) {
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Error */
		return;
	}



	/* Set color */
	if (color == WHITE) {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

char SSD1306_Putc(char ch, FontDef_t* Font, uint8_t color, int x, int y) {
	uint32_t i, b, j;

	/* Check available space in LCD */
	if (
		SSD1306_WIDTH <= (x + Font->FontWidth) ||
		SSD1306_HEIGHT <= (y + Font->FontHeight)
	) {
		/* Error */
		return 0;
	}

	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				SSD1306_DrawPixel(x + j, (y + i),  color);
			} else {
				SSD1306_DrawPixel(x + j, (y + i), !color);
			}
		}
	}



	/* Return character written */
	return ch;
}

void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t c) {
	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	/* Check for overflow */
	if (x0 >= SSD1306_WIDTH) {
		x0 = SSD1306_WIDTH - 1;
	}
	if (x1 >= SSD1306_WIDTH) {
		x1 = SSD1306_WIDTH - 1;
	}
	if (y0 >= SSD1306_HEIGHT) {
		y0 = SSD1306_HEIGHT - 1;
	}
	if (y1 >= SSD1306_HEIGHT) {
		y1 = SSD1306_HEIGHT - 1;
	}

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			SSD1306_DrawPixel(x0, i, c);
		}

		/* Return from function */
		return;
	}

	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Horizontal line */
		for (i = x0; i <= x1; i++) {
			SSD1306_DrawPixel(i, y0, c);
		}

		/* Return from function */
		return;
	}

	while (1) {
		SSD1306_DrawPixel(x0, y0, c);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}
}

void SSD1306_init_seq(){
  SSD1306_WRITECOMMAND(0xAE); //display off
	SSD1306_WRITECOMMAND(0x20); //Set Memory Addressing Mode
	SSD1306_WRITECOMMAND(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	SSD1306_WRITECOMMAND(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	SSD1306_WRITECOMMAND(0xC8); //Set COM Output Scan Direction
	SSD1306_WRITECOMMAND(0x00); //---set low column address
	SSD1306_WRITECOMMAND(0x10); //---set high column address
	SSD1306_WRITECOMMAND(0x40); //--set start line address
	SSD1306_WRITECOMMAND(0x81); //--set contrast control register
	SSD1306_WRITECOMMAND(0xFF);
	SSD1306_WRITECOMMAND(0xA1); //--set segment re-map 0 to 127
	SSD1306_WRITECOMMAND(0xA6); //--set normal display
	SSD1306_WRITECOMMAND(0xA8); //--set multiplex ratio(1 to 64)
	SSD1306_WRITECOMMAND(0x3F); //
	SSD1306_WRITECOMMAND(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	SSD1306_WRITECOMMAND(0xD3); //-set display offset
	SSD1306_WRITECOMMAND(0x00); //-not offset
	SSD1306_WRITECOMMAND(0xD5); //--set display clock divide ratio/oscillator frequency
	SSD1306_WRITECOMMAND(0xF0); //--set divide ratio
	SSD1306_WRITECOMMAND(0xD9); //--set pre-charge period
	SSD1306_WRITECOMMAND(0x22); //
	SSD1306_WRITECOMMAND(0xDA); //--set com pins hardware configuration
	SSD1306_WRITECOMMAND(0x12);
	SSD1306_WRITECOMMAND(0xDB); //--set vcomh
	SSD1306_WRITECOMMAND(0x20); //0x20,0.77xVcc
	SSD1306_WRITECOMMAND(0x8D); //--set DC-DC enable
	SSD1306_WRITECOMMAND(0x14); //
	SSD1306_WRITECOMMAND(0xAE); //--turn on SSD1306 panel

}

void SSD1306_DrawLogo(int x, int y ){

    // draw mast 
    SSD1306_DrawLine(x+23,y,x+23,y-29, WHITE);
    // draw left sail 
    SSD1306_DrawLine(x+5,y,x+23,y-23, WHITE);
    // draw right sail
    SSD1306_DrawLine(x+44,y,x+23,y-29, WHITE);
    // draw deck
    SSD1306_DrawLine(x, y, x+44, y, WHITE);
    // draw bottom
    SSD1306_DrawLine(x+9, y+9, x+35, y+9, WHITE);
    // draw left
    SSD1306_DrawLine(x, y, x+9, y+9, WHITE);
    // drawight 
    SSD1306_DrawLine(x+35, y+9,x+44, y, WHITE);
    

    // // draw left sea 
    // SSD1306_DrawLine(x, y+5, x-30, y+5, WHITE);
    // // draw right sea 
    // SSD1306_DrawLine(x+44, y+5, x+44+30 , y+5, WHITE);

    
}

void drawWireFrame(){

}

void SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color, int x, int y){

    char* p = str; 
    int xn = x;

    while(*p != '\0'){
            SSD1306_Putc(*p++, Font, color, xn, y);
            xn = xn + Font->FontWidth + 1;
    }
} 

uint8_t TouchChecksum (uint8_t* msg, uint32_t length){
  
  uint32_t checksum = 0;
  uint32_t i;
  
  for(i=0; i<length; i++){
    checksum += msg[i];
  }
  
  return (uint8_t) ((~checksum) & 0xFF) +1; 

}

void processTouch(uint8_t* chp){
  
  uint8_t pt1x, pt1y, pt2x, pt2y;
  uint8_t buf[8]; 
  
  if(buf[7] == TouchChecksum(buf, 7)){
  

  pt1x = ((buf[1] & 0xF0) << 4) | buf[2]; 
  pt1y = ((buf[1] & 0x0F) << 8) | buf[3];
  pt2x = ((buf[4] & 0xF0) << 4) | buf[5] + pt1x;
  pt2y = ((buf[4] & 0x0F) << 8) | buf[6] + pt1y;

  __itoa(pt1x, buf, 10);
  USART1_PutString(USART2, buf);
  USART1_PutString(USART1, "\n\r");
  __itoa(pt1x, buf, 10);
  USART1_PutString(USART2, buf);
  USART1_PutString(USART1, "\n\r");
  __itoa(pt1x, buf, 10);
  USART1_PutString(USART2, buf);
  USART1_PutString(USART1, "\n\r");
  __itoa(pt1x, buf, 10);
  USART1_PutString(USART2, buf);
  USART1_PutString(USART1, "\n\r");

  switch ( buf[5] & 0x3F )
  {
  case 0x01: 
    // key 1 
    break; 
  case 0x02:
    //key 2 
    break; 
  case 0x04:
    //key 3 
    break;
  case 0x08:
    //key 4 
    break; 
  case 0x10:
    //key 5 
    break;
  case 0x20:
    //key 6 
    break; 
  default:
    // no key 
    break;
  }

  //fingers = 0; 
  //if((pt1x == pt2x) && (pt1y == pt2y))
  //  fingers = 1; 
  //else fingers = 2;
 
}
}

void SSD1306_DisplayAxis(){

	  getAxis();
      convertAxis();
      SSD1306_ClearScreen(BLACK);

	  SSD1306_Puts("X: ", &Font_7x10, WHITE, 1, 9);
      SSD1306_Puts("Y:", &Font_7x10, WHITE, 1, 28);
      SSD1306_Puts("Z: ", &Font_7x10, WHITE, 1, 46);

      SSD1306_Puts(&gyro.itoaBuffer[X_AXIS], &Font_7x10, WHITE, 21, 9);
      SSD1306_Puts(&gyro.itoaBuffer[Y_AXIS], &Font_7x10, WHITE, 21, 28);
      SSD1306_Puts(&gyro.itoaBuffer[Z_AXIS], &Font_7x10, WHITE, 21, 46);
      SSD1306_UpdateScreen();
}
