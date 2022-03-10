/************************************************
 * @dbg.h 
 * 
 * Debugging file for Anchor monitor 
 * 
 * AUTHOR: Blaz Bogataj, 2021 
 ************************************************/
#include "stm32f10x.h"
#include "stdio.h"
#include "string.h"


#define true 1 
#define false 0
#define BIT_PRINT_REG_SIZE 32
#define DBG_MSG_LENGTH 20


typedef enum {
    eReset = 0x00,
    eUART = 0x01,
    eGyro = 0x02
} eDbgID;


typedef enum {
    RST = 0,
    SPI,
    LED,
    SON,
    I2C,
    WDT,
    UNT,
    COUNT // for cardinality
} eCmdID;

static const char * const CmdID_names[] = { // brez static vrne multiple def, first def here error
	[RST] = "RST",
	[SPI] = "SPI",
	[LED] = "LED",
	[SON] = "SON",
    [I2C] = "I2C",
    [WDT] = "WDT",
    [UNT] = "UNT"
};



struct {
    char message[20];   // received msg
    int parse;          // parse flag 
    int index;          // index for counting
} sDbgMsg;


struct {
    eCmdID ID;      // comand id SON, LED
    int rw;             // read write "bit"
    char data;          // 8 bit data         
} sCMD;




struct {
    uint8_t blink; 
} sStatusLED; 



// initializing functions 
void bb_initStatusLED();
void bb_initUSART2();
void USART1_Init(void);
void USART2_Init(void);
void USART3_Init(void);

// tx rx functions 
void USART1_PutChar(USART_TypeDef* USARTx, uint8_t ch);
void USART1_PutString(USART_TypeDef* USARTx, uint8_t * str);

// print and draw functions
void terminalPrintLogo();
void terminalPrintCommands();
void terminalPrintRegisters();
void bitPrint2(volatile uint32_t* x, char* comment );

// data functions 
int extractCommand(const char* buffer, int i);
int extractCommands();
int checkCommand(char* buffer, char *original);
void reset_dbg_msg();