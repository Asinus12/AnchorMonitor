# AnchorMonitor #




Initialization code for STM32F103C8T6 (bluepill) based acostic communicator for monitoring sailboat anchors (somewhere in the future)   
    - SPI communication with LSM6DSL mems accelerometer (STEVAL-MK1178V2)  
    - I2C implementation for SSD1306 driver (1" OLED touch sensitive display)  
    - Serial communication for debuuging and configuring MEMS accelerometer and OLED display.  

Project in current form is reading LGD acceleromter and displaying values in on display (2' complement).
At certain threshold acceleration value it triggers pulse train on PB11 which goes to analog acoustic communicator.
The pulse train in basiclly a binary value of read G-value guarded with synchronization bits. 


## Periphery pinout ##
- **OLED:**    PB6:SCL, PB7:SDA for OLED 
- **SPI:**     PA4:SS,  PA5:CLK, PA6:MISO, PA7:MOSI 
- **RS232:**   USART2 PA02:TX, PA03:RX interrupt  
- **SWD:**     PA:13-SWDIO, PA:14-SWDCK    



## Building ## 

- dependancies: 
    - arm-none-eabi for building: apt install arm-none-eabi  
    - texane st-link for flashing. (https://github.com/stlink-org/stlink )  


- building:
    make clean ... for cleaning files  
    make       ... for building project  
    make flash ... for flashing  
    make debug ... for debugging   



**Notes**
- specs=nosys.specs ... if removed from building process causes error:undefined reference to _exit (syscalls needs implementation)  
- When extracting zipped FW package ..  #define STM32F10X_MD in stm32f10x.h  
- copy core_cm3.h to folder containing stm32f10x.h  (Makefile looks there)  




