/************************************************
 * @sonar.c 
 * 
 * Soanr file for Anchor monitor 
 * 
 * AUTHOR: Blaz Bogataj, 2021 
 ************************************************/
#include "stm32f10x.h"




void initPWMOutput();
void bb_initSonarLED();
void sonar_send( char hexval, int delay );