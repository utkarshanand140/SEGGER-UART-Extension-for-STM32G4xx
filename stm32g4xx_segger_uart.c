/**********************************************************
File    : stm32g4xx_segger_uart.c
Purpose : For getting the segger system view software to work on STM32G4xx 
          microcontrollers for real time trace monitoring over UART ports.
--------- END-OF-HEADER ---------------------------------*/

#include "SEGGER_SYSVIEW.h"

#if (SEGGER_UART_REC == 1)
#include "SEGGER_RTT.h"
#include "stm32g4xx.h"