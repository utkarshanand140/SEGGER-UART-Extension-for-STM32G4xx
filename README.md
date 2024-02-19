#SEGGER UART EXTENTION ADAPTATION FOR STM32G4xx

This project is an adaption of the original segger_uart.c file for using SystemView software on STM32G4xx MCUs. This file is essential for monitoring RTOS traces on the SEGGER SYSTEMVIEW in real-time over the UART ports.

To use this file, following changes have to be made your the STM32 project:-

1. Add this '_stm32g4xx_segger_uart.c_' file in the list of source files to be compiled

2. In _main.c_, add SEGGER_UART_init(baudrate) before calling the SEGGER_SYSVIEW_Conf() function. Also, remove the SEGGER_SYSVIEW_Start() line. Your code should look like this:<br>
    

        SEGGER_UART_init(115200);

        SEGGER_SYSVIEW_Conf();

        // SEGGER_SYSVIEW_Start();


3. In _SEGGER_SYSVIEW_Conf.h_ file, add the following code at apporpriate places:<br>

        #define SEGGER_UART_REC 1
        
        #if (SEGGER_UART_REC == 1)
        extern void HIF_UART_EnableTXEInterrupt(void);

        #define SEGGER_SYSVIEW_ON_EVENT_RECORDED(x) HIF_EnableTXEInterrupt()
        #endinf


4. In SystemView software, 
    - Click on Target > Recorder Configuration, select UART, COM port and Bauderate.
    - Click on Target > Start Recording 



    

Use of this file is at your own risk.
