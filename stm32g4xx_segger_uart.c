/**********************************************************
File    : stm32g4xx_segger_uart.c
Purpose : For getting the segger system view software to work on STM32G4xx 
          microcontrollers for real time trace monitoring over UART ports.
Author : Utkarsh Anand
--------- END-OF-HEADER ---------------------------------*/

#include "SEGGER_SYSVIEW.h"

#if (SEGGER_UART_REC == 1)
#include "SEGGER_RTT.h"
#include "stm32g4xx.h"

#define OS_FSYS             80000000L  // MCU Core Clock Frequency (Hz)
#define RCC_BASE_ADDR       0x40021000


#define OFF_AHB2ENR         0x4C      // AHB1 peripheral clock enable register
#define OFF_APB1ENR         0x58      // APB1 peripheral clock enable register
#define OFF_APB2ENR         0x60      // APB2 peripheral clock enable register


#define RCC_AHB2ENR         (*(volatile uint32_t*)(RCC_BASE_ADDR + OFF_AHB2ENR))
#define RCC_APB1ENR         (*(volatile uint32_t*)(RCC_BASE_ADDR + OFF_APB1ENR))
#define RCC_APB2ENR         (*(volatile uint32_t*)(RCC_BASE_ADDR + OFF_APB2ENR))


#define GPIOA_BASE_ADDR     0x48000000


#define OFF_MODER           0x00      // GPIOx_MODER   (GPIO port mode register)
#define OFF_OTYPER          0x04      // GPIOx_OTYPER  (GPIO port output type register)
#define OFF_OSPEEDR         0x08      // GPIOx_OSPEEDR (GPIO port output speed register)
#define OFF_PUPDR           0x0C      // GPIOx_PUPDR   (GPIO port pull-up/pull-down register)
#define OFF_IDR             0x10      // GPIOx_IDR     (GPIO port input data register)
#define OFF_ODR             0x14      // GPIOx_ODR     (GPIO port output data register)
#define OFF_BSRR            0x18      // GPIOx_BSRR    (GPIO port bit set/reset register)
#define OFF_LCKR            0x1C      // GPIOx_LCKR    (GPIO port configuration lock register)
#define OFF_AFRL            0x20      // GPIOx_AFRL    (GPIO alternate function low register)
#define OFF_AFRH            0x24      // GPIOx_AFRH    (GPIO alternate function high register)


#define USART1_BASE_ADDR    0x40013800
#define USART2_BASE_ADDR    0x40004400


#define OFF_SR              0x1C     // USART_SR   (USART status register)
#define OFF_RDR             0x24     // USART_RDR  (USART received data register)
#define OFF_TDR             0x28     // USART_TDR  (USART transmitted data register)
#define OFF_BRR             0x0C     // USART_BRR  (USART baud rate register)
#define OFF_CR1             0x00     // USART_CR1  (USART control register 1)
#define OFF_CR2             0x04     // USART_CR2  (USART control register 2)
#define OFF_CR3             0x08     // USART_CR3  (USART control register 3)


#define UART_BASECLK        OS_FSYS / 1  // USART1/2 runs on APB1 clock frequency
#define GPIO_BASE_ADDR      GPIOA_BASE_ADDR
#define USART_BASE_ADDR     USART2_BASE_ADDR
#define GPIO_UART_TX_BIT    2           // USART2_TX is on PA2
#define GPIO_UART_RX_BIT    3           // USART2_RX is on PA3
#define USART_IRQ           USART2_IRQn // USART2 interrupt number

#define GPIO_MODER          (*(volatile uint32_t*)(GPIO_BASE_ADDR + OFF_MODER))
#define GPIO_AFRL           (*(volatile uint32_t*)(GPIO_BASE_ADDR + OFF_AFRL))
#define GPIO_AFRH           (*(volatile uint32_t*)(GPIO_BASE_ADDR + OFF_AFRH))

#define USART_SR            (*(volatile uint32_t*)(USART_BASE_ADDR + OFF_SR))
#define USART_RDR           (*(volatile uint32_t*)(USART_BASE_ADDR + OFF_RDR))
#define USART_TDR           (*(volatile uint32_t*)(USART_BASE_ADDR + OFF_TDR))
#define USART_BRR           (*(volatile uint32_t*)(USART_BASE_ADDR + OFF_BRR))
#define USART_CR1           (*(volatile uint32_t*)(USART_BASE_ADDR + OFF_CR1))
#define USART_CR2           (*(volatile uint32_t*)(USART_BASE_ADDR + OFF_CR2))
#define USART_CR3           (*(volatile uint32_t*)(USART_BASE_ADDR + OFF_CR3))

#define USART_RXNE              5        // Read data register not empty flag (SR)
#define USART_TC                6        // Transmission complete flag (SR)
#define USART_TXE               7        // Transmit data register empty flag (Does not indiate that the byte has already been sent, just indicates that it has been copied to the shift register)
#define USART_TXEIE             7        // Transmit data register empty interrupt enable
#define USART_RXNEIE            5        // Read data register not empty interrupt enable (CR1)
#define USART_RX_ERROR_FLAGS    0xB      // Overrun error, noise error, framing error, Parity error flags (SR)  
#define USART_TCIE              6        // Transmission complete interrupt enable (CR1)

