/* 
* uart.h
* Created: 23/03/21 01:15:58 PM
* Author: Bitboot Team
*/ 

#ifndef UART_H_
#define UART_H_

// UART Init
void uart_init (unsigned int ubbr_value);

// This function sends a character over the UART 
void uart_tx(char data);

// This function sends a string over the UART 
void uart_tx_string(char *data);

#endif