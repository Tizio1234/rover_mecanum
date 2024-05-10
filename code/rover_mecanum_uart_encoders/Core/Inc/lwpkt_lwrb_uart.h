/*
 * lwpkt_lwrb_uart.h
 *
 *  Created on: Jan 11, 2024
 *      Author: tommaso
 */

#ifndef INC_LWPKT_LWRB_UART_H_
#define INC_LWPKT_LWRB_UART_H_

#include <lwpkt/lwpkt.h>

#define UART_RX_BUFFER_SIZE 512
#define UART_DMA_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 512
#define UART_RB_SIZE_QUEUE_SIZE 10

typedef struct {
	UART_HandleTypeDef* huart_p;
	lwpkt_evt_fn lwpkt_evt_fn;
} lwpkt_lwrb_uart_init_data_t;

void lwpkt_lwrb_uart_init(const lwpkt_lwrb_uart_init_data_t*);
void lwrb_uart_callback(uint16_t Size);
lwpktr_t uart_lwpkt_set_evt_fn(lwpkt_evt_fn uart_lwpkt_evt_fn);
lwpktr_t uart_lwpkt_write(const void* data, size_t len);

#endif /* INC_LWPKT_LWRB_UART_H_ */
