/*
 * lwpkt_lwrb_uart.c
 *
 *  Created on: Jan 11, 2024
 *      Author: tommaso
 */

#include <stdio.h>

#include <cmsis_os.h>
#include <stm32f4xx_hal.h>

#include <lwrb/lwrb.h>
#include <lwpkt/lwpkt.h>
#include <lwpkt_lwrb_uart.h>


lwpkt_t uart_lwpkt;

uint8_t uart_dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE];

lwrb_t uart_rx_buffer;
uint8_t uart_rx_data_buffer[UART_RX_BUFFER_SIZE];

lwrb_t uart_tx_buffer;
uint8_t uart_tx_data_buffer[UART_TX_BUFFER_SIZE];

/* Definitions for uart_rb_queue */
osMessageQueueId_t uart_rb_queueHandle;
const osMessageQueueAttr_t uart_rb_queue_attributes = {
  .name = "uart_rb_queue"
};

/* Definitions for uart_rx_rb_task */
osThreadId_t lwpkt_lwrb_uart_taskHandle;
const osThreadAttr_t lwpkt_lwrb_uart_task_attributes = {
  .name = "lwpkt_lwrb_uart_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

UART_HandleTypeDef* huart_p;


void lwpkt_lwrb_uart_task(void* argument);

void lwpkt_lwrb_uart_init(const lwpkt_lwrb_uart_init_data_t* init_data){
	lwpkt_lwrb_uart_taskHandle = osThreadNew(lwpkt_lwrb_uart_task, init_data, &lwpkt_lwrb_uart_task_attributes);
}

void uart_tx_rb_evt_fn(lwrb_t* buff, lwrb_evt_type_t type, lwrb_sz_t len){
	switch (type) {
		case LWRB_EVT_WRITE:
			lwrb_sz_t size = lwrb_get_linear_block_read_length(buff);
			HAL_UART_Transmit(huart_p, (uint8_t*)lwrb_get_linear_block_read_address(buff), size, HAL_MAX_DELAY);
			lwrb_skip(buff, size);
			size = lwrb_get_linear_block_read_length(buff);
			if (size > 0) {
					HAL_UART_Transmit(huart_p, (uint8_t*)lwrb_get_linear_block_read_address(buff), size, HAL_MAX_DELAY);
			}
			lwrb_skip(buff, size);

			break;
		default:
			break;
	}
}

void lwpkt_lwrb_uart_task(void* argument){
	lwpkt_lwrb_uart_init_data_t* init_data_p = (lwpkt_lwrb_uart_init_data_t*)argument;
	huart_p = init_data_p->huart_p;

	uart_rb_queueHandle = osMessageQueueNew(UART_RB_SIZE_QUEUE_SIZE, sizeof(uint16_t), &uart_rb_queue_attributes);

	lwrb_init(&uart_rx_buffer, uart_rx_data_buffer, UART_RX_BUFFER_SIZE);

	lwrb_init(&uart_tx_buffer, uart_tx_data_buffer, UART_TX_BUFFER_SIZE);
	lwrb_set_evt_fn(&uart_tx_buffer, uart_tx_rb_evt_fn);

	lwpkt_init(&uart_lwpkt, &uart_tx_buffer, &uart_rx_buffer);
	lwpkt_set_evt_fn(&uart_lwpkt, init_data_p->lwpkt_evt_fn);
	//osEventFlagsSet(lwpkt_eventsHandle, lwpkt_init_event_flag);

	HAL_UARTEx_ReceiveToIdle_DMA(huart_p, uart_dma_rx_buffer, UART_DMA_RX_BUFFER_SIZE);
	/* Infinite loop */
	for(;;)
	{
		uint16_t Size;
		osMessageQueueGet(uart_rb_queueHandle, &Size, NULL, osWaitForever);

		static uint16_t pos = 0;
		lwrb_write(&uart_rx_buffer, &uart_dma_rx_buffer[pos], Size >= pos ? Size - pos : Size - pos + UART_DMA_RX_BUFFER_SIZE);
		pos = Size;
		lwpkt_process(&uart_lwpkt, HAL_GetTick());
	}
}

void lwrb_uart_callback(uint16_t Size){
	osMessageQueuePut(uart_rb_queueHandle, &Size, 0, 0);
}

lwpktr_t uart_lwpkt_set_evt_fn(lwpkt_evt_fn uart_lwpkt_evt_fn){
	return lwpkt_set_evt_fn(&uart_lwpkt, uart_lwpkt_evt_fn);
}

lwpktr_t uart_lwpkt_write(const void* data, size_t len){
	return lwpkt_write(&uart_lwpkt, data, len);
}
