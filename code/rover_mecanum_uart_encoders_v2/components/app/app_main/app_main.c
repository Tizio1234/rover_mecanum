#include <app_main.h>

#include <stdio.h>

#include <usart.h>
#include <cmsis_os.h>

#include <lwrb/lwrb.h>
#include <lwpkt/lwpkt.h>

#define HUART huart6

#define OS_MS_TO_TICKS(x) (x * osKernelGetTickFreq() / 1000)
#define OS_TICKS_TO_MS(x) (x / osKernelGetTickFreq() * 1000)

static lwpkt_t pkt;
static lwrb_t pkt_tx_rb, pkt_rx_rb;
static uint8_t pkt_tx_rb_data[128], pkt_rx_rb_data[128];

static uint8_t pkt_uart_rx_dma_buf[128];

static void pkt_evt_fn(lwpkt_t *pkt, lwpkt_evt_type_t evt_type){
    switch (evt_type)
    {
    case LWPKT_EVT_PKT:
        printf("Got a valid packet: (%.*s)\r\n", lwpkt_get_data_len(pkt), (char*)lwpkt_get_data(pkt));
        break;
    case LWPKT_EVT_TIMEOUT:
        printf("lwpkt evt timeout\r\n");
        break;
    }
}

static void pkt_tx_rb_evt_fn(lwrb_t* buff, lwrb_evt_type_t type, lwrb_sz_t len){
    switch (type)
    {
    case LWRB_EVT_WRITE:
        lwrb_sz_t size = lwrb_get_linear_block_read_length(buff);
        //lwrb_write(&pkt_rx_rb, lwrb_get_linear_block_read_address(buff), size);
        HAL_UART_Transmit(&HUART, (uint8_t*)lwrb_get_linear_block_read_address(buff), size, HAL_MAX_DELAY);
        
        lwrb_skip(buff, size);
        size = lwrb_get_linear_block_read_length(buff);
        if (size > 0) {
            //lwrb_write(&pkt_rx_rb, lwrb_get_linear_block_read_address(buff), size);
            HAL_UART_Transmit(&HUART, (uint8_t*)lwrb_get_linear_block_read_address(buff), size, HAL_MAX_DELAY);
            lwrb_skip(buff, size);
        }
        break;
    }
}

osSemaphoreId_t pkt_uart_rx_sem_id;
osSemaphoreAttr_t pkt_uart_rx_sem_attr = {
    .name = "pkt uart rx sem attr"
};

static void pkt_uart_rx_evt_cb(UART_HandleTypeDef *huart, uint16_t Pos){
    if (huart->Instance == HUART.Instance)
    {
        static uint16_t last_pos = 0;
        if (Pos > last_pos)
        {
            lwrb_write(&pkt_rx_rb, &pkt_uart_rx_dma_buf[last_pos], Pos - last_pos);
        } else
        {
            lwrb_write(&pkt_rx_rb, &pkt_uart_rx_dma_buf[last_pos], sizeof(pkt_uart_rx_dma_buf) - last_pos);
            if (Pos > 0) lwrb_write(&pkt_rx_rb, pkt_uart_rx_dma_buf, Pos);
        }

		last_pos = Pos;

        osSemaphoreRelease(pkt_uart_rx_sem_id);
    }
}

osThreadId_t pkt_process_thread_id;
const osThreadAttr_t pkt_process_thread_attr = {
    .name = "app main thread",
    .stack_size = 4 * 256,
    .priority = osPriorityHigh
};
void pkt_process_thread_fun(void *param){
    UNUSED(param);
    while (1)
    {
        osSemaphoreAcquire(pkt_uart_rx_sem_id, osWaitForever);
        uint32_t ticks = osKernelGetTickCount();
        while (lwrb_get_full(&pkt_rx_rb) > 0) lwpkt_process(&pkt, OS_TICKS_TO_MS(ticks));
    }
}

osThreadId_t app_main_thread_id;
const osThreadAttr_t app_main_thread_attr = {
    .name = "app main thread",
    .stack_size = 4 * 256,
    .priority = osPriorityNormal
};
void app_main_thread_fun(void *param){
    UNUSED(param);

    lwrb_init(&pkt_tx_rb, pkt_tx_rb_data, sizeof(pkt_tx_rb_data));
    lwrb_init(&pkt_rx_rb, pkt_rx_rb_data, sizeof(pkt_rx_rb_data));
    lwpkt_init(&pkt, &pkt_tx_rb, &pkt_rx_rb);

    lwrb_set_evt_fn(&pkt_tx_rb, pkt_tx_rb_evt_fn);
    lwpkt_set_evt_fn(&pkt, pkt_evt_fn);

    pkt_uart_rx_sem_id = osSemaphoreNew(10, 0, &pkt_uart_rx_sem_attr);
    pkt_process_thread_id = osThreadNew(pkt_process_thread_fun, NULL, &pkt_process_thread_attr);

    HAL_UART_RegisterRxEventCallback(&HUART, pkt_uart_rx_evt_cb);
    HAL_UARTEx_ReceiveToIdle_DMA(&HUART, pkt_uart_rx_dma_buf, sizeof(pkt_uart_rx_dma_buf));

    while (1)
    {
        uint32_t start_ticks = osKernelGetTickCount();
        
        char buf[50];
        int size = snprintf(buf, sizeof(buf), "current time[%u]", OS_TICKS_TO_MS(start_ticks));

        if (size > 0)lwpkt_write(&pkt, buf, size);
        else printf("size < 0\r\n");

        osDelayUntil(start_ticks + OS_MS_TO_TICKS(1000));
    }
}

void init_app_main(void){
    app_main_thread_id = osThreadNew(app_main_thread_fun, NULL, &app_main_thread_attr);

    printf("Initialized:\r\n");
    printf("app main.\r\n");
}

