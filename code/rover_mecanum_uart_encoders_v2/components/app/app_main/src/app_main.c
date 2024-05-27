#include <app_main.h>

#include <stdio.h>

#include <usart.h>
#include <gpio.h>
#include <cmsis_os.h>

#include <lwrb/lwrb.h>
#include <lwpkt/lwpkt.h>

#include <cJSON.h>

#include <uart_rb.h>

#define HUART huart6

#define OS_MS_TO_TICKS(x) (x * osKernelGetTickFreq() / 1000)
#define OS_TICKS_TO_MS(x) (x / osKernelGetTickFreq() * 1000)

static lwpkt_t uart_rb_pkt;

static uint8_t uart_rb_tx_rb_data[128], uart_rb_rx_rb_data[128];
static uart_rb_t uart_rb = {0};

static void pkt_evt_fn(lwpkt_t *pkt, lwpkt_evt_type_t evt_type){
    switch (evt_type)
    {
    case LWPKT_EVT_PKT:
        //printf("Got a valid packet: (%.*s)\r\n", lwpkt_get_data_len(pkt), (char*)lwpkt_get_data(pkt));
        printf("p\r\n");
        break;
    case LWPKT_EVT_TIMEOUT:
        printf("timeout\r\n");
        break;
    }
}

osSemaphoreId_t pkt_sem_id = NULL;
osSemaphoreAttr_t pkt_sem_attr = {
    .name = "pkt sem"
};

static void uart_rx_evt_cb(UART_HandleTypeDef *huart, uint16_t Pos){
    if (huart->Instance == HUART.Instance)
    {
        uart_rb_rx_evt_cb(&uart_rb, Pos);
        while (lwrb_get_full(&uart_rb.rx_rb) > 0) lwpkt_process(&uart_rb_pkt, OS_TICKS_TO_MS(osKernelGetTickCount()));
    }
}

static void uart_tx_tc_cb(UART_HandleTypeDef *huart){
    if (huart->Instance == HUART.Instance)
    {
        uart_rb_tx_tc_cb(&uart_rb);
    }
}

osThreadId_t app_main_thread_id = NULL;
const osThreadAttr_t app_main_thread_attr = {
    .name = "app main thread",
    .stack_size = 4 * 256,
    .priority = osPriorityNormal
};
void app_main_thread_fun(void *param){
    UNUSED(param);

    HAL_UART_RegisterRxEventCallback(&HUART, uart_rx_evt_cb);
    HAL_UART_RegisterCallback(&HUART, HAL_UART_TX_COMPLETE_CB_ID, uart_tx_tc_cb);

    uart_rb_init(&uart_rb, &HUART, uart_rb_rx_rb_data, sizeof(uart_rb_rx_rb_data), uart_rb_tx_rb_data, sizeof(uart_rb_tx_rb_data));
    uart_rb_start(&uart_rb);

    lwpkt_init(&uart_rb_pkt, &uart_rb.tx_rb, &uart_rb.rx_rb);
    lwpkt_set_evt_fn(&uart_rb_pkt, pkt_evt_fn);

    while (1)
    {
        uint32_t start_ticks = osKernelGetTickCount();

        char buf[50];
        int size = snprintf(buf, sizeof(buf), "current time[%lu]", OS_TICKS_TO_MS(start_ticks));

        if (size > 0) lwpkt_write(&uart_rb_pkt, buf, size);
        else printf("size <= 0(%i)\r\n", size);

        osDelayUntil(start_ticks + OS_MS_TO_TICKS(1000));
    }
}

void init_app_main(void){
    app_main_thread_id = osThreadNew(app_main_thread_fun, NULL, &app_main_thread_attr);

    printf("Initialized:\r\n");
    printf("app main.\r\n");
}
