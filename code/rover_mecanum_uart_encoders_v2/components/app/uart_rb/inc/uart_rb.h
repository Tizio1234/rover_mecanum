#include <lwrb/lwrb.h>
#include <usart.h>

#include <stdbool.h>

typedef struct
{
    lwrb_t rx_rb;
    lwrb_t tx_rb;

    uint8_t *rx_rb_data;
    uint8_t *tx_rb_data;

    lwrb_sz_t rx_rb_data_size;
    lwrb_sz_t tx_rb_data_size;

    UART_HandleTypeDef *huart;

    uint16_t rx_last_pos;

    uint16_t tx_current_size;

    bool initialized;
    bool started;
} uart_rb_t;

typedef enum {
    UART_RB_OK = 0,
    UART_RB_FAIL,
    UART_RB_RB_FAIL,
    UART_RB_DMA_FAIL,
    UART_RB_UART_FAIL
} uart_rb_err_t;

/* Callbacks to be called by user */
uart_rb_err_t uart_rb_rx_evt_cb(uart_rb_t *uart_rb, uint16_t pos);
uart_rb_err_t uart_rb_tx_tc_cb(uart_rb_t *uart_rb);

uart_rb_err_t uart_rb_init(uart_rb_t *uart_rb,
                 UART_HandleTypeDef *huart,
                 uint8_t *rx_rb_data, lwrb_sz_t rx_rb_data_size,
                 uint8_t *tx_rb_data, lwrb_sz_t tx_rb_data_size);
uart_rb_err_t uart_rb_deinit(uart_rb_t *uart_rb);

uart_rb_err_t uart_rb_start(uart_rb_t *uart_rb);
uart_rb_err_t uart_rb_stop(uart_rb_t *uart_rb);
