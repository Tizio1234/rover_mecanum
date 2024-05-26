#include <uart_rb.h>
#include <cmsis_os.h>

#include <stdio.h>

static uart_rb_err_t uart_rb_tx_transfer(uart_rb_t *uart_rb)
{
    if (uart_rb->tx_current_size == 0)
    {
        if ((uart_rb->tx_current_size = lwrb_get_linear_block_read_length(&uart_rb->tx_rb)) > 0){
            HAL_UART_Transmit_DMA(uart_rb->huart, lwrb_get_linear_block_read_address(&uart_rb->tx_rb), uart_rb->tx_current_size);
            lwrb_skip(&uart_rb->tx_rb, uart_rb->tx_current_size);
        }
    }

    return UART_RB_OK;
}

uart_rb_err_t uart_rb_rx_evt_cb(uart_rb_t *uart_rb, uint16_t pos)
{
    lwrb_sz_t size = (pos > uart_rb->rx_last_pos) ? (pos - uart_rb->rx_last_pos) : (uart_rb->rx_rb_data_size + pos - uart_rb->rx_last_pos);
    uart_rb->rx_last_pos = pos;

    lwrb_advance(&uart_rb->rx_rb, size);

    return UART_RB_OK;
}

uart_rb_err_t uart_rb_tx_tc_cb(uart_rb_t *uart_rb)
{
    uart_rb->tx_current_size = 0;
    uart_rb_tx_transfer(uart_rb);

    return UART_RB_OK;
}

static void uart_rb_txrb_evt_fn(lwrb_t *buff, lwrb_evt_type_t evt, lwrb_sz_t bp){
    switch (evt)
    {
    case LWRB_EVT_WRITE:
        uart_rb_tx_transfer(lwrb_get_arg(buff));
        break;
    }
}

uart_rb_err_t uart_rb_init(uart_rb_t *uart_rb,
                 UART_HandleTypeDef *huart,
                 uint8_t *rx_rb_data, lwrb_sz_t rx_rb_data_size,
                 uint8_t *tx_rb_data, lwrb_sz_t tx_rb_data_size)
{
    uart_rb->rx_rb_data_size = rx_rb_data_size;
    uart_rb->tx_rb_data_size = tx_rb_data_size;

    uart_rb->rx_rb_data = rx_rb_data;
    uart_rb->tx_rb_data = tx_rb_data;

    uart_rb->huart = huart;

    /* Init ring buffers */
    lwrb_init(&uart_rb->rx_rb, uart_rb->rx_rb_data, uart_rb->rx_rb_data_size);
    lwrb_init(&uart_rb->tx_rb, uart_rb->tx_rb_data, uart_rb->tx_rb_data_size);

    lwrb_set_arg(&uart_rb->rx_rb, uart_rb);
    lwrb_set_arg(&uart_rb->tx_rb, uart_rb);

    lwrb_set_evt_fn(&uart_rb->tx_rb, uart_rb_txrb_evt_fn);

    uart_rb->rx_last_pos = 0;
    uart_rb->tx_current_size = 0;

    uart_rb->initialized = true;

    return UART_RB_OK;
}

uart_rb_err_t uart_rb_deinit(uart_rb_t *uart_rb)
{
    lwrb_reset(&uart_rb->rx_rb);
    lwrb_reset(&uart_rb->tx_rb);

    uart_rb->initialized = false;

    return UART_RB_OK;
}


uart_rb_err_t uart_rb_start(uart_rb_t *uart_rb)
{
    HAL_UARTEx_ReceiveToIdle_DMA(uart_rb->huart, uart_rb->rx_rb_data, uart_rb->rx_rb_data_size);

    uart_rb->started = true;
    
    return UART_RB_OK;
}

uart_rb_err_t uart_rb_stop(uart_rb_t *uart_rb)
{
    uart_rb->started = false;
    
    return UART_RB_OK;
}
