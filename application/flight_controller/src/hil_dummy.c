#include "hil_dummy.h"

//K_THREAD_DEFINE(uart_thread, 1024, uart_thread, NULL, NULL, NULL, 5, 0, 0); 

void uart_thread(fjalar_t *fjalar, void *p2, void *p1) {
    const struct device *uart_sensor = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart2));

    while true {
    /* direct UART print */
    uart_poll_out(uart_sensor, 'hello world!');
    uart_poll_out(uart_sensor, '\n');
    }
}