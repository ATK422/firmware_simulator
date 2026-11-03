// Same as pico-examples uart/hello_uart, except that it sleeps instead of returning from main.
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/sync.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

int main() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Send out a character without any conversions
    uart_putc_raw(UART_ID, 'A');
    // Send out a character but do CR/LF conversions
    uart_putc(UART_ID, 'B');
    // Send out a string, with CR/LF conversions
    uart_puts(UART_ID, " Hello, UART!\n");

    while (true) {
        __wfi();
    }
}
