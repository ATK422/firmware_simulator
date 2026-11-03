// pico-examples hello_usb, extended for the emulator's USB host model: stdio over TinyUSB CDC ACM. It waits for
// the host to open the port (DTR), greets it, then echoes each character it receives on a line of its own. A 'b'
// reboots into BOOTSEL through reset_usb_boot, as `picotool reboot -u` does through the reset interface.
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    printf("Hello, USB!\n");
    int n = 0;
    while (true) {
        int c = getchar_timeout_us(100 * 1000);
        if (c == 'b') {
            printf("rebooting to BOOTSEL\n");
            sleep_ms(10);
            reset_usb_boot(0, 0);
        } else if (c >= 0) {
            printf("echo %c (%d)\n", c, ++n);
        }
    }
}
