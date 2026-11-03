#include <ctype.h>
#include <stdio.h>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    dual_usb_init();
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    printf("titan dual stdio up\n");
    while (true) {
        unsigned char buf[64];
        int n = secondary_usb_in_chars(buf, sizeof buf);
        for (int i = 0; i < n; i++) {
            buf[i] = (unsigned char) toupper(buf[i]);
        }
        if (n > 0) {
            secondary_usb_out_chars(buf, n);
        }
        sleep_ms(1);
    }
}
