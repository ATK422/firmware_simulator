// Reads the flash chip's unique ID like any pico-sdk program linking pico_unique_id: in a constructor before main,
// through flash_do_cmd (the RUID command sent by hand over the SSI, then boot2 called again to restore XIP). Reads
// it once more from main, prints both, and checks that code and data still run from flash afterwards.
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

static const char flash_text[] = "still reading flash";

int main() {
    stdio_init_all();
    char id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(id, sizeof id);
    printf("unique id %s\n", id);
    uint8_t again[FLASH_UNIQUE_ID_SIZE_BYTES];
    flash_get_unique_id(again);
    printf("again ");
    for (int i = 0; i < FLASH_UNIQUE_ID_SIZE_BYTES; ++i)
        printf("%02X", again[i]);
    printf("\n%s\n", flash_text);
    while (true) {
        __wfi();
    }
}
