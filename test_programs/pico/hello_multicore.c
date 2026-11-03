// pico-examples multicore/hello_multicore, plus checks of the other SIO blocks the second core relies on: a
// spinlock-protected counter shared by both cores, the per-core hardware divider and an interpolator.
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/divider.h"
#include "hardware/interp.h"
#include "hardware/sync.h"

#define FLAG_VALUE 123
#define INCREMENTS 1000

static spin_lock_t *lock;
static volatile uint32_t shared_counter;

static void count(void) {
    for (int i = 0; i < INCREMENTS; i++) {
        uint32_t save = spin_lock_blocking(lock);
        shared_counter++;
        spin_unlock(lock, save);
    }
}

void core1_entry() {
    multicore_fifo_push_blocking(FLAG_VALUE);

    uint32_t g = multicore_fifo_pop_blocking();

    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 1!\n");
    else
        printf("Its all gone well on core 1!\n");

    divmod_result_t r = hw_divider_divmod_s32(-1000, 7);
    printf("core 1 divider: %d rem %d\n", (int)to_quotient_s32(r), (int)to_remainder_s32(r));

    count();
    multicore_fifo_push_blocking(1);

    while (1)
        tight_loop_contents();
}

int main() {
    stdio_init_all();
    printf("Hello, multicore!\n");

    lock = spin_lock_init(spin_lock_claim_unused(true));

    multicore_launch_core1(core1_entry);

    // Wait for it to start up
    uint32_t g = multicore_fifo_pop_blocking();

    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 0!\n");
    else {
        multicore_fifo_push_blocking(FLAG_VALUE);
        printf("It's all gone well on core 0!\n");
    }

    count();
    multicore_fifo_pop_blocking();
    printf("shared counter: %u\n", (unsigned)shared_counter);

    // Interpolator 0, lane 0: add the accumulator shifted and masked to a base
    interp_config cfg = interp_default_config();
    interp_config_set_shift(&cfg, 4);
    interp_config_set_mask(&cfg, 0, 7);
    interp_set_config(interp0, 0, &cfg);
    interp0->accum[0] = 0x1234;
    interp0->base[0] = 0x1000;
    printf("interp: %08x\n", (unsigned)interp0->peek[0]);

    while (true) {
        __wfi();
    }
}
