// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------ //
// gclock //
// ------ //

#define gclock_wrap_target 0
#define gclock_wrap 4

static const uint16_t gclock_program_instructions[] = {
            //     .wrap_target
    0x80a0, //  0: pull   block           side 0     
    0xa027, //  1: mov    x, osr          side 0     
    0xa042, //  2: nop                    side 0     
    0x1042, //  3: jmp    x--, 2          side 1     
    0xc000, //  4: irq    nowait 0        side 0     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program gclock_program = {
    .instructions = gclock_program_instructions,
    .length = 5,
    .origin = -1,
};

static inline pio_sm_config gclock_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + gclock_wrap_target, offset + gclock_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}
#endif
