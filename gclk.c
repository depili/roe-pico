#include "roe-pico.h"


// program offset and state machine ID
static int gclock_offset = -1;
static int gclock_sm = -1;
volatile uint8_t addr = 0;
volatile bool vsyncRequest = false;

static inline void incrementAddress() {
	addr = (addr + 1) % SCANLINES;
}

// Assume all address pins start from PIN_A
static inline void writeAddress() {
	const uint32_t mask = 0x0F << PIN_A;
	uint32_t set = addr << PIN_A;

	gpio_put_masked(mask, set);
}

inline void vsync() {
	vsyncRequest = true;
	while(vsyncRequest) {
	}
}

// interrupt handler
void gclock_handler(void) {
	pio_interrupt_clear(pio0, 0);

	if (vsyncRequest) {
		sendCommand(CMD_VSYNC);
		vsyncRequest = false;
		addr = SCANLINES - 1;
	}

	incrementAddress();
	writeAddress();

	dataDelay();
	dataDelay();
	dataDelay();

	// tell PIO to start another cycle
	pio_sm_put_blocking(pio0, gclock_sm, GCLOCK_COUNT-1);
}

void gclock_init() {
	// add program from gclock.pio.h to PIO0
	gclock_offset = pio_add_program(pio0, &gclock_program);

	// claim a state-machine
	gclock_sm = pio_claim_unused_sm(pio0, true);

	// set the pin as PIO pin
	pio_gpio_init(pio0, PIN_OE);
	// direction is output
	pio_sm_set_consecutive_pindirs(pio0, gclock_sm, PIN_OE, 1, true);

	// get default config (in gclock.pio.h by PioAsm)
	pio_sm_config c = gclock_program_get_default_config(gclock_offset);

	// see GCLOCK_DIV above
	sm_config_set_clkdiv_int_frac(&c, GCLOCK_DIV, 0);

	// the clock is output from side-set pin
	sm_config_set_sideset_pins(&c, PIN_OE);

	// set interrupt handler
	irq_set_exclusive_handler(PIO0_IRQ_0, &gclock_handler);
	irq_set_enabled(PIO0_IRQ_0, true);

	// enable in PIO
	pio_set_irq0_source_enabled(pio0, pis_interrupt0 + gclock_sm, true);

	// Load config and jump to start
	pio_sm_init(pio0, gclock_sm, gclock_offset, &c);

	// start .. our PIO starts by stalling on FIFO
	pio_sm_set_enabled(pio0, gclock_sm, true);

	// fire off GCLOCK_COUNT pulses
	pio_sm_put_blocking(pio0, gclock_sm, GCLOCK_COUNT-1);
}
