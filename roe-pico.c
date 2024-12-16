/**
 * Copyright (c) 2024 Vesa-Pekka Palmu.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "roe-pico.h"
#include "manda.h"

pixel_t bitmap[88][88];
pixel_t blank[44];
pixel_t scanline_data[44];

uint8_t fastline_data[22][16];
uint8_t fastline_blank[22][16];

uint8_t allPins[] = { PIN_R1, PIN_R2, PIN_G1, PIN_G2, PIN_B1, PIN_B2, PIN_A, PIN_B, PIN_C, PIN_D, PIN_LE, PIN_DCLK };
uint8_t addrPins[] = { PIN_A, PIN_B, PIN_C, PIN_D };
uint8_t dataPins[] = { PIN_R1, PIN_G1, PIN_B1, PIN_R2, PIN_G2, PIN_B2 };

const uint16_t configRegister1 = CURRENT_GAIN_LOW | MUX_11;
const uint16_t configRegister2 = CONF2_DEFAULT | DIM_COMP_5;

inline void set_red1(uint8_t pixel[], uint16_t c) {
	for (int bit = 0; bit < 16; bit++) {
		pixel[bit] = (pixel[bit] & 0b00111110) | bitRead(c, bit);
	}
}

inline void set_red2(uint8_t pixel[], uint16_t c) {
	for (int bit = 0; bit < 16; bit++) {
		pixel[bit] = (pixel[bit] & 0b00111101) | (bitRead(c, bit) << 1);
	}
}

inline void set_green1(uint8_t pixel[], uint16_t c) {
	for (int bit = 0; bit < 16; bit++) {
		pixel[bit] = (pixel[bit] & 0b00111011) | (bitRead(c, bit) << 2);
	}
}

inline void set_green2(uint8_t pixel[], uint16_t c) {
	for (int bit = 0; bit < 16; bit++) {
		pixel[bit] = (pixel[bit] & 0b00110111) | (bitRead(c, bit) << 3);
	}
}

inline void set_blue1(uint8_t pixel[], uint16_t c) {
	for (int bit = 0; bit < 16; bit++) {
		pixel[bit] = (pixel[bit] & 0b00101111) | (bitRead(c, bit) << 4);
	}
}

inline void set_blue2(uint8_t pixel[], uint16_t c) {
	for (int bit = 0; bit < 16; bit++) {
		pixel[bit] = (pixel[bit] & 0b00011111) | (bitRead(c, bit) << 5);
	}
}

inline void zeroDataPins() {
	const uint32_t mask = 0x7F;
	gpio_put_masked(mask, 0);
}


inline void dataDelay() {
	// Should be 8ns per nop...
	asm volatile (
		"nop\n"
		"nop\n"
		"nop\n"
		"nop\n"
		);
}


void sendCommand(uint8_t length) {
	gpio_put(PIN_LE, 0);
	gpio_put(PIN_DCLK, 0);
	dataDelay();
	gpio_put(PIN_LE, 1);
	dataDelay();

	for (int i = 0; i < length; i++) {
		gpio_put(PIN_DCLK, 1);
		dataDelay();
		gpio_put(PIN_DCLK, 0);
		dataDelay();
	}
	gpio_put(PIN_LE, 0);
}

void sendReset() {
	uint32_t irq_save = save_and_disable_interrupts();
	restore_interrupts(irq_save);

	sendCommand(CMD_RESET);

	irq_save = save_and_disable_interrupts();
	restore_interrupts(irq_save);
}

void writeCommand1(uint16_t data) {
	// Write the pre-active
	for (uint i = 0; i < sizeof(dataPins); i++) {
		gpio_put(dataPins[i], 0);
	}

	sendCommand(CMD_PRE);

	// Write the command register
	for (int i = 0; i < CHAIN_BITS; i++) {
		if (i == (CHAIN_BITS - CMD_WRITE_CMD1)) {
			// Need to put LE high CMD_WRITE_CMD1 cycles before the end...
			gpio_put(PIN_LE, 1);
		}

		uint8_t bit = bitRead(configRegister1, 15 - (i % 16));

		for (uint j = 0; j < sizeof(dataPins); j++) {
			gpio_put(dataPins[j], bit);
		}

		gpio_put(PIN_DCLK, 1);
		dataDelay();
		gpio_put(PIN_DCLK, 0);
		dataDelay();
	}
	gpio_put(PIN_LE, 0);
}


void writeCommand2(uint16_t data) {
	// Write the pre-active
	for (uint i = 0; i < sizeof(dataPins); i++) {
		gpio_put(dataPins[i], 0);
	}

	sendCommand(CMD_PRE);

	// Write the command register
	for (int i = 0; i < CHAIN_BITS; i++) {
		if (i == (CHAIN_BITS - CMD_WRITE_CMD2)) {
			// Need to put LE high CMD_WRITE_CMD1 cycles before the end...
			gpio_put(PIN_LE, 1);
		}

		uint8_t bit = bitRead(configRegister1, 15 - (i % 16));

		for (uint j = 0; j < sizeof(dataPins); j++) {
			gpio_put(dataPins[j], bit);
		}

		gpio_put(PIN_DCLK, 1);
		dataDelay();
		gpio_put(PIN_DCLK, 0);
		dataDelay();
	}
	gpio_put(PIN_LE, 0);
}

inline void writeChain(pixel_t data[]) {
	int data_len = CHAIN_BITS;

	for (int ic = 0; ic < CHAIN; ic++) {
		const pixel_t *p1 = &data[ic];
		const pixel_t *p2 = &data[ic + CHAIN];

		for (int bit = 15; bit >= 0; bit--) {
			if (data_len == 1) {
				gpio_put(PIN_LE, 1);
			}
			gpio_put(PIN_R1, bitRead(p1->r, bit));
			gpio_put(PIN_G1, bitRead(p1->g, bit));
			gpio_put(PIN_B1, bitRead(p1->b, bit));

			gpio_put(PIN_R2, bitRead(p2->r, bit));
			gpio_put(PIN_G2, bitRead(p2->g, bit));
			gpio_put(PIN_B2, bitRead(p2->b, bit));

			gpio_put(PIN_DCLK, 0);
			dataDelay();
			gpio_put(PIN_DCLK, 1);

			data_len--;
		}
	}
	gpio_put(PIN_LE, 0);
	dataDelay();
}

inline void writeFastChain(uint8_t data[22][16]) {
	int data_len = CHAIN_BITS;

	for (int ic = 0; ic < CHAIN; ic++) {
		for (int bit = 15; bit >= 0; bit--) {
			if (data_len == 1) {
				gpio_put(PIN_LE, 1);
			}

			gpio_put_masked(DATA_MASK, data[ic][bit]);
			dataDelay();
			dataDelay();
			gpio_put(PIN_DCLK, 1);

			data_len--;
		}
	}
	gpio_put(PIN_LE, 0);
}

void horizontalLine(pixel_t *color, uint8_t line) {
	// Which chain are we on?
	uint8_t lower = line / 44;
	// First or last half of screen?
	uint8_t lastHalf = (line % 44) / 22;
	// First or last quadrant of the screen?
	uint8_t lastQuadrant = (line % 22) / 11;

	uint8_t targetScanline = line % 11;

	for (int i = 0; i < 44; i++) {
		if ((i / 22 == !lower) && (i % 22) / 11 == !lastHalf) {
			scanline_data[i].r = color->r;
			scanline_data[i].g = color->g;
			scanline_data[i].b = color->b;
		} else {
			scanline_data[i].r = 0;
			scanline_data[i].g = 0;
			scanline_data[i].b = 0;
		}
	}


	zeroDataPins();

	dataDelay();

	for (int scanline = 0; scanline < SCANLINES; scanline++) {
		// Send all 16 leds per IC per scanline
		if (scanline != targetScanline) {
			for (int led = 0; led < 16; led++) {
				writeChain(blank);
			}
		} else if (!lastQuadrant) {
			for (int led = 0; led < 8; led++) {
				writeChain(blank);
			}
			for (int led = 8; led < 16; led++) {
				writeChain(scanline_data);
			}
		} else {
			for (int led = 0; led < 8; led++) {
				writeChain(scanline_data);
			}
			for (int led = 8; led < 16; led++) {
				writeChain(blank);
			}
		}
	}
}

void fastHorizontalLine(pixel_t *color, uint8_t line) {
	// Which chain are we on?
	uint8_t lower = line / 44;
	// First or last half of screen?
	uint8_t lastHalf = (line % 44) / 22;
	// First or last quadrant of the screen?
	uint8_t lastQuadrant = (line % 22) / 11;

	uint8_t targetScanline = line % 11;

	for (int i = 0; i < 22; i++) {
		for (int bit = 0; bit < 16; bit++) {
			fastline_data[i][bit] = 0;
		}
		if (i / 11 != lastHalf) {
			if (!lower) {
				set_red1(fastline_data[i], color->r);
				set_green1(fastline_data[i], color->g);
				set_blue1(fastline_data[i], color->b);
			} else {
				set_red2(fastline_data[i], color->r);
				set_green2(fastline_data[i], color->g);
				set_blue2(fastline_data[i], color->b);
			}
		}
	}

	zeroDataPins();
	dataDelay();

	for (int scanline = 0; scanline < SCANLINES; scanline++) {
		// Send all 16 leds per IC per scanline
		if (scanline != targetScanline) {
			for (int led = 0; led < 16; led++) {
				writeFastChain(fastline_blank);
			}
		} else if (!lastQuadrant) {
			for (int led = 0; led < 8; led++) {
				writeFastChain(fastline_blank);
			}
			for (int led = 8; led < 16; led++) {
				writeFastChain(fastline_data);
			}
		} else {
			for (int led = 0; led < 8; led++) {
				writeFastChain(fastline_data);
			}
			for (int led = 8; led < 16; led++) {
				writeFastChain(fastline_blank);
			}
		}
	}
}

void fillPanel(pixel_t *color) {
	pixel_t data[CHAIN * 2];

	for (int i = 0; i < CHAIN * 2; i++) {
		data[i].r = color->r;
		data[i].g = color->g;
		data[i].b = color->b;
	}

	zeroDataPins();

	dataDelay();


	for (int scanline = 0; scanline < SCANLINES; scanline++) {
		// Send all 16 leds per IC per scanline
		for (int led = 0; led < 16; led++) {
			// Send one 16bit led register to all controllers
			writeChain(data);
		}
	}
}

void fastFillPanel(pixel_t *color) {
	uint8_t data[CHAIN][16];

	for (int i = 0; i < CHAIN; i++) {
		set_red1(data[i], color->r);
		set_green1(data[i], color->g);
		set_blue1(data[i], color->b);

		set_red2(data[i], color->r);
		set_green2(data[i], color->g);
		set_blue2(data[i], color->b);
	}

	zeroDataPins();

	dataDelay();


	for (int scanline = 0; scanline < SCANLINES; scanline++) {
		// Send all 16 leds per IC per scanline
		for (int led = 0; led < 16; led++) {
			// Send one 16bit led register to all controllers
			writeFastChain(data);
		}
	}
}

// Perform initialisation
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
	// A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
	// so we can use normal GPIO functionality to turn the led on and off
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
	// For Pico W devices we need to initialise the driver etc
	return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
	// Just set the GPIO on or off
	gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
	// Ask the wifi "driver" to set the GPIO on or off
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

int roe_init() {
	for (uint i = 0; i < sizeof(allPins); i++) {
		gpio_init(allPins[i]);
		gpio_set_dir(allPins[i], GPIO_OUT);
	}

	gpio_put(PIN_OE, 1);

	for (int i = 0; i < CHAIN; i++) {
		blank[i].r = 0;
		blank[i].g = 0;
		blank[i].b = 0;
	}

	for (int i = 0; i < 22; i++) {
		for (int bit = 0; bit < 16; bit++) {
			fastline_blank[i][bit] = 0;
		}
	}


	/*
	for (int i = 0; i < 88; i++) {
		for (int j = 0; j < 88; j++) {
			int index = (i * 88 * 3) + (j * 3);
			bitmap[i][j].r = sRGBtoLinear(manda_map[index + 2]);
			bitmap[i][j].g = sRGBtoLinear(manda_map[index + 1]);
			bitmap[i][j].b = sRGBtoLinear(manda_map[index]);
		}
	}


	*/
	// Speed doesn't matter on teensy..
	// Serial.begin(9600);
	// t1.begin(gclkProcess, GCLK_INTERVAL);

	// negative timeout means exact delay (rather than delay between callbacks)
	// if (!add_repeating_timer_us(-1, gclk_timer_callback, NULL, &gclk_timer)) {
	//	printf("Failed to add glck timer\n");
	//	return 1;
	// }
	return PICO_OK;
}

// Function to convert a single 8-bit sRGB value to linear
uint16_t sRGB_to_linear(uint8_t srgb) {
	float normalized = srgb / 255.0f;  // Normalize to [0, 1]

	// Convert from gamma corrected sRGB to linear RGB
	if (normalized <= 0.04045f) {
		normalized /= 12.92f;
	} else {
		normalized = powf((normalized + 0.055f) / 1.055f, 2.4f);
	}

	// Scale to 16-bit range [0, 65535]
	return (uint16_t)(normalized * MAX_LINEAR);
}

void getChainData(pixel_t bitmap[88][88], uint scanline, uint led, pixel_t data[44]) {
	// Row delta from current led
	uint8_t ledRow = (led / 8) ? 0 : 11;
	// Led delta in columns
	uint8_t ledColumn = led % 8;

	int row = scanline + ledRow;

	if (ledRow == 11) {
		ledColumn = (7 - ledColumn);
	}

	// Get every 8th pixel
	int start = 33;

	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 11; i++) {
			data[start+i] = bitmap[row][(8 * i) + ledColumn];
		}

		row += 22;
		start -= 11;
	}
}

void sendBitmap(pixel_t bitmap[88][88]) {
	pixel_t data[44];

	zeroDataPins();
	dataDelay();

	for (int scanline = 0; scanline < SCANLINES; scanline++) {
		// Send all 16 leds per IC per scanline
		for (int led = 0; led < 16; led++) {
			getChainData(bitmap, scanline, led, data);
			writeChain(data);
		}
	}
}

int main() {
	stdio_init_all();

	int rc = pico_led_init();
	hard_assert(rc == PICO_OK);

	for (int i = 0; i < 88; i++) {
		for (int j = 0; j < 88; j++) {
			int index = (i * 88 * 3) + (j * 3);
			bitmap[i][j].r = sRGB_to_linear(manda_map[index + 2]);
			bitmap[i][j].g = sRGB_to_linear(manda_map[index + 1]);
			bitmap[i][j].b = sRGB_to_linear(manda_map[index]);
		}
	}



	rc = roe_init();
	hard_assert(rc == PICO_OK);

	printf("Sending reset\n");
	sendReset();

	gclock_init();

	multicore_launch_core1(core1_main);

	while (true) {

	}
}

void core1_main() {
	while (true) {
		pico_set_led(true);

		printf("Writing command register\n");
		writeCommand1(configRegister1);
		sleep_ms(10);

		// writeCommand2(configRegister2);
		// sleep_ms(10);


		pixel_t color;

		color.r = 0xFFFF;
		color.g = 0;
		color.b = 0;

		fastFillPanel(&color);
		vsync();
		sleep_ms(500);


		color.r = 0;
		color.g = 0xFFFF;
		color.b = 0;

		fastFillPanel(&color);
		vsync();
		sleep_ms(500);

		color.r = 0;
		color.g = 0;
		color.b = 0xFFFF;

		fastFillPanel(&color);
		vsync();
		sleep_ms(500);

		color.r = 0xFFFF;
		color.g = 0x00FF;
		color.b = 0xF0FF;

		for (uint8_t line = 0; line < 88; line++) {
			fastHorizontalLine(&color, line);
			vsync();
		}

		for (uint8_t c = 0xFF; c != 0; c--) {
			uint16_t val = sRGB_to_linear(c);
			if (c < 0x0003) {
				break;
			}
			color.r = val;
			color.g = val;
			color.b = val;

			fastFillPanel(&color);
			vsync();
		}

		sendBitmap(bitmap);
		vsync();
		sleep_ms(5000);

		pico_set_led(false);
	}
}
