#ifndef ROEPICO_H
#define ROEPICO_H

#include <stdio.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "gclock.pio.h"

// Arduino bit operation macros
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

// Pins used
#define PIN_R2  0
#define PIN_R1  1
#define PIN_G2  2
#define PIN_G1  3
#define PIN_B2  4
#define PIN_B1  5
#define PIN_DCLK 6
#define PIN_LE 7

#define PIN_OE 8

#define PIN_A 10
#define PIN_B 11
#define PIN_C 12
#define PIN_D 13
#define DATA_MASK   0x7F

// if we set divider as 125MHz / 50k we get 2.5kHz
// output clock is 2 PIO cycles, so this would be 1.25kHz
// adjust (this is slow for demo purposes).. max div is 64k
#define GCLOCK_DIV 6

// this is how many times we pulse the pin before IRQ
#define GCLOCK_COUNT 513

// Panel size
#define SCANLINES 11
#define CHAIN 22
#define PIXELS 88
#define CHAIN_BITS 16 * CHAIN

// Driver command pulse lengths
#define CMD_VSYNC 2
#define CMD_WRITE_CMD1 4
#define CMD_ERROR_DETECT 7
#define CMD_WRITE_CMD2 8
#define CMD_RESET 10
#define CMD_PRE 14

#define CURRENT_GAIN_DEFAULT 0b0000000000101011
#define CURRENT_GAIN_LOW     0b0000000000000000
#define GCLK_MULT            0b0000000001000000
#define PWM_13BIT            0b0000000010000000
#define MUX_11               0b0000101000000000
#define GHOST_CANCEL         0b1000000000000000
#define CONF2_DEFAULT        0b0001000000010000
#define DOUBLE_FRAMERATE     0b0000010000000000
#define DIM_COMP_35          0b0000000000001110
#define DIM_COMP_5           0b0000000000000010

#define MAX_SRGB 255
#define MAX_LINEAR 65535

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 250
#endif

typedef struct pixel {
	uint16_t r;
	uint16_t g;
	uint16_t b;
} pixel_t;

typedef uint8_t fastpixel_t[16];

extern pixel_t bitmap[88][88];
extern pixel_t blank[44];
extern pixel_t scanline_data[44];

extern uint8_t fastline_data[22][16];
extern uint8_t fastline_blank[22][16];

extern uint8_t allPins[];
extern uint8_t addrPins[];
extern uint8_t dataPins[];

// GCLK related
void gclock_init();
void vsync();

// NOP based small delay for data operations
void dataDelay();

// Splat 16bit colors into data ready to be written into GPIO
void set_red1(uint8_t pixel[], uint16_t c);
void set_green1(uint8_t pixel[], uint16_t c);
void set_blue1(uint8_t pixel[], uint16_t c);
void set_red2(uint8_t pixel[], uint16_t c);
void set_green2(uint8_t pixel[], uint16_t c);
void set_blue2(uint8_t pixel[], uint16_t c);

uint16_t sRGB_to_linear(uint8_t srgb);

void zeroDataPins();
void dataDelay();

// Send commands
void sendCommand(uint8_t length);
void sendReset();
void writeCommand1(uint16_t data);
void writeCommand2(uint16_t data);

// Write one led to each controller
void writeChain(pixel_t data[]);
void writeFastChain(uint8_t data[22][16]);

// Fill the panel with single line
void horizontalLine(pixel_t *color, uint8_t line);
void fastHorizontalLine(pixel_t *color, uint8_t line);

// Fill the panel with single color
void fillPanel(pixel_t *color);
void fastFillPanel(pixel_t *color);

void getChainData(pixel_t bitmap[88][88], uint scanline, uint led, pixel_t data[44]);
void sendBitmap(pixel_t bitmap[88][88]);

int main();
void core1_main();

int pico_led_init(void);
void pico_set_led(bool led_on);

int roe_init();

#endif /* ROEPICO_H */
