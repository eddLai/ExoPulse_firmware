/**
 * nRF52840 Dongle - LED Blink Test
 *
 * Simple LED blink to verify DFU works correctly
 * LED1 (green) on P0.06
 * LED2 (RGB) - R:P0.08, G:P1.09, B:P0.12
 */

#include <stdint.h>
#include "nrf.h"

// nRF52840 Dongle PCA10059 LEDs (active low)
#define LED1_PIN    6   // P0.06 - Green LED (main)
#define LED2_R_PIN  8   // P0.08 - RGB Red

// Simple delay using loop
void delay_ms(uint32_t ms)
{
    // Approximate delay at 64MHz
    volatile uint32_t count = ms * 8000;
    while (count--) {
        __asm volatile("nop");
    }
}

int main(void)
{
    // Configure LED1 (P0.06) as output
    NRF_P0->DIRSET = (1 << LED1_PIN);
    NRF_P0->OUTSET = (1 << LED1_PIN);  // LED off (active low)

    // Configure LED2 Red (P0.08) as output
    NRF_P0->DIRSET = (1 << LED2_R_PIN);
    NRF_P0->OUTSET = (1 << LED2_R_PIN);  // LED off

    // Blink forever
    while (1)
    {
        // LED1 on (green)
        NRF_P0->OUTCLR = (1 << LED1_PIN);
        delay_ms(200);

        // LED1 off
        NRF_P0->OUTSET = (1 << LED1_PIN);
        delay_ms(200);

        // LED2 Red on
        NRF_P0->OUTCLR = (1 << LED2_R_PIN);
        delay_ms(200);

        // LED2 Red off
        NRF_P0->OUTSET = (1 << LED2_R_PIN);
        delay_ms(200);
    }

    return 0;
}
