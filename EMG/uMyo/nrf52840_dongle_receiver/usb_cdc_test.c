/**
 * Minimal USB CDC Test - Just enumerate and print "Hello"
 * No radio, no complex logic - just USB CDC
 */

#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "usb_cdc.h"

// LED on P0.06 (active low)
#define LED_PIN 6

void led_on(void) { NRF_P0->OUTCLR = (1 << LED_PIN); }
void led_off(void) { NRF_P0->OUTSET = (1 << LED_PIN); }

void delay(volatile int count) {
    while (count--) __asm volatile("nop");
}

void blink(int n) {
    for (int i = 0; i < n; i++) {
        led_on(); delay(2000000);
        led_off(); delay(2000000);
    }
    delay(4000000);
}

void fast_clock_start(void) {
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}
}

int main(void)
{
    // Init LED FIRST - absolute first thing
    NRF_P0->DIRSET = (1 << LED_PIN);
    NRF_P0->OUTCLR = (1 << LED_PIN);  // LED ON immediately

    // Very long delay to show we reached main()
    for (volatile int i = 0; i < 3000000; i++) __asm volatile("nop");

    NRF_P0->OUTSET = (1 << LED_PIN);  // LED OFF
    for (volatile int i = 0; i < 1000000; i++) __asm volatile("nop");

    // Start HFCLK
    fast_clock_start();
    blink(1);  // 1 blink = HFCLK done

    // Initialize USB CDC
    usb_cdc_init();
    blink(2);  // 2 blinks = USB init done

    // Queue initial message (will be sent after enumeration)
    usb_cdc_print("Hello from nRF52840 Dongle!\r\n");

    uint32_t counter = 0;
    char buf[32];

    // Main loop - just process USB and send periodic messages
    while (1) {
        // Process USB events
        usb_cdc_process();

        // Send a message every ~2 seconds
        counter++;
        if (counter >= 2000000) {
            counter = 0;
            led_on();

            // Simple counter message
            usb_cdc_print("Count: ");

            // Convert counter to string
            static uint32_t msg_num = 0;
            msg_num++;
            int val = msg_num;
            int i = 0;
            char tmp[12];
            if (val == 0) {
                tmp[i++] = '0';
            } else {
                while (val > 0) {
                    tmp[i++] = '0' + (val % 10);
                    val /= 10;
                }
            }
            int pos = 0;
            while (i > 0) {
                buf[pos++] = tmp[--i];
            }
            buf[pos] = 0;

            usb_cdc_print(buf);
            usb_cdc_print("\r\n");

            delay(100000);
            led_off();
        }
    }
}
