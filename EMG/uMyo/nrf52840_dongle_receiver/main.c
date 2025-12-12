/**
 * nRF52840 Dongle - Star Protocol Central Receiver
 *
 * Receives data from uMyo sensors via Fast64 star_protocol
 * Outputs via USB CDC (serial over USB)
 * LED blinks on packet receive
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "urf_radio.h"
#include "urf_timer.h"
#include "urf_star_protocol.h"
#include "usb_cdc.h"

// nRF52840 Dongle LED: P0.06 (green, active low)
#define LED_PIN 6

void led_init(void)
{
    NRF_P0->DIRSET = (1 << LED_PIN);
    NRF_P0->OUTSET = (1 << LED_PIN);  // LED off
}

void led_on(void)
{
    NRF_P0->OUTCLR = (1 << LED_PIN);
}

void led_off(void)
{
    NRF_P0->OUTSET = (1 << LED_PIN);
}

void fast_clock_start(void)
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}
}

void int_to_str(int val, char *buf)
{
    if (val < 0) {
        *buf++ = '-';
        val = -val;
    }
    char tmp[12];
    int i = 0;
    if (val == 0) {
        tmp[i++] = '0';
    } else {
        while (val > 0) {
            tmp[i++] = '0' + (val % 10);
            val /= 10;
        }
    }
    while (i > 0) {
        *buf++ = tmp[--i];
    }
    *buf = 0;
}

void hex_to_str(uint32_t val, char *buf)
{
    for (int i = 7; i >= 0; i--) {
        int nibble = (val >> (i * 4)) & 0xF;
        *buf++ = nibble < 10 ? '0' + nibble : 'A' + nibble - 10;
    }
    *buf = 0;
}

int main(void)
{
    // Initialize LED first - for debugging
    led_init();

    // Long blink to show we're alive
    led_on();
    for (volatile int d = 0; d < 2000000; d++);
    led_off();
    for (volatile int d = 0; d < 1000000; d++);

    // Start high-frequency clock for radio and USB
    fast_clock_start();

    // Initialize timer
    time_start();

    // Initialize USB CDC
    usb_cdc_init();

    // Blink twice to show USB init done
    for (int i = 0; i < 2; i++) {
        led_on();
        for (volatile int d = 0; d < 1000000; d++);
        led_off();
        for (volatile int d = 0; d < 1000000; d++);
    }

    // Initialize star protocol AFTER USB
    star_init(21, 1000, 2000, 1);

    // Set our unit ID
    uint32_t unit_id = NRF_FICR->DEVICEID[1];
    star_set_id(unit_id);

    char buf[32];
    char hex_buf[16];

    usb_cdc_print("\r\n=== nRF52840 Dongle Star Protocol Receiver ===\r\n");
    usb_cdc_print("Fast64 mode - Channel 21, 1000kbps, 2000us phase\r\n");
    usb_cdc_print("Central ID: 0x");
    hex_to_str(unit_id, hex_buf);
    usb_cdc_print(hex_buf);
    usb_cdc_print("\r\nListening for uMyo sensors...\r\n\r\n");

    uint32_t last_status_ms = 0;
    uint32_t packet_count = 0;
    uint32_t last_led_ms = 0;

    while (1)
    {
        // Process USB CDC
        usb_cdc_process();

        // Process star protocol
        star_loop_step();

        uint32_t ms = millis();

        // Check for received packets
        if (star_has_packet())
        {
            uint8_t pack[256];
            int len = star_get_packet(pack, 256);
            packet_count++;

            // Blink LED
            led_on();
            last_led_ms = ms;

            // Build output in a single buffer
            char out[256];
            int pos = 0;

            out[pos++] = 'R'; out[pos++] = 'X'; out[pos++] = ':'; out[pos++] = ' ';
            out[pos++] = 'I'; out[pos++] = 'D'; out[pos++] = '='; out[pos++] = '0'; out[pos++] = 'x';

            uint32_t rx_id = (pack[2] << 24) | (pack[3] << 16) | (pack[4] << 8) | pack[5];
            hex_to_str(rx_id, buf);
            for (int i = 0; buf[i]; i++) out[pos++] = buf[i];

            if (len > 11) {
                out[pos++] = ' '; out[pos++] = 'A'; out[pos++] = 'D'; out[pos++] = 'C';
                out[pos++] = '_'; out[pos++] = 'I'; out[pos++] = 'D'; out[pos++] = '=';
                int_to_str(pack[11], buf);
                for (int i = 0; buf[i]; i++) out[pos++] = buf[i];
            }

            if (len > 27) {
                out[pos++] = ' '; out[pos++] = 'A'; out[pos++] = 'D'; out[pos++] = 'C'; out[pos++] = ':';
                for (int i = 0; i < 8; i++) {
                    int idx = 12 + i * 2;
                    int16_t adc_val = (pack[idx] << 8) | pack[idx + 1];
                    int_to_str(adc_val, buf);
                    for (int j = 0; buf[j]; j++) out[pos++] = buf[j];
                    if (i < 7) out[pos++] = ',';
                }
            }

            out[pos++] = '\r'; out[pos++] = '\n'; out[pos] = 0;

            // Send if USB is ready
            if (usb_cdc_write_ready()) {
                usb_cdc_write((uint8_t *)out, pos);
            }
        }

        // Turn off LED after 20ms
        if (ms - last_led_ms > 20 && last_led_ms > 0)
        {
            led_off();
        }

        // Print status every 5 seconds
        if (ms - last_status_ms > 5000)
        {
            last_status_ms = ms;
            led_on();
            usb_cdc_print("--- Status: ");
            int_to_str(packet_count, buf);
            usb_cdc_print(buf);
            usb_cdc_print(" packets ---\r\n");
            for (volatile int d = 0; d < 50000; d++);
            led_off();
        }
    }
}
