/**
 * nRF52840 DK - Star Protocol Central Receiver
 *
 * Receives data from uMyo sensors via Fast64 star_protocol
 * Outputs via UART (USB CDC on nRF52840 DK)
 * LED1 blinks on packet receive
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "urf_radio.h"
#include "urf_timer.h"
#include "urf_star_protocol.h"
#include "urf_uart.h"

// nRF52840 DK LEDs: P0.13, P0.14, P0.15, P0.16 (active low)
#define LED1_PIN 13
#define LED2_PIN 14
#define LED3_PIN 15
#define LED4_PIN 16

// UART TX on P0.06, RX on P0.08 (nRF52840 DK default)
#define UART_TX_PIN 6
#define UART_RX_PIN 8

void led_init(void)
{
    NRF_P0->DIRSET = (1 << LED1_PIN) | (1 << LED2_PIN) | (1 << LED3_PIN) | (1 << LED4_PIN);
    NRF_P0->OUTSET = (1 << LED1_PIN) | (1 << LED2_PIN) | (1 << LED3_PIN) | (1 << LED4_PIN);  // All LEDs off
}

void led_on(int pin) { NRF_P0->OUTCLR = (1 << pin); }
void led_off(int pin) { NRF_P0->OUTSET = (1 << pin); }
void led_toggle(int pin) { NRF_P0->OUT ^= (1 << pin); }

// Helper to send string via urf_uart
void uart_puts(const char *str)
{
    int len = strlen(str);
    if (len > 255) len = 255;
    uart_send((uint8_t *)str, len);
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

void fast_clock_start(void)
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}
}

int main(void)
{
    // Start high-frequency clock for radio
    fast_clock_start();

    // Initialize timer
    time_start();

    // Initialize LEDs
    led_init();

    // Blink all LEDs on startup
    for (int i = 0; i < 3; i++) {
        led_on(LED1_PIN);
        led_on(LED2_PIN);
        led_on(LED3_PIN);
        led_on(LED4_PIN);
        delay_ms(100);
        led_off(LED1_PIN);
        led_off(LED2_PIN);
        led_off(LED3_PIN);
        led_off(LED4_PIN);
        delay_ms(100);
    }

    // Initialize UART using urf_uart library - 921600 baud for high data rate
    uart_init(UART_TX_PIN, UART_RX_PIN, 921600);

    uart_puts("\r\n=== nRF52840 DK Star Protocol Receiver ===\r\n");
    uart_puts("Fast64 mode - Channel 21, 1000kbps, 2000us phase, 921600 baud UART\r\n");

    // Initialize star protocol as CENTRAL (receiver)
    // Channel 21, 1000 kbps, 2000us phase length, is_central=1
    star_init(21, 1000, 2000, 1);

    // Set our unit ID
    uint32_t unit_id = NRF_FICR->DEVICEID[1];
    star_set_id(unit_id);

    char buf[32];
    char hex_buf[16];

    uart_puts("Central ID: 0x");
    hex_to_str(unit_id, hex_buf);
    uart_puts(hex_buf);
    uart_puts("\r\nListening for uMyo sensors...\r\n\r\n");

    uint32_t last_status_ms = 0;
    uint32_t packet_count = 0;
    uint32_t last_led_ms = 0;
    uint32_t output_counter = 0;

    // Light LED4 to show we're running
    led_on(LED4_PIN);

    while (1)
    {
        // Process star protocol
        star_loop_step();

        uint32_t ms = millis();

        // Check for received packets
        if (star_has_packet())
        {
            uint8_t pack[256];
            int len = star_get_packet(pack, 256);
            packet_count++;

            // Blink LED1
            led_on(LED1_PIN);
            last_led_ms = ms;

            // Only output every packet (no skipping) - UART should handle it now
            output_counter++;

            // Build output in a single buffer to avoid UART overflow
            // Format: RX: ID=0xXXXXXXXX ADC_ID=N ADC:v0,v1,v2,v3,v4,v5,v6,v7 SP:s0,s1,s2,s3
            char out[256];
            int pos = 0;

            // Header
            out[pos++] = 'R'; out[pos++] = 'X'; out[pos++] = ':'; out[pos++] = ' ';
            out[pos++] = 'I'; out[pos++] = 'D'; out[pos++] = '='; out[pos++] = '0'; out[pos++] = 'x';

            // Unit ID (bytes 2-5)
            uint32_t rx_id = (pack[2] << 24) | (pack[3] << 16) | (pack[4] << 8) | pack[5];
            hex_to_str(rx_id, buf);
            for (int i = 0; buf[i]; i++) out[pos++] = buf[i];

            // ADC data ID (byte 11)
            if (len > 11) {
                out[pos++] = ' '; out[pos++] = 'A'; out[pos++] = 'D'; out[pos++] = 'C';
                out[pos++] = '_'; out[pos++] = 'I'; out[pos++] = 'D'; out[pos++] = '=';
                int_to_str(pack[11], buf);
                for (int i = 0; buf[i]; i++) out[pos++] = buf[i];
            }

            // ADC samples (8 x 16-bit, starting at byte 12)
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

            // Spectrum (4 x 16-bit, starting at byte 28)
            if (len > 35) {
                out[pos++] = ' '; out[pos++] = 'S'; out[pos++] = 'P'; out[pos++] = ':';
                for (int i = 0; i < 4; i++) {
                    int idx = 28 + i * 2;
                    int16_t sp_val = (pack[idx] << 8) | pack[idx + 1];
                    int_to_str(sp_val, buf);
                    for (int j = 0; buf[j]; j++) out[pos++] = buf[j];
                    if (i < 3) out[pos++] = ',';
                }
            }

            out[pos++] = '\r'; out[pos++] = '\n'; out[pos] = 0;

            // Wait for UART to be ready before sending (prevent buffer overflow)
            while (!uart_send_ready()) {}

            // Send as single chunk
            uart_send((uint8_t *)out, pos);
        }

        // Turn off LED1 after 20ms
        if (ms - last_led_ms > 20 && last_led_ms > 0) {
            led_off(LED1_PIN);
        }

        // Print status every 5 seconds
        if (ms - last_status_ms > 5000) {
            last_status_ms = ms;
            led_toggle(LED2_PIN);
            uart_puts("--- Status: ");
            int_to_str(packet_count, buf);
            uart_puts(buf);
            uart_puts(" packets, ");
            int_to_str(ms / 1000, buf);
            uart_puts(buf);
            uart_puts("s uptime ---\r\n");
        }
    }
}
