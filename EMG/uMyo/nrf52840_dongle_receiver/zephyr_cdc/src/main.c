/*
 * nRF52840 Dongle - Star Protocol Central Receiver
 * Zephyr RTOS version
 *
 * Receives data from uMyo sensors via Fast64 star_protocol
 * Outputs via USB CDC (serial over USB)
 * LED blinks on packet receive
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <string.h>
#include <stdio.h>

/* nRF register access */
#include <nrf.h>

/* Star protocol library */
#include "urf_radio.h"
#include "urf_timer.h"
#include "urf_star_protocol.h"

/* LED on P0.06 for nRF52840 Dongle (active low) */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* USB CDC ACM device */
static const struct device *cdc_dev;
static volatile bool usb_configured = false;

/* Output buffer */
static char out_buf[512];

static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    switch (status) {
    case USB_DC_CONFIGURED:
        usb_configured = true;
        break;
    case USB_DC_DISCONNECTED:
        usb_configured = false;
        break;
    default:
        break;
    }
}

static void cdc_write(const char *data, size_t len)
{
    if (!usb_configured || cdc_dev == NULL) {
        return;
    }

    const uint8_t *buf = (const uint8_t *)data;
    size_t remaining = len;

    while (remaining > 0) {
        int sent = uart_fifo_fill(cdc_dev, buf, remaining);
        if (sent > 0) {
            buf += sent;
            remaining -= sent;
        } else {
            /* FIFO full, yield briefly */
            k_usleep(100);
        }
    }
}

static void cdc_print(const char *str)
{
    cdc_write(str, strlen(str));
}

static void int_to_str(int val, char *buf)
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

static void hex_to_str(uint32_t val, char *buf)
{
    for (int i = 7; i >= 0; i--) {
        int nibble = (val >> (i * 4)) & 0xF;
        *buf++ = nibble < 10 ? '0' + nibble : 'A' + nibble - 10;
    }
    *buf = 0;
}

static void fast_clock_start(void)
{
    /* Request high-frequency clock for radio */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
        k_usleep(10);
    }
}

int main(void)
{
    int ret;
    char buf[32];
    char hex_buf[16];

    /* Initialize LED */
    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    /* Turn on LED to show we're alive */
    gpio_pin_set_dt(&led, 1);
    k_msleep(300);
    gpio_pin_set_dt(&led, 0);

    /* Get CDC ACM device */
    cdc_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
    if (!device_is_ready(cdc_dev)) {
        return 0;
    }

    /* Blink twice - CDC device ready */
    for (int i = 0; i < 2; i++) {
        gpio_pin_set_dt(&led, 1);
        k_msleep(150);
        gpio_pin_set_dt(&led, 0);
        k_msleep(150);
    }

    /* Enable USB */
    ret = usb_enable(usb_status_cb);
    if (ret != 0) {
        return 0;
    }

    /* Blink 3 times - USB enabled */
    for (int i = 0; i < 3; i++) {
        gpio_pin_set_dt(&led, 1);
        k_msleep(150);
        gpio_pin_set_dt(&led, 0);
        k_msleep(150);
    }

    /* Wait for USB to be configured (with timeout) */
    int wait_count = 0;
    while (!usb_configured && wait_count < 100) {
        k_msleep(100);
        wait_count++;
    }

    /* Start high-frequency clock for radio */
    fast_clock_start();

    /* Initialize timer (for star protocol) */
    time_start();

    /* Initialize star protocol as CENTRAL (receiver)
     * Channel 21, 1000 kbps, 2000us phase length, is_central=1 */
    star_init(21, 1000, 2000, 1);

    /* Set our unit ID from device ID */
    uint32_t unit_id = NRF_FICR->DEVICEID[1];
    star_set_id(unit_id);

    /* Long blink - radio initialized */
    gpio_pin_set_dt(&led, 1);
    k_msleep(500);
    gpio_pin_set_dt(&led, 0);

    /* Send welcome message */
    cdc_print("\r\n=== nRF52840 Dongle Star Protocol Receiver ===\r\n");
    cdc_print("Zephyr RTOS - Fast64 mode\r\n");
    cdc_print("Channel 21, 1000kbps, 2000us phase\r\n");
    cdc_print("Central ID: 0x");
    hex_to_str(unit_id, hex_buf);
    cdc_print(hex_buf);
    cdc_print("\r\nListening for uMyo sensors...\r\n\r\n");

    uint32_t last_status_ms = 0;
    uint32_t packet_count = 0;
    int64_t last_led_time = 0;

    while (1) {
        /* Process star protocol */
        star_loop_step();

        int64_t now = k_uptime_get();

        /* Check for received packets */
        if (star_has_packet()) {
            uint8_t pack[256];
            int len = star_get_packet(pack, 256);
            packet_count++;

            /* Blink LED */
            gpio_pin_set_dt(&led, 1);
            last_led_time = now;

            /* Build output in a single buffer */
            int pos = 0;

            /* Header */
            out_buf[pos++] = 'R'; out_buf[pos++] = 'X'; out_buf[pos++] = ':'; out_buf[pos++] = ' ';
            out_buf[pos++] = 'I'; out_buf[pos++] = 'D'; out_buf[pos++] = '='; out_buf[pos++] = '0'; out_buf[pos++] = 'x';

            /* Unit ID (bytes 2-5) */
            uint32_t rx_id = (pack[2] << 24) | (pack[3] << 16) | (pack[4] << 8) | pack[5];
            hex_to_str(rx_id, buf);
            for (int i = 0; buf[i]; i++) out_buf[pos++] = buf[i];

            /* ADC data ID (byte 11) */
            if (len > 11) {
                out_buf[pos++] = ' '; out_buf[pos++] = 'A'; out_buf[pos++] = 'D'; out_buf[pos++] = 'C';
                out_buf[pos++] = '_'; out_buf[pos++] = 'I'; out_buf[pos++] = 'D'; out_buf[pos++] = '=';
                int_to_str(pack[11], buf);
                for (int i = 0; buf[i]; i++) out_buf[pos++] = buf[i];
            }

            /* ADC samples (8 x 16-bit, starting at byte 12) */
            if (len > 27) {
                out_buf[pos++] = ' '; out_buf[pos++] = 'A'; out_buf[pos++] = 'D'; out_buf[pos++] = 'C'; out_buf[pos++] = ':';
                for (int i = 0; i < 8; i++) {
                    int idx = 12 + i * 2;
                    int16_t adc_val = (pack[idx] << 8) | pack[idx + 1];
                    int_to_str(adc_val, buf);
                    for (int j = 0; buf[j]; j++) out_buf[pos++] = buf[j];
                    if (i < 7) out_buf[pos++] = ',';
                }
            }

            /* Spectrum (4 x 16-bit, starting at byte 28) */
            if (len > 35) {
                out_buf[pos++] = ' '; out_buf[pos++] = 'S'; out_buf[pos++] = 'P'; out_buf[pos++] = ':';
                for (int i = 0; i < 4; i++) {
                    int idx = 28 + i * 2;
                    int16_t sp_val = (pack[idx] << 8) | pack[idx + 1];
                    int_to_str(sp_val, buf);
                    for (int j = 0; buf[j]; j++) out_buf[pos++] = buf[j];
                    if (i < 3) out_buf[pos++] = ',';
                }
            }

            out_buf[pos++] = '\r'; out_buf[pos++] = '\n'; out_buf[pos] = 0;

            /* Send if USB is ready */
            if (usb_configured) {
                cdc_write(out_buf, pos);
            }
        }

        /* Turn off LED after 20ms */
        if (now - last_led_time > 20 && last_led_time > 0) {
            gpio_pin_set_dt(&led, 0);
        }

        /* Print status every 5 seconds */
        uint32_t ms = (uint32_t)now;
        if (ms - last_status_ms > 5000) {
            last_status_ms = ms;
            gpio_pin_set_dt(&led, 1);

            snprintf(out_buf, sizeof(out_buf), "--- Status: %u packets, %us uptime ---\r\n",
                     packet_count, ms / 1000);
            cdc_print(out_buf);

            k_msleep(50);
            gpio_pin_set_dt(&led, 0);
        }

        /* Small yield to allow other tasks */
        k_usleep(100);
    }

    return 0;
}
