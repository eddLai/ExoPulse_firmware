/**
 * nRF52840 Dongle - USB CDC Test Only
 * Minimal test - just USB, no radio
 */

#include <stdint.h>
#include <string.h>
#include "nrf.h"

// LED on P0.06 (active low)
#define LED_PIN 6

void led_on(void) { NRF_P0->OUTCLR = (1 << LED_PIN); }
void led_off(void) { NRF_P0->OUTSET = (1 << LED_PIN); }

void delay(volatile int count) {
    while (count--) __asm volatile("nop");
}

void blink(int n) {
    for (int i = 0; i < n; i++) {
        led_on(); delay(4000000);   // ~0.5 sec on
        led_off(); delay(4000000);  // ~0.5 sec off (total ~1 sec per blink)
    }
    delay(8000000);  // ~1 sec pause between groups
}

// USBD registers
#define USBD_BASE 0x40027000UL
#define USBD_ENABLE          (*(volatile uint32_t *)(USBD_BASE + 0x500))
#define USBD_USBPULLUP       (*(volatile uint32_t *)(USBD_BASE + 0x504))
#define USBD_EVENTCAUSE      (*(volatile uint32_t *)(USBD_BASE + 0x400))
#define USBD_EVENTS_USBEVENT (*(volatile uint32_t *)(USBD_BASE + 0x158))

// Power registers - nRF52840 specific
#define POWER_BASE           0x40000000UL
#define POWER_USBREGSTATUS   (*(volatile uint32_t *)(POWER_BASE + 0x438))

// USB Regulator peripheral (nRF52840 only) - needed to enable USB power
#define USBREG_BASE          0x40037000UL
#define USBREG_EVENTS_USBDETECTED  (*(volatile uint32_t *)(USBREG_BASE + 0x100))
#define USBREG_EVENTS_USBREMOVED   (*(volatile uint32_t *)(USBREG_BASE + 0x104))
#define USBREG_EVENTS_USBPWRRDY    (*(volatile uint32_t *)(USBREG_BASE + 0x108))

int main(void)
{
    // Init LED
    NRF_P0->DIRSET = (1 << LED_PIN);
    NRF_P0->OUTSET = (1 << LED_PIN);

    // Long blink = alive
    led_on(); delay(500000); led_off(); delay(300000);

    // Start HFCLK
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}

    blink(1); // 1 = HFCLK done

    // Clear USB regulator events
    USBREG_EVENTS_USBDETECTED = 0;
    USBREG_EVENTS_USBPWRRDY = 0;

    // Wait for VBUS detected
    while (!(POWER_USBREGSTATUS & 0x01) && !USBREG_EVENTS_USBDETECTED) {}
    blink(2); // 2 = VBUS detected

    // Apply errata workarounds BEFORE enabling USB (per Nordic docs)
    // Errata 171 workaround - must be before USBD enable
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;
    *(volatile uint32_t *)0x4006EC14 = 0x000000C0;
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;

    // Errata 187 workaround - must be before USBD enable
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;
    *(volatile uint32_t *)0x4006ED14 = 0x00000003;
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;

    blink(3); // 3 = Errata pre-enable done

    // Clear and enable USBD
    USBD_EVENTS_USBEVENT = 0;
    USBD_EVENTCAUSE = 0xFFFFFFFF;  // Clear all cause bits
    USBD_ENABLE = 1;

    blink(4); // 4 = USBD_ENABLE written

    // Wait for USBEVENT or EVENTCAUSE.READY with very long timeout
    for (volatile int t = 0; t < 20000000; t++) {
        if (USBD_EVENTS_USBEVENT || (USBD_EVENTCAUSE & 0x01)) break;
    }

    blink(5); // 5 = Wait complete

    // Check what we got
    if (USBD_EVENTCAUSE & 0x01) {
        blink(1); // Extra 1 = READY bit set!
    }

    // Clear events
    USBD_EVENTS_USBEVENT = 0;
    USBD_EVENTCAUSE = 0xFFFFFFFF;  // Write 1s to clear

    // Errata 171 post-enable cleanup
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;
    *(volatile uint32_t *)0x4006EC14 = 0x00000000;
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;

    // Errata 187 post-enable cleanup
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;
    *(volatile uint32_t *)0x4006ED14 = 0x00000000;
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;

    blink(6); // 6 = Errata post done

    // Enable pull-up to signal device presence to host
    USBD_USBPULLUP = 1;

    blink(7); // 7 = Pull-up enabled - should enumerate!

    // Continuous slow blink = running
    while (1) {
        led_on(); delay(200000);
        led_off(); delay(800000);
    }
}
