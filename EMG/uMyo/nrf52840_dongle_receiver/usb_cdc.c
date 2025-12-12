/**
 * Minimal USB CDC (ACM) driver for nRF52840
 * Implements a simple USB serial port
 */

#include "usb_cdc.h"
#include "nrf.h"
#include <string.h>

// USB Device peripheral - direct register access for nRF52840
#define USBD_BASE 0x40027000UL

// Task registers
#define USBD_TASKS_STARTEPIN(n)  (*(volatile uint32_t *)(USBD_BASE + 0x004 + (n)*4))
#define USBD_TASKS_STARTEPOUT(n) (*(volatile uint32_t *)(USBD_BASE + 0x030 + (n)*4))
#define USBD_TASKS_EP0RCVOUT     (*(volatile uint32_t *)(USBD_BASE + 0x05C))
#define USBD_TASKS_EP0STATUS     (*(volatile uint32_t *)(USBD_BASE + 0x060))
#define USBD_TASKS_EP0STALL      (*(volatile uint32_t *)(USBD_BASE + 0x064))

// Event registers
#define USBD_EVENTS_USBRESET     (*(volatile uint32_t *)(USBD_BASE + 0x100))
#define USBD_EVENTS_STARTED      (*(volatile uint32_t *)(USBD_BASE + 0x104))
#define USBD_EVENTS_ENDEPIN(n)   (*(volatile uint32_t *)(USBD_BASE + 0x108 + (n)*4))
#define USBD_EVENTS_EP0DATADONE  (*(volatile uint32_t *)(USBD_BASE + 0x128))
#define USBD_EVENTS_ENDEPOUT(n)  (*(volatile uint32_t *)(USBD_BASE + 0x130 + (n)*4))
#define USBD_EVENTS_USBEVENT     (*(volatile uint32_t *)(USBD_BASE + 0x158))
#define USBD_EVENTS_EP0SETUP     (*(volatile uint32_t *)(USBD_BASE + 0x15C))
#define USBD_EVENTS_EPDATA       (*(volatile uint32_t *)(USBD_BASE + 0x160))

// Control registers
#define USBD_EVENTCAUSE          (*(volatile uint32_t *)(USBD_BASE + 0x400))
#define USBD_EPSTATUS            (*(volatile uint32_t *)(USBD_BASE + 0x468))
#define USBD_EPDATASTATUS        (*(volatile uint32_t *)(USBD_BASE + 0x46C))
#define USBD_USBADDR             (*(volatile uint32_t *)(USBD_BASE + 0x470))

// Setup packet registers
#define USBD_BMREQUESTTYPE       (*(volatile uint32_t *)(USBD_BASE + 0x480))
#define USBD_BREQUEST            (*(volatile uint32_t *)(USBD_BASE + 0x484))
#define USBD_WVALUEL             (*(volatile uint32_t *)(USBD_BASE + 0x488))
#define USBD_WVALUEH             (*(volatile uint32_t *)(USBD_BASE + 0x48C))
#define USBD_WINDEXL             (*(volatile uint32_t *)(USBD_BASE + 0x490))
#define USBD_WINDEXH             (*(volatile uint32_t *)(USBD_BASE + 0x494))
#define USBD_WLENGTHL            (*(volatile uint32_t *)(USBD_BASE + 0x498))
#define USBD_WLENGTHH            (*(volatile uint32_t *)(USBD_BASE + 0x49C))

// SIZE registers
#define USBD_SIZE_EPOUT(n)       (*(volatile uint32_t *)(USBD_BASE + 0x4A0 + (n)*4))

// Configuration registers
#define USBD_ENABLE              (*(volatile uint32_t *)(USBD_BASE + 0x500))
#define USBD_USBPULLUP           (*(volatile uint32_t *)(USBD_BASE + 0x504))
#define USBD_EPINEN              (*(volatile uint32_t *)(USBD_BASE + 0x51C))
#define USBD_EPOUTEN             (*(volatile uint32_t *)(USBD_BASE + 0x520))

// Endpoint buffer registers (EPIN)
#define USBD_EPIN_PTR(n)         (*(volatile uint32_t *)(USBD_BASE + 0x600 + (n)*0x10))
#define USBD_EPIN_MAXCNT(n)      (*(volatile uint32_t *)(USBD_BASE + 0x604 + (n)*0x10))
#define USBD_EPIN_AMOUNT(n)      (*(volatile uint32_t *)(USBD_BASE + 0x608 + (n)*0x10))

// Endpoint buffer registers (EPOUT)
#define USBD_EPOUT_PTR(n)        (*(volatile uint32_t *)(USBD_BASE + 0x700 + (n)*0x10))
#define USBD_EPOUT_MAXCNT(n)     (*(volatile uint32_t *)(USBD_BASE + 0x704 + (n)*0x10))
#define USBD_EPOUT_AMOUNT(n)     (*(volatile uint32_t *)(USBD_BASE + 0x708 + (n)*0x10))

// Power registers for USB
#define POWER_BASE               0x40000000UL
#define POWER_USBREGSTATUS       (*(volatile uint32_t *)(POWER_BASE + 0x438))

// USB Regulator peripheral (nRF52840 only)
#define USBREG_BASE              0x40037000UL
#define USBREG_EVENTS_USBDETECTED  (*(volatile uint32_t *)(USBREG_BASE + 0x100))
#define USBREG_EVENTS_USBPWRRDY    (*(volatile uint32_t *)(USBREG_BASE + 0x108))

// USB endpoint sizes
#define EP0_SIZE 64
#define CDC_SIZE 64

// USB state
static volatile uint8_t usb_configured = 0;
static volatile uint8_t usb_address = 0;
static volatile uint8_t set_address_pending = 0;
static volatile uint8_t ep0_out_pending = 0;  // Track if we're expecting OUT data

// TX/RX buffers
static uint8_t tx_buffer[256];
static volatile int tx_head = 0;
static volatile int tx_tail = 0;
static volatile int tx_busy = 0;

static uint8_t rx_buffer[256];
static volatile int rx_head = 0;
static volatile int rx_tail = 0;

// Endpoint buffers (must be in RAM, aligned)
static uint8_t ep0_buf[EP0_SIZE] __attribute__((aligned(4)));
static uint8_t cdc_rx_buf[CDC_SIZE] __attribute__((aligned(4)));
static uint8_t cdc_tx_buf[CDC_SIZE] __attribute__((aligned(4)));

// Device descriptor
static const uint8_t device_descriptor[] = {
    18,         // bLength
    1,          // bDescriptorType (Device)
    0x00, 0x02, // bcdUSB 2.0
    0x02,       // bDeviceClass (CDC)
    0x00,       // bDeviceSubClass
    0x00,       // bDeviceProtocol
    EP0_SIZE,   // bMaxPacketSize0
    0x15, 0x19, // idVendor (Nordic)
    0x20, 0x52, // idProduct
    0x00, 0x01, // bcdDevice
    1,          // iManufacturer
    2,          // iProduct
    3,          // iSerialNumber
    1           // bNumConfigurations
};

// Configuration descriptor (CDC ACM)
static const uint8_t config_descriptor[] = {
    // Configuration descriptor
    9, 2, 67, 0, 2, 1, 0, 0xC0, 50,  // 100mA, self-powered

    // Interface 0: CDC Control
    9, 4, 0, 0, 1, 0x02, 0x02, 0x01, 0,

    // CDC Header
    5, 0x24, 0x00, 0x10, 0x01,

    // CDC Call Management
    5, 0x24, 0x01, 0x00, 1,

    // CDC ACM
    4, 0x24, 0x02, 0x02,

    // CDC Union
    5, 0x24, 0x06, 0, 1,

    // Endpoint 1 IN (interrupt)
    7, 5, 0x81, 0x03, 8, 0, 255,

    // Interface 1: CDC Data
    9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,

    // Endpoint 2 OUT (bulk)
    7, 5, 0x02, 0x02, CDC_SIZE, 0, 0,

    // Endpoint 2 IN (bulk)
    7, 5, 0x82, 0x02, CDC_SIZE, 0, 0
};

// String descriptors
static const uint8_t string0[] = {4, 3, 0x09, 0x04};  // Language: English
static const uint8_t string1[] = {14, 3, 'N',0,'o',0,'r',0,'d',0,'i',0,'c',0};
static const uint8_t string2[] = {20, 3, 'u',0,'M',0,'y',0,'o',0,' ',0,'R',0,'x',0,' ',0,' ',0};
static const uint8_t string3[] = {10, 3, '1',0,'2',0,'3',0,'4',0};

// Line coding (115200 8N1)
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

static void usb_ep0_stall(void) {
    USBD_TASKS_EP0STALL = 1;
}

static void usb_ep0_send(const uint8_t *data, int len) {
    if (len > EP0_SIZE) len = EP0_SIZE;
    memcpy(ep0_buf, data, len);
    USBD_EPIN_PTR(0) = (uint32_t)ep0_buf;
    USBD_EPIN_MAXCNT(0) = len;
    USBD_TASKS_STARTEPIN(0) = 1;
}

static void usb_ep0_status(void) {
    USBD_TASKS_EP0STATUS = 1;
}

static void handle_setup(void) {
    uint8_t bmRequestType = USBD_BMREQUESTTYPE;
    uint8_t bRequest = USBD_BREQUEST;
    uint16_t wValue = USBD_WVALUEL | (USBD_WVALUEH << 8);
    uint16_t wIndex = USBD_WINDEXL | (USBD_WINDEXH << 8);
    uint16_t wLength = USBD_WLENGTHL | (USBD_WLENGTHH << 8);

    (void)wIndex;

    // Standard requests
    if ((bmRequestType & 0x60) == 0x00) {
        switch (bRequest) {
            case 0x00: // GET_STATUS
                ep0_buf[0] = 0;
                ep0_buf[1] = 0;
                usb_ep0_send(ep0_buf, 2);
                break;

            case 0x05: // SET_ADDRESS
                usb_address = wValue & 0x7F;
                set_address_pending = 1;
                usb_ep0_status();
                break;

            case 0x06: // GET_DESCRIPTOR
                switch (wValue >> 8) {
                    case 1: // Device
                        usb_ep0_send(device_descriptor,
                            wLength < sizeof(device_descriptor) ? wLength : sizeof(device_descriptor));
                        break;
                    case 2: // Configuration
                        usb_ep0_send(config_descriptor,
                            wLength < sizeof(config_descriptor) ? wLength : sizeof(config_descriptor));
                        break;
                    case 3: // String
                        switch (wValue & 0xFF) {
                            case 0: usb_ep0_send(string0, string0[0] < wLength ? string0[0] : wLength); break;
                            case 1: usb_ep0_send(string1, string1[0] < wLength ? string1[0] : wLength); break;
                            case 2: usb_ep0_send(string2, string2[0] < wLength ? string2[0] : wLength); break;
                            case 3: usb_ep0_send(string3, string3[0] < wLength ? string3[0] : wLength); break;
                            default: usb_ep0_stall(); break;
                        }
                        break;
                    default:
                        usb_ep0_stall();
                        break;
                }
                break;

            case 0x09: // SET_CONFIGURATION
                usb_configured = (wValue != 0);
                // Enable endpoints
                USBD_EPOUTEN = 0x05;  // EP0 OUT + EP2 OUT
                USBD_EPINEN = 0x07;   // EP0 IN + EP1 IN + EP2 IN
                // Prepare to receive data on EP2 OUT
                USBD_EPOUT_PTR(2) = (uint32_t)cdc_rx_buf;
                USBD_EPOUT_MAXCNT(2) = CDC_SIZE;
                USBD_SIZE_EPOUT(2) = 0;
                USBD_TASKS_STARTEPOUT(2) = 1;
                usb_ep0_status();
                break;

            default:
                usb_ep0_stall();
                break;
        }
    }
    // CDC class requests
    else if ((bmRequestType & 0x60) == 0x20) {
        switch (bRequest) {
            case 0x20: // SET_LINE_CODING
                // Receive line coding data
                ep0_out_pending = 1;
                USBD_EPOUT_PTR(0) = (uint32_t)line_coding;
                USBD_EPOUT_MAXCNT(0) = 7;
                USBD_TASKS_EP0RCVOUT = 1;
                break;
            case 0x21: // GET_LINE_CODING
                usb_ep0_send(line_coding, 7);
                break;
            case 0x22: // SET_CONTROL_LINE_STATE
                usb_ep0_status();
                break;
            default:
                usb_ep0_stall();
                break;
        }
    }
    else {
        usb_ep0_stall();
    }
}

// External LED control for debugging (from main.c)
extern void led_on(void);
extern void led_off(void);

static void debug_blink(int count) {
    for (int i = 0; i < count; i++) {
        led_on();
        for (volatile int d = 0; d < 100000; d++);
        led_off();
        for (volatile int d = 0; d < 100000; d++);
    }
    // Pause between blink groups
    for (volatile int d = 0; d < 300000; d++);
}

void usb_cdc_init(void) {
    // Clear USB regulator events
    USBREG_EVENTS_USBDETECTED = 0;
    USBREG_EVENTS_USBPWRRDY = 0;

    // Wait for VBUS detected (bit 0 of USBREGSTATUS) with timeout
    for (volatile int t = 0; t < 10000000; t++) {
        if ((POWER_USBREGSTATUS & 0x01) || USBREG_EVENTS_USBDETECTED) break;
    }

    // Wait for USB regulator output ready (bit 1) with timeout
    for (volatile int t = 0; t < 10000000; t++) {
        if ((POWER_USBREGSTATUS & 0x02) || USBREG_EVENTS_USBPWRRDY) break;
    }

    // Apply errata workarounds BEFORE enabling USBD
    // Errata 171 workaround
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;
    *(volatile uint32_t *)0x4006EC14 = 0x000000C0;
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;

    // Errata 187 workaround
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;
    *(volatile uint32_t *)0x4006ED14 = 0x00000003;
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;

    // Clear and enable USBD
    USBD_EVENTS_USBEVENT = 0;
    USBD_EVENTCAUSE = 0xFFFFFFFF;
    USBD_ENABLE = 1;

    // Wait for EVENTCAUSE.READY (bit 0)
    while (!(USBD_EVENTCAUSE & 0x01)) {}

    // Clear EVENTCAUSE.READY by writing 1
    USBD_EVENTCAUSE = 0x01;
    USBD_EVENTS_USBEVENT = 0;

    // Errata 171 post-enable cleanup
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;
    *(volatile uint32_t *)0x4006EC14 = 0x00000000;
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;

    // Errata 187 post-enable cleanup
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;
    *(volatile uint32_t *)0x4006ED14 = 0x00000000;
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;

    // Enable EP0 before pull-up (important!)
    USBD_EPINEN = 0x01;   // EP0 IN only
    USBD_EPOUTEN = 0x01;  // EP0 OUT only

    // Clear all events before enabling pull-up
    USBD_EVENTS_USBRESET = 0;
    USBD_EVENTS_EP0SETUP = 0;
    USBD_EVENTS_EP0DATADONE = 0;
    USBD_EVENTS_ENDEPOUT(2) = 0;
    USBD_EVENTS_ENDEPIN(2) = 0;

    // Enable pull-up to signal device presence to host
    USBD_USBPULLUP = 1;

    usb_configured = 0;
    usb_address = 0;
    set_address_pending = 0;
    ep0_out_pending = 0;
    tx_head = tx_tail = 0;
    rx_head = rx_tail = 0;
    tx_busy = 0;
}

void usb_cdc_process(void) {
    // USB Reset
    if (USBD_EVENTS_USBRESET) {
        USBD_EVENTS_USBRESET = 0;
        usb_configured = 0;
        usb_address = 0;
        USBD_EPOUTEN = 1;  // EP0 OUT only
        USBD_EPINEN = 1;   // EP0 IN only
    }

    // Setup packet
    if (USBD_EVENTS_EP0SETUP) {
        USBD_EVENTS_EP0SETUP = 0;
        handle_setup();
    }

    // EP0 data done
    if (USBD_EVENTS_EP0DATADONE) {
        USBD_EVENTS_EP0DATADONE = 0;
        if (set_address_pending) {
            set_address_pending = 0;
            USBD_USBADDR = usb_address;
        }
        // Only send status for OUT transfers (e.g., SET_LINE_CODING)
        // For IN transfers, status is handled by the host
        if (ep0_out_pending) {
            ep0_out_pending = 0;
            usb_ep0_status();
        }
    }

    // CDC RX complete (EP2 OUT)
    if (USBD_EVENTS_ENDEPOUT(2)) {
        USBD_EVENTS_ENDEPOUT(2) = 0;
        int len = USBD_SIZE_EPOUT(2);
        for (int i = 0; i < len; i++) {
            int next = (rx_head + 1) % sizeof(rx_buffer);
            if (next != rx_tail) {
                rx_buffer[rx_head] = cdc_rx_buf[i];
                rx_head = next;
            }
        }
        // Prepare for next receive
        USBD_SIZE_EPOUT(2) = 0;
        USBD_TASKS_STARTEPOUT(2) = 1;
    }

    // CDC TX complete (EP2 IN)
    if (USBD_EVENTS_ENDEPIN(2)) {
        USBD_EVENTS_ENDEPIN(2) = 0;
        tx_busy = 0;
    }

    // Send pending TX data
    if (!tx_busy && tx_head != tx_tail && usb_configured) {
        int len = 0;
        while (tx_head != tx_tail && len < CDC_SIZE) {
            cdc_tx_buf[len++] = tx_buffer[tx_tail];
            tx_tail = (tx_tail + 1) % sizeof(tx_buffer);
        }
        if (len > 0) {
            tx_busy = 1;
            USBD_EPIN_PTR(2) = (uint32_t)cdc_tx_buf;
            USBD_EPIN_MAXCNT(2) = len;
            USBD_TASKS_STARTEPIN(2) = 1;
        }
    }
}

int usb_cdc_write(const uint8_t *data, int len) {
    int written = 0;
    for (int i = 0; i < len; i++) {
        int next = (tx_head + 1) % sizeof(tx_buffer);
        if (next == tx_tail) break;  // Buffer full
        tx_buffer[tx_head] = data[i];
        tx_head = next;
        written++;
    }
    return written;
}

void usb_cdc_print(const char *str) {
    int len = 0;
    while (str[len]) len++;
    usb_cdc_write((const uint8_t *)str, len);
}

int usb_cdc_available(void) {
    return (rx_head - rx_tail + sizeof(rx_buffer)) % sizeof(rx_buffer);
}

int usb_cdc_read(uint8_t *data, int max_len) {
    int read = 0;
    while (rx_head != rx_tail && read < max_len) {
        data[read++] = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % sizeof(rx_buffer);
    }
    return read;
}

int usb_cdc_write_ready(void) {
    // Check if there's room in the TX buffer (at least 64 bytes free)
    int used = (tx_head - tx_tail + sizeof(tx_buffer)) % sizeof(tx_buffer);
    return (sizeof(tx_buffer) - used) > 64;
}
