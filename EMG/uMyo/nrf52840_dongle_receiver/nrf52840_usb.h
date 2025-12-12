/**
 * nRF52840 USB Device Register Definitions
 * Minimal definitions for USB CDC
 */
#ifndef NRF52840_USB_H
#define NRF52840_USB_H

#include <stdint.h>

// USB Device peripheral base address
#define NRF_USBD_BASE 0x40027000UL

// USB Device Register Structure
typedef struct {
    volatile uint32_t RESERVED0;
    volatile uint32_t TASKS_STARTEPIN[8];  // 0x004 - 0x020
    volatile uint32_t RESERVED1[2];
    volatile uint32_t TASKS_STARTISOIN;    // 0x02C
    volatile uint32_t TASKS_STARTEPOUT[8]; // 0x030 - 0x04C
    volatile uint32_t RESERVED2[2];
    volatile uint32_t TASKS_STARTISOOUT;   // 0x058
    volatile uint32_t TASKS_EP0RCVOUT;     // 0x05C
    volatile uint32_t TASKS_EP0STATUS;     // 0x060
    volatile uint32_t TASKS_EP0STALL;      // 0x064
    volatile uint32_t TASKS_DPDMDRIVE;     // 0x068
    volatile uint32_t TASKS_DPDMNODRIVE;   // 0x06C
    volatile uint32_t RESERVED3[36];
    volatile uint32_t EVENTS_USBRESET;     // 0x100
    volatile uint32_t EVENTS_STARTED;      // 0x104
    volatile uint32_t EVENTS_ENDEPIN[8];   // 0x108 - 0x124
    volatile uint32_t EVENTS_EP0DATADONE;  // 0x128
    volatile uint32_t EVENTS_ENDISOIN;     // 0x12C
    volatile uint32_t EVENTS_ENDEPOUT[8];  // 0x130 - 0x14C
    volatile uint32_t EVENTS_ENDISOOUT;    // 0x150
    volatile uint32_t EVENTS_SOF;          // 0x154
    volatile uint32_t EVENTS_USBEVENT;     // 0x158
    volatile uint32_t EVENTS_EP0SETUP;     // 0x15C
    volatile uint32_t EVENTS_EPDATA;       // 0x160
    volatile uint32_t RESERVED4[39];
    volatile uint32_t SHORTS;              // 0x200
    volatile uint32_t RESERVED5[63];
    volatile uint32_t INTEN;               // 0x300
    volatile uint32_t INTENSET;            // 0x304
    volatile uint32_t INTENCLR;            // 0x308
    volatile uint32_t RESERVED6[61];
    volatile uint32_t EVENTCAUSE;          // 0x400
    volatile uint32_t RESERVED7[7];
    struct {
        volatile uint32_t EPIN;
        volatile uint32_t EPOUT;
    } HALTED[8];                           // 0x420 - 0x45C
    volatile uint32_t RESERVED8[8];
    volatile uint32_t EPSTATUS;            // 0x468
    volatile uint32_t EPDATASTATUS;        // 0x46C
    volatile uint32_t USBADDR;             // 0x470
    volatile uint32_t RESERVED9[3];
    volatile uint32_t BMREQUESTTYPE;       // 0x480
    volatile uint32_t BREQUEST;            // 0x484
    volatile uint32_t WVALUEL;             // 0x488
    volatile uint32_t WVALUEH;             // 0x48C
    volatile uint32_t WINDEXL;             // 0x490
    volatile uint32_t WINDEXH;             // 0x494
    volatile uint32_t WLENGTHL;            // 0x498
    volatile uint32_t WLENGTHH;            // 0x49C
    struct {
        volatile uint32_t PTR;
        volatile uint32_t MAXCNT;
        volatile uint32_t AMOUNT;
        volatile uint32_t RESERVED;
    } SIZE_EPOUT_FAKE;                     // 0x4A0 - actually SIZE
    volatile uint32_t RESERVED10[15];
    volatile uint32_t ENABLE;              // 0x500
    volatile uint32_t USBPULLUP;           // 0x504
    volatile uint32_t RESERVED11[3];
    volatile uint32_t DPDMVALUE;           // 0x514
    volatile uint32_t DTOGGLE;             // 0x518
    volatile uint32_t EPINEN;              // 0x51C
    volatile uint32_t EPOUTEN;             // 0x520
    volatile uint32_t EPSTALL;             // 0x524
    volatile uint32_t ISOSPLIT;            // 0x528
    volatile uint32_t RESERVED12[5];
    volatile uint32_t FRAMECNTR;           // 0x540
    volatile uint32_t RESERVED13[3];
    volatile uint32_t LOWPOWER;            // 0x550
    volatile uint32_t ISOINCONFIG;         // 0x554
    volatile uint32_t RESERVED14[42];
    struct {
        volatile uint32_t PTR;
        volatile uint32_t MAXCNT;
        volatile uint32_t AMOUNT;
        volatile uint32_t RESERVED;
    } EPIN[8];                             // 0x600 - 0x67C
    struct {
        volatile uint32_t PTR;
        volatile uint32_t MAXCNT;
        volatile uint32_t AMOUNT;
        volatile uint32_t RESERVED;
    } ISOIN;                               // 0x680 - 0x68C
    volatile uint32_t RESERVED15[20];
    struct {
        volatile uint32_t PTR;
        volatile uint32_t MAXCNT;
        volatile uint32_t AMOUNT;
        volatile uint32_t RESERVED;
    } EPOUT[8];                            // 0x700 - 0x77C
    struct {
        volatile uint32_t PTR;
        volatile uint32_t MAXCNT;
        volatile uint32_t AMOUNT;
        volatile uint32_t RESERVED;
    } ISOOUT;                              // 0x780 - 0x78C
} NRF_USBD_Type;

// SIZE register structure (different location)
typedef struct {
    volatile uint32_t EPOUT[8];            // 0x4A0 - 0x4BC
    volatile uint32_t ISOIN;               // 0x4C0
    volatile uint32_t ISOOUT;              // 0x4C4
} NRF_USBD_SIZE_Type;

#define NRF_USBD ((NRF_USBD_Type *)NRF_USBD_BASE)
#define NRF_USBD_SIZE ((NRF_USBD_SIZE_Type *)(NRF_USBD_BASE + 0x4A0))

// POWER register for USB
#define NRF_POWER_USBREGSTATUS (*(volatile uint32_t *)(0x40000000 + 0x438))

#endif // NRF52840_USB_H
