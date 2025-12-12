/**
 * Minimal USB CDC for nRF52840
 */
#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

void usb_cdc_init(void);
void usb_cdc_process(void);
int usb_cdc_write(const uint8_t *data, int len);
void usb_cdc_print(const char *str);
int usb_cdc_available(void);
int usb_cdc_read(uint8_t *data, int max_len);
int usb_cdc_write_ready(void);  // Returns non-zero if ready to accept more data

#endif
