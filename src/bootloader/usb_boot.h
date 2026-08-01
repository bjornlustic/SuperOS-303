/*
 * Polled USB-MIDI device for the SysEx bootloader. See usb_boot.c.
 */
#ifndef USB_BOOT_H
#define USB_BOOT_H

void usb_boot_init(void);   /* start the USB controller and attach */
void usb_boot_task(void);   /* poll: enumeration + MIDI-OUT endpoint drain */
void usb_boot_detach(void); /* drop off the bus (no-op if init never ran) */

#endif
