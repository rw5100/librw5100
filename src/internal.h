#ifndef RW_INTERNAL_H
#define RW_INTERNAL_H
#include "rw5100.h"
#include <stdatomic.h>
struct libusb_context;
struct libusb_device_handle;
struct rw_device {
    struct libusb_context *usb;
    struct libusb_device_handle *handle;
    int interface_number, alternate, detached, claimed;
    uint8_t ep_in, ep_out;
    uint16_t packet_in, packet_out;
    atomic_flag busy;
    uint8_t sequence, card_status, protocol, ifsc, ns, nr;
    uint8_t atr[64];
    size_t atr_len;
    int ready, poisoned;
    unsigned timeout_ms;
    uint16_t firmware_version;
    uint8_t status_bits, protocol_mask;
};
int rw_exchange(rw_device *, const uint8_t *, size_t, uint8_t *, size_t *, unsigned);
int rw_receive(rw_device *, uint8_t *, size_t *, unsigned);
int rw_command(rw_device *, uint8_t, const uint8_t *, size_t, uint8_t *, size_t *, unsigned);
uint64_t rw_now_ms(void);
void rw_sleep_ms(unsigned);
#endif
