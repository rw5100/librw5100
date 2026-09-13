#ifndef RW_CARD_H
#define RW_CARD_H

#include "internal.h"
int rw_parse_atr(rw_device *,const uint8_t *,size_t);

int rw_initialize_impl(rw_device *device, unsigned timeout_ms);
int rw_status_impl(rw_device *device, int *status, unsigned timeout_ms);
int rw_reset_impl(rw_device *device, int warm, uint8_t *atr, size_t *atr_len,
                  unsigned timeout_ms);
int rw_set_protocol_impl(rw_device *device, int protocol, unsigned timeout_ms);
int rw_transmit_impl(rw_device *device, const uint8_t *apdu, size_t apdu_len,
                     uint8_t *response, size_t *response_len,
                     unsigned timeout_ms);
int rw_power_off_impl(rw_device *device, unsigned timeout_ms);

#endif
