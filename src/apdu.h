#ifndef RW_APDU_H
#define RW_APDU_H

#include "internal.h"

int rw_apdu_transmit(rw_device *device, const uint8_t *apdu, size_t apdu_len,
                     uint8_t *response, size_t *response_len,
                     unsigned timeout_ms);

#endif
