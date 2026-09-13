#ifndef RW5100_H
#define RW5100_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct rw_device rw_device;
enum rw_error {
    RW_OK=0, RW_ERROR_ARGUMENT=-1, RW_ERROR_NO_DEVICE=-2,
    RW_ERROR_ACCESS=-3, RW_ERROR_BUSY=-4, RW_ERROR_NO_CARD=-5,
    RW_ERROR_TIMEOUT=-6, RW_ERROR_DISCONNECTED=-7, RW_ERROR_FRAME=-8,
    RW_ERROR_UNSUPPORTED=-9, RW_ERROR_BUFFER=-10, RW_ERROR_IO=-11,
    RW_ERROR_MEMORY=-12, RW_ERROR_STATE=-13, RW_ERROR_CARD=-14
};
enum rw_protocol { RW_PROTOCOL_T0=0, RW_PROTOCOL_T1=1 };
enum rw_card_status { RW_CARD_ABSENT=0, RW_CARD_PRESENT=1, RW_CARD_POWERED=2 };
typedef struct rw_device_info { uint8_t bus, address; uint16_t vendor, product; } rw_device_info;
/* count is capacity on input and required count on output; NULL/0 queries count. */
int rw_enumerate(rw_device_info *devices, size_t *count);
/* NULL selector is accepted only when exactly one matching reader exists. */
int rw_open(rw_device **out, const rw_device_info *selector);
void rw_close(rw_device *device);
int rw_status(rw_device *device, int *status, unsigned timeout_ms);
int rw_reset(rw_device *device, int warm, uint8_t *atr, size_t *atr_len, unsigned timeout_ms);
int rw_set_protocol(rw_device *device, int protocol, unsigned timeout_ms);
int rw_transmit(rw_device *device, const uint8_t *apdu, size_t apdu_len,
                uint8_t *response, size_t *response_len, unsigned timeout_ms);
int rw_power_off(rw_device *device, unsigned timeout_ms);
const char *rw_strerror(int error);
/* timeout_ms must be nonzero. Response lengths are zero on failure except
 * RW_ERROR_BUFFER, which reports required capacity. A transmit buffer error
 * does not authorize replay: the card may already have executed the command. */
#ifdef __cplusplus
}
#endif
#endif
