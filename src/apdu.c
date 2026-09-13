#include "apdu.h"

#include <stdlib.h>
#include <string.h>

enum {
    RW_R_T1_HEADER = 3,
    RW_R_T0_HEADER = 4,
    RW_R_LRC = 1,
    RW_APDU_MAX = 0x10006,
    RW_T0_FRAME_MAX = 0x10004,
    RW_T1_FRAME_MAX = 259,
    RW_T1_RETRY_LIMIT = 3,
    RW_T1_CONTROL_LIMIT = 32,
    RW_T0_NULL_LIMIT = 4096
};

typedef enum { RW_BLOCK_I, RW_BLOCK_R, RW_BLOCK_S } rw_block_kind;

typedef struct {
    rw_block_kind kind;
    uint8_t pcb;
    const uint8_t *data;
    size_t data_len;
} rw_block;

static uint8_t xor_bytes(const uint8_t *bytes, size_t count) {
    uint8_t value = 0;
    while (count--) value ^= *bytes++;
    return value;
}

static unsigned time_left(uint64_t deadline) {
    uint64_t now = rw_now_ms();
    return now >= deadline ? 0 : (unsigned)(deadline - now);
}

static int exchange_deadline(rw_device *device, const uint8_t *tx, size_t tx_len,
                             uint8_t *rx, size_t *rx_len, uint64_t deadline) {
    unsigned left = time_left(deadline);
    return left ? rw_exchange(device, tx, tx_len, rx, rx_len, left) : RW_ERROR_TIMEOUT;
}

static int receive_deadline(rw_device *device, uint8_t *rx, size_t *rx_len,
                            uint64_t deadline) {
    unsigned left = time_left(deadline);
    return left ? rw_receive(device, rx, rx_len, left) : RW_ERROR_TIMEOUT;
}

static int append_response(uint8_t *out, size_t capacity, size_t *used,
                           const uint8_t *data, size_t count, int *overflow) {
    if (count > SIZE_MAX - *used) return RW_ERROR_BUFFER;
    if (!*overflow && count <= capacity - *used) {
        if (count) memcpy(out + *used, data, count);
    } else {
        *overflow = 1;
    }
    *used += count;
    return RW_OK;
}

static size_t frame_t1(uint8_t *frame, uint8_t pcb,
                       const uint8_t *data, size_t data_len) {
    frame[0] = 'R';
    frame[1] = pcb;
    frame[2] = (uint8_t)data_len;
    if (data_len) memcpy(frame + RW_R_T1_HEADER, data, data_len);
    frame[RW_R_T1_HEADER + data_len] = xor_bytes(frame, RW_R_T1_HEADER + data_len);
    return RW_R_T1_HEADER + data_len + RW_R_LRC;
}

static size_t frame_t0(uint8_t *frame, uint8_t pcb,
                       const uint8_t *data, size_t data_len) {
    frame[0] = 'R';
    frame[1] = pcb;
    frame[2] = (uint8_t)(data_len >> 8);
    frame[3] = (uint8_t)data_len;
    if (data_len) memcpy(frame + RW_R_T0_HEADER, data, data_len);
    frame[RW_R_T0_HEADER + data_len] = xor_bytes(frame, RW_R_T0_HEADER + data_len);
    return RW_R_T0_HEADER + data_len + RW_R_LRC;
}

static int parse_t1(const uint8_t *frame, size_t frame_len, rw_block *block) {
    size_t data_len;
    uint8_t family;
    if (frame_len < RW_R_T1_HEADER + RW_R_LRC)
        return RW_ERROR_FRAME;
    data_len = frame[2];
    if (data_len != frame_len - RW_R_T1_HEADER - RW_R_LRC ||
        xor_bytes(frame, frame_len) != 0)
        return RW_ERROR_FRAME;
    block->pcb = frame[1];
    block->data = frame + RW_R_T1_HEADER;
    block->data_len = data_len;
    family = block->pcb & 0xc0u;
    /* The device classifies both 0x00 and 0x40 as I blocks. */
    block->kind = family == 0x80u ? RW_BLOCK_R :
                  family == 0xc0u ? RW_BLOCK_S : RW_BLOCK_I;
    return RW_OK;
}

static int parse_t0(const uint8_t *frame, size_t frame_len,
                    const uint8_t **data, size_t *data_len) {
    size_t length;
    if (frame_len < RW_R_T0_HEADER + RW_R_LRC)
        return RW_ERROR_FRAME;
    length = ((size_t)frame[2] << 8) | frame[3];
    if (length != frame_len - RW_R_T0_HEADER - RW_R_LRC ||
        xor_bytes(frame, frame_len) != 0)
        return RW_ERROR_FRAME;
    *data = frame + RW_R_T0_HEADER;
    *data_len = length;
    return RW_OK;
}

static int is_device_nak(const uint8_t *frame, size_t frame_len) {
    return frame_len == 8 && frame[0] == 0x21 && frame[2] == 4 &&
           (frame[5] & 0xf0u) == 0xf0u && frame[6] == 0xadu &&
           xor_bytes(frame, frame_len) == 0;
}

static int t0_tpdu_length(const uint8_t *apdu, size_t apdu_len, size_t *tpdu_len) {
    uint8_t lc;
    if (apdu_len < 4) return RW_ERROR_ARGUMENT;
    if (apdu_len <= 5) {
        *tpdu_len = apdu_len;
        return RW_OK;
    }
    lc = apdu[4];
    if (lc == 0 || apdu_len < (size_t)lc + 5)
        return RW_ERROR_ARGUMENT;
    if (apdu_len != (size_t)lc + 5 && apdu_len != (size_t)lc + 6)
        return RW_ERROR_ARGUMENT;
    /* At 0x13840..0x1386a the device driver drops case-4 Le and sends the
     * command TPDU through the reader. SW1/SW2 still pass through verbatim. */
    *tpdu_len = (size_t)lc + 5;
    return RW_OK;
}

static int transmit_t0(rw_device *device, const uint8_t *apdu, size_t apdu_len,
                       uint8_t *response, size_t *response_len, uint64_t deadline) {
    uint8_t *tx, *rx;
    const uint8_t *data;
    size_t tpdu_len, tx_len, rx_len, data_len, used = 0;
    size_t capacity = *response_len;
    unsigned null_count = 0;
    int overflow = 0, rc;

    rc = t0_tpdu_length(apdu, apdu_len, &tpdu_len);
    if (rc != RW_OK) return rc;
    if (tpdu_len > 0xffffu) return RW_ERROR_UNSUPPORTED;
    tx = malloc(tpdu_len + RW_R_T0_HEADER + RW_R_LRC);
    rx = malloc(RW_T0_FRAME_MAX);
    if (tx == NULL || rx == NULL) {
        free(tx);
        free(rx);
        return RW_ERROR_MEMORY;
    }
    tx_len = frame_t0(tx, device->ns ? 0x40u : 0, apdu, tpdu_len);
    rx_len = RW_T0_FRAME_MAX;
    rc = exchange_deadline(device, tx, tx_len, rx, &rx_len, deadline);
    while (rc == RW_OK) {
        if (rx_len && rx[0] == 0x21) {
            rc = RW_ERROR_CARD;
            break;
        }
        rc = parse_t0(rx, rx_len, &data, &data_len);
        if (rc != RW_OK) break;
        /* T=0 NULL is a complete one-byte R record. The driver queues the
         * next IN at 0x13a89..0x13b17 without replaying the command. */
        if (data_len == 1 && data[0] == 0x60) {
            if (++null_count > RW_T0_NULL_LIMIT) {
                rc = RW_ERROR_CARD;
                break;
            }
            rx_len = RW_T0_FRAME_MAX;
            rc = receive_deadline(device, rx, &rx_len, deadline);
            continue;
        }
        rc = append_response(response, capacity, &used, data, data_len, &overflow);
        break;
    }
    *response_len = used;
    free(rx);
    free(tx);
    if (rc != RW_OK) return rc;
    return overflow ? RW_ERROR_BUFFER : RW_OK;
}

static int transmit_t1(rw_device *device, const uint8_t *apdu, size_t apdu_len,
                       uint8_t *response, size_t *response_len, uint64_t deadline) {
    uint8_t tx[RW_T1_FRAME_MAX], rx[RW_T1_FRAME_MAX], last_i[RW_T1_FRAME_MAX];
    rw_block block;
    size_t capacity = *response_len, used = 0, offset = 0;
    size_t tx_len, rx_len, last_i_len, last_data_len;
    size_t chunk_limit = device->ifsc ? device->ifsc : 32u;
    size_t chunks = (apdu_len + chunk_limit - 1) / chunk_limit;
    size_t steps = 0, step_limit = chunks * (RW_T1_RETRY_LIMIT + 1u) + RW_T1_CONTROL_LIMIT;
    unsigned retries = 0, control_count = 0;
    int overflow = 0, abort_pending = 0, i_pending = 1, rc;

    last_data_len = apdu_len < chunk_limit ? apdu_len : chunk_limit;
    tx_len = frame_t1(tx,
                      (uint8_t)((device->ns ? 0x40u : 0) |
                                (last_data_len < apdu_len ? 0x20u : 0)),
                      apdu, last_data_len);
    memcpy(last_i, tx, tx_len);
    last_i_len = tx_len;

    while (++steps <= step_limit) {
        rx_len = sizeof(rx);
        rc = exchange_deadline(device, tx, tx_len, rx, &rx_len, deadline);
        if (rc != RW_OK) {
            *response_len = used;
            return rc; /* Never replay the APDU after uncertain delivery. */
        }
        if (abort_pending) {
            *response_len = used;
            return RW_ERROR_CARD;
        }
        if (rx_len && rx[0] == 0x21) {
            if (!is_device_nak(rx, rx_len) || ++retries > RW_T1_RETRY_LIMIT) {
                *response_len = used;
                return RW_ERROR_CARD;
            }
            /* The device routes its F?/AD NAK through state 2 at
             * 0x13313..0x13655, which emits an R(other-error) block. */
            tx_len = frame_t1(tx, (uint8_t)(0x82u | (device->nr ? 0x10u : 0)), NULL, 0);
            continue;
        }
        rc = parse_t1(rx, rx_len, &block);
        if (rc != RW_OK) {
            if (++retries > RW_T1_RETRY_LIMIT) {
                *response_len = used;
                return RW_ERROR_CARD;
            }
            tx_len = frame_t1(tx, (uint8_t)(0x81u | (device->nr ? 0x10u : 0)), NULL, 0);
            continue;
        }

        if (block.kind == RW_BLOCK_I) {
            uint8_t incoming_ns = (block.pcb & 0x40u) ? 1u : 0u;
            if (incoming_ns != device->nr) {
                if (++retries > RW_T1_RETRY_LIMIT) {
                    *response_len = used;
                    return RW_ERROR_CARD;
                }
                tx_len = frame_t1(tx, (uint8_t)(0x82u | (device->nr ? 0x10u : 0)), NULL, 0);
                continue;
            }
            retries = 0;
            if (i_pending) {
                offset += last_data_len;
                device->ns ^= 1u;
                i_pending = 0;
            }
            if (offset < apdu_len) {
                /* An early I response is not a safe point from which to replay. */
                *response_len = used;
                return RW_ERROR_CARD;
            }
            rc = append_response(response, capacity, &used, block.data, block.data_len, &overflow);
            if (rc != RW_OK) {
                *response_len = used;
                return rc;
            }
            device->nr ^= 1u;
            if (!(block.pcb & 0x20u)) {
                *response_len = used;
                return overflow ? RW_ERROR_BUFFER : RW_OK;
            }
            if (++control_count > RW_T1_CONTROL_LIMIT) break;
            tx_len = frame_t1(tx, (uint8_t)(0x80u | (device->nr ? 0x10u : 0)), NULL, 0);
            continue;
        }

        if (block.kind == RW_BLOCK_R) {
            uint8_t error = block.pcb & 0x03u;
            uint8_t requested_ns = (block.pcb & 0x10u) ? 1u : 0u;
            if (block.data_len != 0 || error == 3u || !i_pending) {
                *response_len = used;
                return RW_ERROR_CARD;
            }
            if (requested_ns == (uint8_t)(device->ns ^ 1u)) {
                offset += last_data_len;
                device->ns ^= 1u;
                i_pending = 0;
                retries = 0;
                if (offset >= apdu_len) {
                    *response_len = used;
                    return RW_ERROR_CARD;
                }
                last_data_len = apdu_len - offset;
                if (last_data_len > chunk_limit) last_data_len = chunk_limit;
                tx_len = frame_t1(tx,
                                  (uint8_t)((device->ns ? 0x40u : 0) |
                                            (offset + last_data_len < apdu_len ? 0x20u : 0)),
                                  apdu + offset, last_data_len);
                memcpy(last_i, tx, tx_len);
                last_i_len = tx_len;
                i_pending = 1;
                continue;
            }
            /* N(R)==N(S) explicitly asks for the same unaccepted I block. */
            if (requested_ns != device->ns || ++retries > RW_T1_RETRY_LIMIT) {
                *response_len = used;
                return RW_ERROR_CARD;
            }
            memcpy(tx, last_i, last_i_len);
            tx_len = last_i_len;
            continue;
        }

        /* Device 0x13476 dispatches C1/C2/C3 requests and E0..E3 responses. */
        if (++control_count > RW_T1_CONTROL_LIMIT) break;
        switch (block.pcb & 0x3fu) {
        case 0x01: /* IFS request */
            if (block.data_len != 1 || block.data[0] == 0) {
                *response_len = used;
                return RW_ERROR_CARD;
            }
            device->ifsc = block.data[0];
            chunk_limit = block.data[0];
            tx_len = frame_t1(tx, 0xe1u, block.data, 1);
            retries = 0;
            break;
        case 0x02: /* ABORT request */
            if (block.data_len != 0) {
                *response_len = used;
                return RW_ERROR_CARD;
            }
            tx_len = frame_t1(tx, 0xe2u, NULL, 0);
            abort_pending = 1;
            break;
        case 0x03: /* WTX request */
            if (block.data_len != 1 || block.data[0] == 0) {
                *response_len = used;
                return RW_ERROR_CARD;
            }
            /* Echo WTX while retaining the API call's one absolute deadline. */
            tx_len = frame_t1(tx, 0xe3u, block.data, 1);
            retries = 0;
            break;
        default:
            /* E0 makes the original driver rebuild and replay at
             * 0x134f0..0x13537. Delivery is uncertain, so this API stops. */
            *response_len = used;
            return RW_ERROR_CARD;
        }
    }
    *response_len = used;
    return RW_ERROR_CARD;
}

int rw_apdu_transmit(rw_device *device, const uint8_t *apdu, size_t apdu_len,
                     uint8_t *response, size_t *response_len,
                     unsigned timeout_ms) {
    uint64_t deadline;
    size_t capacity, result_len;
    int rc;
    if (device == NULL || apdu == NULL || apdu_len == 0 ||
        response_len == NULL || timeout_ms == 0)
        return RW_ERROR_ARGUMENT;
    capacity = *response_len;
    *response_len = 0;
    if (response == NULL && capacity != 0) return RW_ERROR_ARGUMENT;
    if (apdu_len > RW_APDU_MAX) return RW_ERROR_UNSUPPORTED;
    if (device->protocol != RW_PROTOCOL_T0 && device->protocol != RW_PROTOCOL_T1)
        return RW_ERROR_STATE;
    deadline = rw_now_ms() + timeout_ms;
    result_len = capacity;
    rc = device->protocol == RW_PROTOCOL_T0 ?
        transmit_t0(device, apdu, apdu_len, response, &result_len, deadline) :
        transmit_t1(device, apdu, apdu_len, response, &result_len, deadline);
    if (rc == RW_OK && result_len < 2) rc = RW_ERROR_FRAME;
    if (rc == RW_OK || rc == RW_ERROR_BUFFER) {
        *response_len = result_len;
    } else {
        *response_len = 0;
        /* A protocol failure after submission leaves N(S)/N(R), and possibly
         * card execution, uncertain. Force a reset before another APDU. */
        if (rc != RW_ERROR_ARGUMENT && rc != RW_ERROR_UNSUPPORTED &&
            rc != RW_ERROR_MEMORY && rc != RW_ERROR_STATE)
            device->ready = 0;
    }
    return rc;
}
