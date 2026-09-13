#include "apdu.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>

enum { MAX_STEPS = 12, MAX_FRAME = 300 };

typedef struct {
    uint8_t tx[MAX_FRAME], rx[MAX_FRAME];
    size_t tx_len, rx_len;
    int rc, receive_only;
} step;

static step script[MAX_STEPS];
static size_t script_len, call_count;
static uint64_t clock_ms;
static unsigned previous_timeout;

static uint8_t xsum(const uint8_t *p, size_t n) {
    uint8_t x = 0;
    while (n--) x ^= *p++;
    return x;
}

static size_t t1(uint8_t *out, uint8_t pcb, const uint8_t *p, size_t n) {
    out[0] = 'R'; out[1] = pcb; out[2] = (uint8_t)n;
    if (n) memcpy(out + 3, p, n);
    out[3 + n] = xsum(out, 3 + n);
    return n + 4;
}

static size_t t0(uint8_t *out, uint8_t pcb, const uint8_t *p, size_t n) {
    out[0] = 'R'; out[1] = pcb; out[2] = (uint8_t)(n >> 8); out[3] = (uint8_t)n;
    if (n) memcpy(out + 4, p, n);
    out[4 + n] = xsum(out, 4 + n);
    return n + 5;
}

static step *add_exchange(uint8_t pcb, const uint8_t *tx, size_t tx_len,
                          uint8_t reply_pcb, const uint8_t *reply, size_t reply_len) {
    step *s = &script[script_len++];
    s->tx_len = t1(s->tx, pcb, tx, tx_len);
    s->rx_len = t1(s->rx, reply_pcb, reply, reply_len);
    return s;
}

static void reset_script(void) {
    memset(script, 0, sizeof(script));
    script_len = call_count = 0;
    clock_ms = 100;
    previous_timeout = 0;
}

uint64_t rw_now_ms(void) { return clock_ms++; }
void rw_sleep_ms(unsigned ms) { clock_ms += ms; }

static int deliver(step *s, uint8_t *rx, size_t *rx_len, unsigned timeout) {
    assert(timeout > 0);
    if (previous_timeout) assert(timeout <= previous_timeout);
    previous_timeout = timeout;
    if (s->rc != RW_OK) return s->rc;
    assert(*rx_len >= s->rx_len);
    memcpy(rx, s->rx, s->rx_len);
    *rx_len = s->rx_len;
    return RW_OK;
}

int rw_exchange(rw_device *d, const uint8_t *tx, size_t tx_len,
                uint8_t *rx, size_t *rx_len, unsigned timeout) {
    step *s;
    (void)d;
    assert(call_count < script_len);
    s = &script[call_count++];
    assert(!s->receive_only);
    assert(tx_len == s->tx_len && memcmp(tx, s->tx, tx_len) == 0);
    return deliver(s, rx, rx_len, timeout);
}

int rw_receive(rw_device *d, uint8_t *rx, size_t *rx_len, unsigned timeout) {
    step *s;
    (void)d;
    assert(call_count < script_len);
    s = &script[call_count++];
    assert(s->receive_only);
    return deliver(s, rx, rx_len, timeout);
}

static rw_device fresh_t1(uint8_t ifsc) {
    rw_device d;
    memset(&d, 0, sizeof(d));
    d.protocol = RW_PROTOCOL_T1;
    d.ifsc = ifsc;
    return d;
}

static void simple_select(void) {
    const uint8_t apdu[] = {0x00,0xa4,0x00,0x0c,0x02,0x3f,0x00};
    const uint8_t sw[] = {0x90,0x00};
    uint8_t out[8]; size_t n = sizeof(out); rw_device d = fresh_t1(0xfe);
    reset_script(); add_exchange(0x00, apdu, sizeof(apdu), 0x00, sw, sizeof(sw));
    assert(rw_apdu_transmit(&d, apdu, sizeof(apdu), out, &n, 100) == RW_OK);
    assert(call_count == 1 && n == 2 && !memcmp(out, sw, 2));
    assert(d.ns == 1 && d.nr == 1);
}

static void chained_command(void) {
    const uint8_t apdu[] = {1,2,3}, first[] = {1,2}, last[] = {3}, sw[] = {0x90,0};
    uint8_t out[8]; size_t n = sizeof(out); rw_device d = fresh_t1(2);
    reset_script();
    add_exchange(0x20, first, 2, 0x90, NULL, 0);
    add_exchange(0x40, last, 1, 0x00, sw, 2);
    assert(rw_apdu_transmit(&d, apdu, 3, out, &n, 100) == RW_OK);
    assert(call_count == 2 && n == 2 && d.ns == 0 && d.nr == 1);
}

static void chained_response(void) {
    const uint8_t apdu[] = {0,0x84,0,0,8}, a[] = {0x11}, b[] = {0x22,0x90,0};
    uint8_t out[8]; size_t n = sizeof(out); rw_device d = fresh_t1(32);
    reset_script();
    add_exchange(0x00, apdu, sizeof(apdu), 0x20, a, 1);
    add_exchange(0x90, NULL, 0, 0x40, b, 3);
    assert(rw_apdu_transmit(&d, apdu, sizeof(apdu), out, &n, 100) == RW_OK);
    assert(n == 4 && out[0] == 0x11 && out[1] == 0x22 && d.ns == 1 && d.nr == 0);
}

static void receive_recovery(void) {
    const uint8_t apdu[] = {0,0x84}, sw[] = {0x90,0};
    uint8_t out[8]; size_t n; rw_device d;
    step *s;

    reset_script(); d = fresh_t1(32); n = sizeof(out);
    s = add_exchange(0x00, apdu, 2, 0x00, sw, 2); s->rx[s->rx_len - 1] ^= 1;
    add_exchange(0x81, NULL, 0, 0x00, sw, 2);
    assert(rw_apdu_transmit(&d, apdu, 2, out, &n, 100) == RW_OK && call_count == 2);

    reset_script(); d = fresh_t1(32); n = sizeof(out);
    add_exchange(0x00, apdu, 2, 0x40, sw, 2);
    add_exchange(0x82, NULL, 0, 0x00, sw, 2);
    assert(rw_apdu_transmit(&d, apdu, 2, out, &n, 100) == RW_OK && call_count == 2);

    reset_script(); d = fresh_t1(32); n = sizeof(out);
    s = add_exchange(0x00, apdu, 2, 0, NULL, 0);
    s->rx[0] = 0x21; s->rx[1] = 0; s->rx[2] = 4; s->rx[3] = 0;
    s->rx[4] = 0; s->rx[5] = 0xf0; s->rx[6] = 0xad;
    s->rx[7] = xsum(s->rx, 7); s->rx_len = 8;
    add_exchange(0x82, NULL, 0, 0x00, sw, 2);
    assert(rw_apdu_transmit(&d, apdu, 2, out, &n, 100) == RW_OK && call_count == 2);
}

static void explicit_retransmit(void) {
    const uint8_t apdu[] = {0,0x84}, sw[] = {0x90,0};
    uint8_t out[8]; size_t n = sizeof(out); rw_device d = fresh_t1(32);
    reset_script();
    add_exchange(0x00, apdu, 2, 0x80, NULL, 0);
    add_exchange(0x00, apdu, 2, 0x00, sw, 2);
    assert(rw_apdu_transmit(&d, apdu, 2, out, &n, 100) == RW_OK && call_count == 2);
}

static void supervisory(void) {
    const uint8_t apdu[] = {0,0x84}, sw[] = {0x90,0}, ifs[] = {0x10}, wtx[] = {2};
    uint8_t out[8]; size_t n; rw_device d;

    reset_script(); d = fresh_t1(32); n = sizeof(out);
    add_exchange(0x00, apdu, 2, 0xc1, ifs, 1);
    add_exchange(0xe1, ifs, 1, 0x00, sw, 2);
    assert(rw_apdu_transmit(&d, apdu, 2, out, &n, 100) == RW_OK && d.ifsc == 0x10);

    reset_script(); d = fresh_t1(32); n = sizeof(out);
    add_exchange(0x00, apdu, 2, 0xc3, wtx, 1);
    add_exchange(0xe3, wtx, 1, 0x00, sw, 2);
    assert(rw_apdu_transmit(&d, apdu, 2, out, &n, 100) == RW_OK);

    reset_script(); d = fresh_t1(32); n = sizeof(out);
    add_exchange(0x00, apdu, 2, 0xc2, NULL, 0);
    add_exchange(0xe2, NULL, 0, 0x00, sw, 2);
    assert(rw_apdu_transmit(&d, apdu, 2, out, &n, 100) == RW_ERROR_CARD);
}

static void no_uncertain_replay(void) {
    const uint8_t apdu[] = {0,0x84};
    uint8_t out[8]; size_t n; rw_device d; step *s;

    reset_script(); d = fresh_t1(32); d.ready = 1; n = sizeof(out);
    add_exchange(0x00, apdu, 2, 0xe0, NULL, 0);
    assert(rw_apdu_transmit(&d, apdu, 2, out, &n, 100) == RW_ERROR_CARD);
    assert(call_count == 1 && n == 0 && d.ready == 0);

    reset_script(); d = fresh_t1(32); d.ready = 1; n = sizeof(out);
    s = add_exchange(0x00, apdu, 2, 0, NULL, 0); s->rc = RW_ERROR_TIMEOUT;
    assert(rw_apdu_transmit(&d, apdu, 2, out, &n, 100) == RW_ERROR_TIMEOUT);
    assert(call_count == 1 && n == 0 && d.ready == 0);
}

static void buffer_count(void) {
    const uint8_t apdu[] = {0,0x84}, reply[] = {1,2,3};
    uint8_t out[2]; size_t n = sizeof(out); rw_device d = fresh_t1(32);
    reset_script(); add_exchange(0x00, apdu, 2, 0x00, reply, 3);
    assert(rw_apdu_transmit(&d, apdu, 2, out, &n, 100) == RW_ERROR_BUFFER);
    assert(n == 3 && d.ns == 1 && d.nr == 1);
}

static void t0_null_and_case4(void) {
    const uint8_t apdu[] = {0,0xd6,0,0,2,0xaa,0xbb,0};
    const uint8_t tpdu[] = {0,0xd6,0,0,2,0xaa,0xbb};
    const uint8_t null_byte[] = {0x60}, sw[] = {0x90,0};
    uint8_t out[8]; size_t n = sizeof(out); rw_device d;
    step *s;
    memset(&d, 0, sizeof(d)); d.protocol = RW_PROTOCOL_T0;
    reset_script();
    s = &script[script_len++];
    s->tx_len = t0(s->tx, 0, tpdu, sizeof(tpdu));
    s->rx_len = t0(s->rx, 0, null_byte, 1);
    s = &script[script_len++]; s->receive_only = 1; s->rx_len = t0(s->rx, 0, sw, 2);
    assert(rw_apdu_transmit(&d, apdu, sizeof(apdu), out, &n, 100) == RW_OK);
    assert(call_count == 2 && n == 2 && !memcmp(out, sw, 2));
}

int main(void) {
    simple_select();
    chained_command();
    chained_response();
    receive_recovery();
    explicit_retransmit();
    supervisory();
    no_uncertain_replay();
    buffer_count();
    t0_null_and_case4();
    puts("APDU framing, sequencing, recovery, controls and deadline passed");
    return 0;
}
