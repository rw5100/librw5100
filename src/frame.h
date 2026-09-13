#ifndef RW_FRAME_H
#define RW_FRAME_H
#include <stddef.h>
#include <stdint.h>
uint8_t rw_lrc(const uint8_t *, size_t);
int rw_frame_encode(uint8_t sequence, uint8_t opcode, const uint8_t *args,
                    size_t nargs, uint8_t *out, size_t *len);
int rw_frame_validate(const uint8_t *, size_t);
#endif
