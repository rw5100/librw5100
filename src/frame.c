#include "frame.h"
#include "rw5100.h"
#include <string.h>
uint8_t rw_lrc(const uint8_t *p, size_t n) {
    uint8_t x=0;
    while(n--) x^=*p++;
    return x;
}
int rw_frame_encode(uint8_t sequence, uint8_t opcode, const uint8_t *args,
                    size_t nargs, uint8_t *out, size_t *len) {
    if(!len || (nargs && !args) || nargs>253) return RW_ERROR_ARGUMENT;
    size_t cap=*len; *len=nargs+6;
    if(!out || cap<*len) return RW_ERROR_BUFFER;
    out[0]=0x12; out[1]=sequence; out[2]=(uint8_t)(nargs+2);
    out[3]=1; out[4]=opcode;
    if(nargs) memcpy(out+5,args,nargs);
    out[nargs+5]=rw_lrc(out,nargs+5);
    return RW_OK;
}
int rw_frame_validate(const uint8_t *p, size_t n) {
    if(!p || n<4 || n!=(size_t)p[2]+4 || rw_lrc(p,n)) return RW_ERROR_FRAME;
    return RW_OK;
}
