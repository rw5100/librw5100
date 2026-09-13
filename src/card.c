#include "card.h"
#include "frame.h"
#include <string.h>

static unsigned left(uint64_t deadline) {
    uint64_t now=rw_now_ms();
    return now>=deadline?0:(unsigned)(deadline-now);
}
static int command(rw_device *d,uint8_t op,const uint8_t *args,size_t count,
                   uint8_t *reply,size_t *n,uint64_t deadline) {
    unsigned ms=left(deadline);
    return ms?rw_command(d,op,args,count,reply,n,ms):RW_ERROR_TIMEOUT;
}
static int simple(rw_device *d,uint8_t op,const uint8_t *args,size_t count,uint64_t deadline) {
    uint8_t reply[259]; size_t n=sizeof(reply);
    return command(d,op,args,count,reply,&n,deadline);
}
static int hex_digit(uint8_t c) {
    if(c>='0' && c<='9') return c-'0';
    if(c>='A' && c<='F') return c-'A'+10;
    if(c>='a' && c<='f') return c-'a'+10;
    return -1;
}
/* device 0x120c0: ATR starts at response byte 5. The device's original
 * host parser synthesizes TCK and excludes eight framing/trailer bytes. */
int rw_parse_atr(rw_device *d,const uint8_t *source,size_t available) {
    if(!d || !source || available<2 || available>64) return RW_ERROR_FRAME;
    uint8_t atr[64]; memcpy(atr,source,available);
    if(atr[0]!=0x3b && atr[0]!=0x3f) return RW_ERROR_FRAME;
    unsigned y=atr[1]>>4,k=atr[1]&15,group=1;
    size_t i=2; int nonzero=0,first=-1;
    unsigned mask=0; uint8_t ifsc=32;
    for(;;group++) {
        for(unsigned bit=0;bit<3;bit++) if(y&(1u<<bit)) {
            if(i>=available) return RW_ERROR_FRAME;
            if(group==3 && bit==0) ifsc=atr[i];
            i++;
        }
        if(!(y&8)) break;
        if(i>=available) return RW_ERROR_FRAME;
        uint8_t td=atr[i++]; unsigned protocol=td&15;
        if(first<0 && protocol!=15) first=(int)protocol;
        if(protocol<2) mask|=1u<<protocol;
        if(protocol) nonzero=1;
        y=td>>4;
    }
    if(i+k+(nonzero?1u:0u)>available) return RW_ERROR_FRAME;
    i+=k;
    if(nonzero) { atr[i]=rw_lrc(atr+1,i-1); i++; }
    if(first<0) { first=0; mask|=1; }
    memcpy(d->atr,atr,i); d->atr_len=i;
    d->protocol=(uint8_t)first; d->protocol_mask=(uint8_t)mask;
    d->ifsc=ifsc; d->ns=d->nr=0;
    return RW_OK;
}
static int get_atr(rw_device *d,uint64_t deadline) {
    const uint8_t args[]={0x0a,0};
    uint8_t reply[259]; size_t n=sizeof(reply);
    int e=command(d,0x35,args,2,reply,&n,deadline);
    if(e) return e;
    if(n<10 || n-8>64) return RW_ERROR_FRAME;
    return rw_parse_atr(d,reply+5,n-8);
}
static int status_at(rw_device *d,int *status,uint64_t deadline) {
    uint8_t reply[259]; size_t n=sizeof(reply);
    int e=command(d,0x31,NULL,0,reply,&n,deadline);
    if(e) return e;
    if(n<7) return RW_ERROR_FRAME;
    d->status_bits=reply[5];
    if(!(d->status_bits&0x20)) {
        /* device 0x12631: restore reader after status reports uninitialized. */
        if(left(deadline)<=25) return RW_ERROR_TIMEOUT;
        rw_sleep_ms(10);
        e=simple(d,0xd0,NULL,0,deadline);
        if(e) return e;
        rw_sleep_ms(15);
        e=simple(d,0x41,NULL,0,deadline);
        if(e) return e;
        d->status_bits|=0x20;
        d->ready=0; d->atr_len=0;
    }
    int state=(d->status_bits&0x38)==0x38?RW_CARD_POWERED:
        (d->status_bits&0x28)==0x28?RW_CARD_PRESENT:RW_CARD_ABSENT;
    if(state!=RW_CARD_POWERED) { d->ready=0; d->atr_len=0; }
    d->card_status=(uint8_t)state; *status=state;
    return RW_OK;
}
int rw_initialize_impl(rw_device *d,unsigned timeout) {
    uint64_t deadline=rw_now_ms()+timeout;
    uint8_t reply[259]; size_t n=sizeof(reply); int state;
    int e=command(d,0xc0,NULL,0,reply,&n,deadline);
    if(e) return e;
    if(n<17) return RW_ERROR_FRAME;
    int a=hex_digit(reply[12]),b=hex_digit(reply[14]),c=hex_digit(reply[15]);
    if(a<0 || b<0 || c<0) return RW_ERROR_FRAME;
    d->firmware_version=(uint16_t)((a<<8)|(b<<4)|c);
    e=status_at(d,&state,deadline);
    if(e) return e;
    if(state==RW_CARD_POWERED) return get_atr(d,deadline);
    return RW_OK;
}
int rw_status_impl(rw_device *d,int *status,unsigned timeout) {
    return status_at(d,status,rw_now_ms()+timeout);
}
int rw_reset_impl(rw_device *d,int warm,uint8_t *atr,size_t *n,unsigned timeout) {
    size_t cap=*n; *n=0;
    uint64_t deadline=rw_now_ms()+timeout;
    int state,e=status_at(d,&state,deadline);
    if(e) return e;
    if(state==RW_CARD_ABSENT) return RW_ERROR_NO_CARD;
    d->ready=0; d->atr_len=0;
    /* device 0x12fd9/0x12977: cold action=1 -> 0; warm action=2 -> 3. */
    uint8_t arg=warm?3:0;
    if(d->status_bits==0x28) {
        if(left(deadline)<=500) return RW_ERROR_TIMEOUT;
        rw_sleep_ms(500);
    }
    e=simple(d,0xd1,&arg,1,deadline);
    if(!e) e=get_atr(d,deadline);
    if(e) return e;
    d->card_status=RW_CARD_POWERED;
    /* device 0x12e2c uses D1 to select the requested PC/SC protocol.
     * D8 belongs to an unrelated vendor control, not protocol selection. */
    if(d->protocol>1) return RW_ERROR_UNSUPPORTED;
    arg=d->protocol;
    e=simple(d,0xd1,&arg,1,deadline);
    if(e) return e;
    d->ready=1;
    *n=d->atr_len;
    if(cap<d->atr_len) return RW_ERROR_BUFFER;
    memcpy(atr,d->atr,d->atr_len); return RW_OK;
}
int rw_set_protocol_impl(rw_device *d,int protocol,unsigned timeout) {
    if(!d->atr_len || d->card_status!=RW_CARD_POWERED) return RW_ERROR_STATE;
    if(!(d->protocol_mask&(1u<<protocol))) return RW_ERROR_UNSUPPORTED;
    if(d->ready && d->protocol==protocol) return RW_OK;
    uint8_t arg=(uint8_t)protocol;
    d->ready=0;
    int e=simple(d,0xd1,&arg,1,rw_now_ms()+timeout);
    if(e) return e;
    d->protocol=arg; d->ns=d->nr=0; d->ready=1; return RW_OK;
}
int rw_apdu_transmit(rw_device *,const uint8_t *,size_t,uint8_t *,size_t *,unsigned);
int rw_transmit_impl(rw_device *d,const uint8_t *tx,size_t tn,uint8_t *rx,size_t *rn,unsigned timeout) {
    if(!d->ready) { *rn=0; return RW_ERROR_STATE; }
    return rw_apdu_transmit(d,tx,tn,rx,rn,timeout);
}
int rw_power_off_impl(rw_device *d,unsigned timeout) {
    uint64_t deadline=rw_now_ms()+timeout;
    int state,e=status_at(d,&state,deadline);
    if(e) return e;
    if(state==RW_CARD_ABSENT) return RW_ERROR_NO_CARD;
    if(left(deadline)<=25) return RW_ERROR_TIMEOUT;
    d->ready=0; d->atr_len=0;
    rw_sleep_ms(10); e=simple(d,0xd0,NULL,0,deadline);
    if(e) return e;
    rw_sleep_ms(15);
    e=simple(d,0x41,NULL,0,deadline);
    if(e) return e;
    d->card_status=RW_CARD_PRESENT; d->ns=d->nr=0;
    return RW_OK;
}
