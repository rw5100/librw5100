#include "card.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); return 1; } } while(0)
static uint64_t now;
static uint8_t ops[32],arg0[32],arg1[32],status_bits=0x28;
static size_t counts[32],calls;
static int fail_at=-1;
uint64_t rw_now_ms(void) { return now; }
void rw_sleep_ms(unsigned ms) { now+=ms; }
int rw_apdu_transmit(rw_device *d,const uint8_t *t,size_t n,uint8_t *r,size_t *s,unsigned ms) {
    (void)d;(void)t;(void)n;(void)r;(void)s;(void)ms;return RW_ERROR_CARD;
}
int rw_command(rw_device *d,uint8_t op,const uint8_t *args,size_t nargs,uint8_t *r,size_t *n,unsigned timeout) {
    (void)d; (void)timeout;
    if(calls>=32) return RW_ERROR_IO;
    ops[calls]=op; counts[calls]=nargs; arg0[calls]=nargs?args[0]:0; arg1[calls]=nargs>1?args[1]:0;
    if((int)calls++==fail_at) return RW_ERROR_TIMEOUT;
    uint8_t reply[32]={0x21,0,3,1,0}; size_t size=7;
    if(op==0x31) reply[5]=status_bits;
    if(op==0xc0) { size=20; reply[12]='1';reply[14]='0';reply[15]='9'; }
    if(op==0x35) { size=10;reply[5]=0x3b;reply[6]=0; }
    if(*n<size) { *n=size;return RW_ERROR_BUFFER; }
    memcpy(r,reply,size); *n=size; return 0;
}
static void reset(void) { calls=0;now=0;fail_at=-1;status_bits=0x28; }
int main(void) {
    rw_device d={0}; int status; uint8_t out[64]; size_t n;
    reset(); CHECK(!rw_status_impl(&d,&status,1000)); CHECK(status==RW_CARD_PRESENT);
    status_bits=0x38;CHECK(!rw_status_impl(&d,&status,1000));CHECK(status==RW_CARD_POWERED);
    status_bits=0x20;CHECK(!rw_status_impl(&d,&status,1000));CHECK(status==RW_CARD_ABSENT);
    reset();CHECK(!rw_initialize_impl(&d,1000));CHECK(d.firmware_version==0x109 && !d.ready);
    reset();n=sizeof(out); CHECK(!rw_reset_impl(&d,0,out,&n,1000));
    CHECK(calls==4 && ops[0]==0x31 && ops[1]==0xd1 && arg0[1]==0);
    CHECK(ops[2]==0x35 && counts[2]==2 && arg0[2]==10 && arg1[2]==0);
    CHECK(ops[3]==0xd1 && n==2 && out[0]==0x3b && d.ready);
    reset();n=sizeof(out);CHECK(!rw_reset_impl(&d,1,out,&n,1000));CHECK(arg0[1]==3);
    reset();n=1;CHECK(rw_reset_impl(&d,0,out,&n,1000)==RW_ERROR_BUFFER);CHECK(n==2);
    reset();status_bits=0x20;n=sizeof(out);CHECK(rw_reset_impl(&d,0,out,&n,1000)==RW_ERROR_NO_CARD);CHECK(n==0 && calls==1);
    reset();fail_at=1;n=sizeof(out);CHECK(rw_reset_impl(&d,0,out,&n,1000)==RW_ERROR_TIMEOUT);CHECK(n==0 && !d.ready);
    reset();n=sizeof(out);CHECK(rw_reset_impl(&d,0,out,&n,100)==RW_ERROR_TIMEOUT);CHECK(calls==1);
    const uint8_t t1[]={0x3b,0x80,0x01,0};
    CHECK(!rw_parse_atr(&d,t1,sizeof(t1)));CHECK(d.atr_len==4 && d.atr[3]==0x81 && d.protocol==1);
    CHECK(rw_parse_atr(&d,t1,2)==RW_ERROR_FRAME);
    const uint8_t bad[]={0x3b,0xf0}; CHECK(rw_parse_atr(&d,bad,2)==RW_ERROR_FRAME);
    d.card_status=RW_CARD_POWERED;d.ready=0;
    reset();CHECK(!rw_set_protocol_impl(&d,1,1000));CHECK(ops[0]==0xd1 && arg0[0]==1);
    CHECK(rw_set_protocol_impl(&d,0,1000)==RW_ERROR_UNSUPPORTED);
    reset();CHECK(!rw_power_off_impl(&d,1000));CHECK(calls==3 && ops[1]==0xd0 && ops[2]==0x41 && !d.ready);
    puts("card status, reset, ATR, protocol and failures passed");return 0;
}
