#include "internal.h"
#include "frame.h"
#include <libusb.h>
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); return 1; } } while(0)
static uint8_t response[259];
static size_t response_len,offset;
static int calls,outs,zlps,fragment,fault;
int LIBUSB_CALL libusb_bulk_transfer(libusb_device_handle *h,unsigned char ep,
    unsigned char *data,int length,int *actual,unsigned ms) {
    (void)h; (void)ms; calls++; *actual=0;
    if(!(ep&0x80)) {
        outs++; if(!length) zlps++;
        if(fault==1) { *actual=length/2; return LIBUSB_ERROR_TIMEOUT; }
        *actual=length; return 0;
    }
    if(fault==2) return LIBUSB_ERROR_NO_DEVICE;
    size_t n=response_len-offset;
    if(n>(size_t)length) n=(size_t)length;
    if(fragment && n>(size_t)fragment) n=(size_t)fragment;
    memcpy(data,response+offset,n); offset+=n; *actual=(int)n; return 0;
}
static void setup(rw_device *d) {
    memset(d,0,sizeof(*d)); d->ep_out=2; d->ep_in=0x81; d->packet_in=64;
    const uint8_t r[]={0x21,0,3,1,0x31,0x38,0x2a};
    memcpy(response,r,sizeof(r)); response_len=sizeof(r);
    calls=outs=zlps=fragment=fault=0; offset=0;
}
int main(void) {
    rw_device d; uint8_t tx[64]={0},rx[259]; size_t n;
    setup(&d); fragment=2; n=sizeof(rx);
    CHECK(!rw_exchange(&d,tx,6,rx,&n,100)); CHECK(n==7 && outs==1 && calls==5);
    setup(&d); n=sizeof(rx); CHECK(!rw_exchange(&d,tx,64,rx,&n,100)); CHECK(zlps==1 && outs==2);
    setup(&d); fault=1; n=sizeof(rx);
    CHECK(rw_exchange(&d,tx,6,rx,&n,100)==RW_ERROR_TIMEOUT); CHECK(outs==1 && d.poisoned);
    CHECK(rw_exchange(&d,tx,6,rx,&n,100)==RW_ERROR_STATE); CHECK(outs==1);
    setup(&d); fault=2; n=sizeof(rx);
    CHECK(rw_exchange(&d,tx,6,rx,&n,100)==RW_ERROR_DISCONNECTED);
    setup(&d); response[6]^=1; n=sizeof(rx);
    CHECK(rw_exchange(&d,tx,6,rx,&n,100)==RW_ERROR_FRAME);
    setup(&d); n=2;
    CHECK(rw_exchange(&d,tx,6,rx,&n,100)==RW_ERROR_BUFFER); CHECK(n==7 && !d.poisoned);
    setup(&d); n=sizeof(rx); CHECK(!rw_command(&d,0x31,NULL,0,rx,&n,100)); CHECK(d.sequence==0x40);
    setup(&d); fragment=2; n=sizeof(rx);
    const uint8_t t0[]={0x25,0,0,2,0x90,0,0xb7};
    memcpy(response,t0,sizeof(t0)); response_len=sizeof(t0);
    CHECK(!rw_receive(&d,rx,&n,100)); CHECK(n==7 && outs==0 && calls==4);
    setup(&d); d.protocol=RW_PROTOCOL_T1; n=sizeof(rx);
    const uint8_t t1[]={0x25,0x40,2,0x90,0,0xf7};
    memcpy(response,t1,sizeof(t1)); response_len=sizeof(t1);
    CHECK(!rw_receive(&d,rx,&n,100)); CHECK(n==6 && outs==0);
    puts("fragmentation, ZLP, no replay and recovery tests passed"); return 0;
}
