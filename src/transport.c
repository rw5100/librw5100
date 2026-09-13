#include "internal.h"
#include "frame.h"
#include <libusb.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <errno.h>
#endif
uint64_t rw_now_ms(void) {
#ifdef _WIN32
    return GetTickCount64();
#else
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC,&t);
    return (uint64_t)t.tv_sec*1000+(uint64_t)t.tv_nsec/1000000;
#endif
}
void rw_sleep_ms(unsigned ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    struct timespec t={(time_t)(ms/1000),(long)(ms%1000)*1000000};
    while(nanosleep(&t,&t)<0 && errno==EINTR) {}
#endif
}
static int map_error(int e) {
    switch(e) {
    case LIBUSB_ERROR_TIMEOUT:return RW_ERROR_TIMEOUT;
    case LIBUSB_ERROR_NO_DEVICE:return RW_ERROR_DISCONNECTED;
    case LIBUSB_ERROR_ACCESS:return RW_ERROR_ACCESS;
    case LIBUSB_ERROR_BUSY:return RW_ERROR_BUSY;
    case LIBUSB_ERROR_OVERFLOW:return RW_ERROR_FRAME;
    default:return RW_ERROR_IO;
    }
}
static unsigned remaining(uint64_t deadline) {
    uint64_t now=rw_now_ms();
    return now>=deadline?0:(unsigned)(deadline-now);
}
static void trace(const char *direction,const uint8_t *data,size_t n,int error) {
    const char *enabled=getenv("RW5100_TRACE");
    if(!enabled || strcmp(enabled,"1")) return;
    fprintf(stderr,"device %llu %s bytes=%zu usb_error=%d",(unsigned long long)rw_now_ms(),direction,n,error);
    for(size_t i=0;i<n;i++) fprintf(stderr," %02X",data[i]);
    fputc('\n',stderr);
}
/* All transfers share one deadline; partial OUT is never replayed. */
int rw_exchange(rw_device *d,const uint8_t *tx,size_t txlen,
                uint8_t *rx,size_t *rxlen,unsigned timeout) {
    if(!d || !tx || !txlen || txlen>0x10400 || !rx || !rxlen || !timeout)
        return RW_ERROR_ARGUMENT;
    size_t cap=*rxlen; *rxlen=0;
    if(d->poisoned) return RW_ERROR_STATE;
    uint64_t deadline=rw_now_ms()+timeout;
    trace("OUT",tx,txlen,0);
    int actual=0, e=libusb_bulk_transfer(d->handle,d->ep_out,(unsigned char *)tx,
                                        (int)txlen,&actual,timeout);
    if(e<0 || actual!=(int)txlen) {
        trace("OUT-result",tx,actual>0?(size_t)actual:0,e);
        d->poisoned=1; return e<0?map_error(e):RW_ERROR_IO;
    }
    /* device 0x122f9: a frame divisible by 64 requires a terminating ZLP. */
    if(txlen%64==0) {
        unsigned left=remaining(deadline);
        if(!left) { d->poisoned=1; return RW_ERROR_TIMEOUT; }
        e=libusb_bulk_transfer(d->handle,d->ep_out,(unsigned char *)tx,0,&actual,left);
        trace("OUT-ZLP",tx,0,e);
        if(e<0) { d->poisoned=1; return map_error(e); }
    }
    unsigned left=remaining(deadline);
    if(!left) { d->poisoned=1; return RW_ERROR_TIMEOUT; }
    *rxlen=cap;
    return rw_receive(d,rx,rxlen,left);
}
int rw_receive(rw_device *d,uint8_t *rx,size_t *rxlen,unsigned timeout) {
    if(!d || !rx || !rxlen || !timeout || !d->packet_in || d->packet_in>512)
        return RW_ERROR_ARGUMENT;
    size_t cap=*rxlen; *rxlen=0;
    if(d->poisoned) return RW_ERROR_STATE;
    uint64_t deadline=rw_now_ms()+timeout;
    int e,actual=0;
    uint8_t frame[65540], packet[512];
    size_t used=0, expected=0;
    for(;;) {
        unsigned left=remaining(deadline);
        if(!left) { d->poisoned=1; return RW_ERROR_TIMEOUT; }
        /* Packet-sized buffers avoid truncating the last USB packet. */
        e=libusb_bulk_transfer(d->handle,d->ep_in,packet,d->packet_in,&actual,left);
        trace("IN",packet,actual>0?(size_t)actual:0,e);
        if(e<0) { d->poisoned=1; return map_error(e); }
        if(actual<0 || used+(size_t)actual>sizeof(frame)) {
            d->poisoned=1; return RW_ERROR_FRAME;
        }
        memcpy(frame+used,packet,(size_t)actual); used+=(size_t)actual;
        if(used>=4 && d->protocol==RW_PROTOCOL_T0 && frame[0]!=0x21)
            expected=((size_t)frame[2]<<8)+frame[3]+5;
        else if(used>=3 && !(d->protocol==RW_PROTOCOL_T0 && frame[0]!=0x21))
            expected=(size_t)frame[2]+4;
        if(expected && used>=expected) break;
        if(!actual) rw_sleep_ms(1);
    }
    if(used!=expected || rw_lrc(frame,used)) { d->poisoned=1; return RW_ERROR_FRAME; }
    if(cap<used) { *rxlen=used; return RW_ERROR_BUFFER; }
    memcpy(rx,frame,used); *rxlen=used;
    return RW_OK;
}
int rw_command(rw_device *d,uint8_t opcode,const uint8_t *args,size_t nargs,
               uint8_t *rx,size_t *rxlen,unsigned timeout) {
    uint8_t tx[259]; size_t n=sizeof(tx);
    int e=rw_frame_encode(d->sequence,opcode,args,nargs,tx,&n);
    if(e) return e;
    e=rw_exchange(d,tx,n,rx,rxlen,timeout);
    if(!e || e==RW_ERROR_BUFFER) d->sequence^=0x40;
    return e;
}
