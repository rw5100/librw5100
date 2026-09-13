#include "internal.h"
#include "card.h"
#include <libusb.h>
#include <stdlib.h>
#define VID 0x04dd
#define PID 0x9259
static int usb_error(int e) {
    switch(e) {
    case LIBUSB_ERROR_ACCESS:return RW_ERROR_ACCESS;
    case LIBUSB_ERROR_BUSY:return RW_ERROR_BUSY;
    case LIBUSB_ERROR_NO_DEVICE:return RW_ERROR_NO_DEVICE;
    case LIBUSB_ERROR_NO_MEM:return RW_ERROR_MEMORY;
    case LIBUSB_ERROR_NOT_SUPPORTED:return RW_ERROR_UNSUPPORTED;
    default:return RW_ERROR_IO;
    }
}
int rw_enumerate(rw_device_info *out,size_t *count) {
    if(!count || (!out && *count)) return RW_ERROR_ARGUMENT;
    size_t cap=*count,n=0; *count=0;
    libusb_context *ctx=NULL; libusb_device **list=NULL;
    int e=libusb_init(&ctx); if(e<0) return usb_error(e);
    ssize_t total=libusb_get_device_list(ctx,&list);
    if(total<0) { libusb_exit(ctx); return usb_error((int)total); }
    for(ssize_t i=0;i<total;i++) {
        struct libusb_device_descriptor desc;
        if(libusb_get_device_descriptor(list[i],&desc)<0) continue;
        if(desc.idVendor!=VID || desc.idProduct!=PID) continue;
        if(n<cap) out[n]=(rw_device_info){libusb_get_bus_number(list[i]),
            libusb_get_device_address(list[i]),VID,PID};
        n++;
    }
    libusb_free_device_list(list,1); libusb_exit(ctx); *count=n;
    return out && n>cap?RW_ERROR_BUFFER:RW_OK;
}
static int choose_interface(rw_device *d,libusb_device *dev) {
    struct libusb_config_descriptor *cfg=NULL;
    int e=libusb_get_active_config_descriptor(dev,&cfg);
    if(e<0) return usb_error(e);
    int found=0;
    for(unsigned i=0;i<cfg->bNumInterfaces;i++) {
        const struct libusb_interface *it=&cfg->interface[i];
        for(int a=0;a<it->num_altsetting;a++) {
            const struct libusb_interface_descriptor *alt=&it->altsetting[a];
            uint8_t in=0,out=0; uint16_t pin=0,pout=0;
            for(unsigned j=0;j<alt->bNumEndpoints;j++) {
                const struct libusb_endpoint_descriptor *ep=&alt->endpoint[j];
                if((ep->bmAttributes&3)!=LIBUSB_TRANSFER_TYPE_BULK) continue;
                if(ep->bEndpointAddress&LIBUSB_ENDPOINT_IN) {
                    if(in) { in=0; break; }
                    in=ep->bEndpointAddress; pin=ep->wMaxPacketSize;
                } else {
                    if(out) { out=0; break; }
                    out=ep->bEndpointAddress; pout=ep->wMaxPacketSize;
                }
            }
            if(in && out && pin && pin<=512 && pout==64) {
                found++;
                d->interface_number=alt->bInterfaceNumber;
                d->alternate=alt->bAlternateSetting;
                d->ep_in=in; d->ep_out=out; d->packet_in=pin; d->packet_out=pout;
            }
        }
    }
    libusb_free_config_descriptor(cfg);
    return found==1?RW_OK:RW_ERROR_UNSUPPORTED;
}
int rw_open(rw_device **out,const rw_device_info *selector) {
    if(!out) return RW_ERROR_ARGUMENT;
    *out=NULL;
    if(selector && (selector->vendor!=VID || selector->product!=PID)) return RW_ERROR_ARGUMENT;
    rw_device *d=calloc(1,sizeof(*d)); if(!d) return RW_ERROR_MEMORY;
    d->interface_number=-1; d->timeout_ms=5000;
    atomic_flag_clear(&d->busy);
    int e=libusb_init(&d->usb);
    if(e<0) { free(d); return usb_error(e); }
    libusb_device **list=NULL,*selected=NULL;
    ssize_t n=libusb_get_device_list(d->usb,&list);
    if(n<0) { e=usb_error((int)n); goto fail; }
    int matches=0;
    for(ssize_t i=0;i<n;i++) {
        struct libusb_device_descriptor desc;
        if(libusb_get_device_descriptor(list[i],&desc)<0) continue;
        if(desc.idVendor!=VID || desc.idProduct!=PID) continue;
        if(selector && (selector->bus!=libusb_get_bus_number(list[i]) ||
            selector->address!=libusb_get_device_address(list[i]))) continue;
        selected=list[i]; matches++;
    }
    if(matches!=1) { e=matches?RW_ERROR_BUSY:RW_ERROR_NO_DEVICE; goto free_list; }
    e=libusb_open(selected,&d->handle);
    if(e<0) { e=usb_error(e); goto free_list; }
    e=choose_interface(d,selected);
    if(e) goto free_list;
    /* Detach only this matching device's interface, restore on close. */
    if(libusb_kernel_driver_active(d->handle,d->interface_number)==1) {
        int result=libusb_detach_kernel_driver(d->handle,d->interface_number);
        if(result<0) { e=usb_error(result); goto free_list; }
        d->detached=1;
    }
    e=libusb_claim_interface(d->handle,d->interface_number);
    if(e<0) { e=usb_error(e); goto free_list; }
    d->claimed=1;
    if(d->alternate) {
        e=libusb_set_interface_alt_setting(d->handle,d->interface_number,d->alternate);
        if(e<0) { e=usb_error(e); goto free_list; }
    }
    libusb_free_device_list(list,1);
    e=rw_initialize_impl(d,d->timeout_ms);
    if(e) goto fail;
    *out=d; return RW_OK;
free_list:
    libusb_free_device_list(list,1);
fail:
    rw_close(d); return e;
}
void rw_close(rw_device *d) {
    if(!d) return;
    if(d->handle) {
        if(d->interface_number>=0) {
            if(d->claimed) libusb_release_interface(d->handle,d->interface_number);
            if(d->detached) libusb_attach_kernel_driver(d->handle,d->interface_number);
        }
        libusb_close(d->handle);
    }
    if(d->usb) libusb_exit(d->usb);
    free(d);
}
static int enter(rw_device *d,unsigned timeout) {
    if(!d || !timeout) return RW_ERROR_ARGUMENT;
    if(atomic_flag_test_and_set(&d->busy)) return RW_ERROR_BUSY;
    if(d->poisoned) { atomic_flag_clear(&d->busy); return RW_ERROR_STATE; }
    return RW_OK;
}
#define LEAVE(expr) do { int result=(expr); atomic_flag_clear(&d->busy); return result; } while(0)
int rw_status(rw_device *d,int *status,unsigned timeout) {
    if(!status) return RW_ERROR_ARGUMENT;
    int e=enter(d,timeout); if(e) return e;
    LEAVE(rw_status_impl(d,status,timeout));
}
int rw_reset(rw_device *d,int warm,uint8_t *atr,size_t *n,unsigned timeout) {
    if(!atr || !n || (warm!=0 && warm!=1)) return RW_ERROR_ARGUMENT;
    int e=enter(d,timeout); if(e) { *n=0; return e; }
    LEAVE(rw_reset_impl(d,warm,atr,n,timeout));
}
int rw_set_protocol(rw_device *d,int protocol,unsigned timeout) {
    if(protocol!=RW_PROTOCOL_T0 && protocol!=RW_PROTOCOL_T1) return RW_ERROR_ARGUMENT;
    int e=enter(d,timeout); if(e) return e;
    LEAVE(rw_set_protocol_impl(d,protocol,timeout));
}
int rw_transmit(rw_device *d,const uint8_t *tx,size_t tn,uint8_t *rx,size_t *rn,unsigned timeout) {
    if(!tx || !tn || !rx || !rn) return RW_ERROR_ARGUMENT;
    int e=enter(d,timeout); if(e) { *rn=0; return e; }
    LEAVE(rw_transmit_impl(d,tx,tn,rx,rn,timeout));
}
int rw_power_off(rw_device *d,unsigned timeout) {
    int e=enter(d,timeout); if(e) return e;
    LEAVE(rw_power_off_impl(d,timeout));
}
const char *rw_strerror(int e) {
    switch(e) {
    case RW_OK:return "success";
    case RW_ERROR_ARGUMENT:return "invalid argument";
    case RW_ERROR_NO_DEVICE:return "RW5100 not found; connect the reader";
    case RW_ERROR_ACCESS:return "USB access denied; check device permissions/driver";
    case RW_ERROR_BUSY:return "reader busy or selection ambiguous; close other users or select bus:address";
    case RW_ERROR_NO_CARD:return "no card; insert a card";
    case RW_ERROR_TIMEOUT:return "communication timed out; close and reopen the reader";
    case RW_ERROR_DISCONNECTED:return "reader disconnected; reconnect and reopen";
    case RW_ERROR_FRAME:return "invalid reader response; close and reopen the reader";
    case RW_ERROR_UNSUPPORTED:return "unsupported reader descriptor, card protocol or operation";
    case RW_ERROR_BUFFER:return "response buffer too small; command may already have executed";
    case RW_ERROR_MEMORY:return "out of memory";
    case RW_ERROR_STATE:return "session not ready; reset card or close/reopen reader after transport failure";
    case RW_ERROR_CARD:return "card protocol error";
    default:return "USB I/O error; close and reopen the reader";
    }
}
