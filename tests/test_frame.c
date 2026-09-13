#include "frame.h"
#include "rw5100.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); return 1; } } while(0)
int main(void) {
    /* Independently calculated control command vectors from device constructors. */
    const uint8_t status[]={0x12,0,2,1,0x31,0x20};
    const uint8_t firmware[]={0x12,0x40,2,1,0xc0,0x91};
    const uint8_t reset[]={0x12,0,3,1,0xd1,2,0xc3};
    uint8_t out[259],arg=2; size_t n=sizeof(out);
    CHECK(!rw_frame_encode(0,0x31,NULL,0,out,&n));
    CHECK(n==sizeof(status) && !memcmp(out,status,n));
    n=sizeof(out); CHECK(!rw_frame_encode(0x40,0xc0,NULL,0,out,&n));
    CHECK(n==sizeof(firmware) && !memcmp(out,firmware,n));
    n=sizeof(out); CHECK(!rw_frame_encode(0,0xd1,&arg,1,out,&n));
    CHECK(n==sizeof(reset) && !memcmp(out,reset,n));
    CHECK(!rw_frame_validate(out,n));
    CHECK(rw_frame_validate(out,n-1)==RW_ERROR_FRAME);
    out[n-1]^=1; CHECK(rw_frame_validate(out,n)==RW_ERROR_FRAME);
    uint8_t args[254]={0}; n=sizeof(out);
    CHECK(!rw_frame_encode(0,0x35,args,253,out,&n)); CHECK(n==259);
    CHECK(!rw_frame_validate(out,n));
    CHECK(rw_frame_encode(0,0x35,args,254,out,&n)==RW_ERROR_ARGUMENT);
    n=1; CHECK(rw_frame_encode(0,0x31,NULL,0,out,&n)==RW_ERROR_BUFFER); CHECK(n==6);
    CHECK(rw_frame_validate(NULL,0)==RW_ERROR_FRAME);
    puts("frame vectors and bounds passed"); return 0;
}
