#include "rw5100.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

static int json;
static unsigned timeout=5000;
static int report(const char *operation,int error,const uint8_t *data,size_t n,int state) {
    if(json) {
        printf("{\"operation\":\"%s\",\"error\":%d,\"message\":\"%s\"",operation,error,rw_strerror(error));
        if(!error && data) {
            printf(",\"hex\":\"");
            for(size_t i=0;i<n;i++) printf("%02X",data[i]);
            printf("\"");
        }
        if(!error && state>=0) printf(",\"status\":%d",state);
        printf("}\n");
    } else if(error) fprintf(stderr,"%s: %d: %s\n",operation,error,rw_strerror(error));
    else if(data) {
        printf("%s:",operation);
        for(size_t i=0;i<n;i++) printf(" %02X",data[i]);
        puts("");
    } else if(state>=0) printf("%s: %s\n",operation,state==RW_CARD_ABSENT?"absent":state==RW_CARD_POWERED?"powered":"present");
    else printf("%s: success\n",operation);
    return error?1:0;
}
static int number(const char *s,unsigned *n) {
    char *end; errno=0;
    unsigned long v=strtoul(s,&end,10);
    if(errno || !*s || *end || v>3600000 || *s=='-') return 0;
    *n=(unsigned)v; return 1;
}
static int hex(const char *s,uint8_t *out,size_t *n) {
    size_t used=0; int high=-1;
    for(;*s;s++) {
        if(isspace((unsigned char)*s)) continue;
        int c=toupper((unsigned char)*s),v;
        if(c>='0' && c<='9') v=c-'0';
        else if(c>='A' && c<='F') v=c-'A'+10;
        else return 0;
        if(high<0) high=v;
        else { if(used==*n) return 0; out[used++]=(uint8_t)(high*16+v); high=-1; }
    }
    if(high>=0 || !used) return 0;
    *n=used; return 1;
}
static int run(rw_device *d,char *line) {
    char *cmd=line; while(isspace((unsigned char)*cmd)) cmd++;
    if(!*cmd || *cmd=='#') return 0;
    char *arg=cmd;
    while(*arg && !isspace((unsigned char)*arg)) arg++;
    if(*arg) *arg++=0;
    while(isspace((unsigned char)*arg)) arg++;
    char *end=arg+strlen(arg);
    while(end>arg && isspace((unsigned char)end[-1])) *--end=0;
    uint8_t out[65538]; size_t n=sizeof(out); int e;
    if(!strcmp(cmd,"status") && !*arg) {
        int state=-1; e=rw_status(d,&state,timeout);
        return report("status",e,NULL,0,state);
    }
    if((!strcmp(cmd,"atr") || !strcmp(cmd,"reset")) && (!*arg || !strcmp(arg,"warm"))) {
        e=rw_reset(d,!strcmp(arg,"warm"),out,&n,timeout);
        return report("atr",e,out,n,-1);
    }
    if(!strcmp(cmd,"protocol") && (!strcmp(arg,"0") || !strcmp(arg,"1"))) {
        e=rw_set_protocol(d,*arg-'0',timeout); return report("protocol",e,NULL,0,-1);
    }
    if(!strcmp(cmd,"transmit")) {
        uint8_t in[65544]; size_t len=sizeof(in);
        if(!hex(arg,in,&len)) return report("transmit",RW_ERROR_ARGUMENT,NULL,0,-1);
        e=rw_transmit(d,in,len,out,&n,timeout); return report("transmit",e,out,n,-1);
    }
    if(!strcmp(cmd,"off") && !*arg) return report("off",rw_power_off(d,timeout),NULL,0,-1);
    return report("command",RW_ERROR_ARGUMENT,NULL,0,-1);
}
static void usage(void) {
    puts("rw5100ctl [--json] [--timeout MS] [--device BUS:ADDRESS] COMMAND\n"
         "  list | status | atr [warm] | protocol 0|1 | transmit HEX | off\n"
         "  session FILE (or - for stdin): one command per line; stop at first error\n"
         "Single transmit resets the card and selects its default protocol first.\n"
         "Use a session to preserve card selection/authentication across APDUs.");
}
int main(int argc,char **argv) {
    rw_device_info selector={0,0,0x04dd,0x9259}; int selected=0,i=1;
    for(;i<argc && !strncmp(argv[i],"--",2);i++) {
        if(!strcmp(argv[i],"--help")) { usage(); return 0; }
        if(!strcmp(argv[i],"--json")) { json=1; continue; }
        if(!strcmp(argv[i],"--timeout") && i+1<argc && number(argv[i+1],&timeout) && timeout) { i++; continue; }
        if(!strcmp(argv[i],"--device") && i+1<argc) {
            unsigned b,a; char extra;
            if(sscanf(argv[++i],"%u:%u%c",&b,&a,&extra)==2 && b<=255 && a<=255) {
                selector.bus=(uint8_t)b; selector.address=(uint8_t)a; selected=1; continue;
            }
        }
        return report("arguments",RW_ERROR_ARGUMENT,NULL,0,-1);
    }
    if(i>=argc) { usage(); return 2; }
    if(!strcmp(argv[i],"list")) {
        if(i+1!=argc) return report("arguments",RW_ERROR_ARGUMENT,NULL,0,-1);
        size_t n=0; int e=rw_enumerate(NULL,&n);
        if(e) return report("list",e,NULL,0,-1);
        rw_device_info *list=calloc(n?n:1,sizeof(*list));
        if(!list) return report("list",RW_ERROR_MEMORY,NULL,0,-1);
        size_t cap=n; e=rw_enumerate(list,&cap);
        if(e) { free(list); return report("list",e,NULL,0,-1); }
        if(json) printf("{\"operation\":\"list\",\"error\":0,\"devices\":[");
        for(size_t j=0;j<cap;j++) {
            if(json) printf("%s{\"bus\":%u,\"address\":%u,\"vid\":1245,\"pid\":37465}",j?",":"",list[j].bus,list[j].address);
            else printf("%u:%u 04dd:9259 RW5100\n",list[j].bus,list[j].address);
        }
        if(json) puts("]}"); else if(!cap) puts("No RW5100 connected.");
        free(list); return 0;
    }
    int session=!strcmp(argv[i],"session");
    if(session && argc-i!=2) return report("arguments",RW_ERROR_ARGUMENT,NULL,0,-1);
    rw_device *d=NULL; int e=rw_open(&d,selected?&selector:NULL);
    if(e) return report("open",e,NULL,0,-1);
    int result=0;
    if(session) {
        FILE *f=!strcmp(argv[i+1],"-")?stdin:fopen(argv[i+1],"r");
        if(!f) result=report("session file",RW_ERROR_IO,NULL,0,-1);
        else {
            char line[131200];
            while(fgets(line,sizeof(line),f)) {
                if(!strchr(line,'\n') && !feof(f)) { result=report("line too long",RW_ERROR_ARGUMENT,NULL,0,-1); break; }
                if((result=run(d,line))) break;
            }
            if(ferror(f)) result=report("session read",RW_ERROR_IO,NULL,0,-1);
            if(f!=stdin) fclose(f);
        }
    } else {
        char line[131200]; size_t used=0;
        if(!strcmp(argv[i],"transmit")) {
            uint8_t atr[64]; size_t n=sizeof(atr);
            e=rw_reset(d,0,atr,&n,timeout);
            if(e) { result=report("reset",e,NULL,0,-1); goto done; }
        }
        for(;i<argc;i++) {
            size_t n=strlen(argv[i]);
            if(used+n+2>sizeof(line)) { result=report("arguments",RW_ERROR_ARGUMENT,NULL,0,-1); goto done; }
            if(used) line[used++]=' ';
            memcpy(line+used,argv[i],n); used+=n;
        }
        line[used]=0; result=run(d,line);
    }
done:
    rw_close(d); return result;
}
