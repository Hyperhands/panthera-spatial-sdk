#include "gello_action_bank.h"
#include "gello_radio_codec.h"
#include <string.h>
static void w32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=v>>(8*i);}
static uint32_t r32(const uint8_t *p){return p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
bool ga_name_valid(const char *s) {
    size_t n=0;while(n<GA_NAME_SIZE && s[n])n++;
    if(!n || n>=GA_NAME_SIZE || s[0]==' ' || s[n-1]==' ')return false;
    for(size_t i=0;i<n;) {
        unsigned c=(uint8_t)s[i++],more=0,min=0;
        if(c<0x80){if(c<0x20 || c==0x7f)return false;continue;}
        if(c>=0xc2 && c<=0xdf){c&=31;more=1;min=0x80;}
        else if(c>=0xe0 && c<=0xef){c&=15;more=2;min=0x800;}
        else if(c>=0xf0 && c<=0xf4){c&=7;more=3;min=0x10000;}
        else return false;
        if(i+more>n)return false;
        for(unsigned j=0;j<more;j++){unsigned v=(uint8_t)s[i++];if((v&0xc0)!=0x80)return false;c=(c<<6)|(v&63);}
        if(c<min || c>0x10ffff || (c>=0xd800 && c<=0xdfff))return false;
    }
    return true;
}
void ga_bank_init(struct ga_bank *b) {
    memset(b,0,sizeof(*b));b->revision=1;
    const char *names[]={"当前遥操","旧版抓握","旧版双指捏","旧版四指捏","握拳"};
    for(unsigned side=0;side<2;side++)for(unsigned i=0;i<5;i++) {
        struct ga_slot *s=&b->slots[side][i];s->used=true;strcpy(s->name,names[i]);ga_builtin(i,&s->action);
        if(!i) {
            const uint16_t legacy[6]={1000,1000,611,678,667,633};
            ga_from_legacy(legacy,2000,&s->action);
        }
    }
}
int ga_bank_slot(const struct ga_bank *b,unsigned side,const char *name) {
    if(side>1 || !ga_name_valid(name))return -1;
    int empty=-1;
    for(unsigned i=0;i<GA_SLOTS;i++) {
        if(b->slots[side][i].used && !strcmp(name,b->slots[side][i].name))return i;
        if(!b->slots[side][i].used && empty<0)empty=i;
    }
    return empty;
}
bool ga_bank_delete(struct ga_bank *b,unsigned side,unsigned slot) {
    if(side>1 || slot>=GA_SLOTS || !b->slots[side][slot].used)return false;
    memset(&b->slots[side][slot],0,sizeof(b->slots[side][slot]));
    if(!slot) {
        const uint16_t legacy[6]={1000,1000,611,678,667,633};
        b->slots[side][0].used=true;strcpy(b->slots[side][0].name,"当前遥操");
        ga_from_legacy(legacy,2000,&b->slots[side][0].action);
    }
    if(b->selected[side]==slot)b->selected[side]=0;
    if(!++b->revision)b->revision=1;
    return true;
}
bool ga_bank_valid(const struct ga_bank *b) {
    if(!b->revision)return false;
    for(unsigned side=0;side<2;side++) {
        if(b->selected[side]>=GA_SLOTS || !b->slots[side][b->selected[side]].used)return false;
        for(unsigned i=0;i<GA_SLOTS;i++)if(b->slots[side][i].used) {
            if(!ga_name_valid(b->slots[side][i].name) || !ga_valid(&b->slots[side][i].action))return false;
            for(unsigned j=0;j<i;j++)if(b->slots[side][j].used && !strcmp(b->slots[side][j].name,b->slots[side][i].name))return false;
        }
    }
    return true;
}
void ga_bank_encode(const struct ga_bank *b,uint8_t out[GA_BANK_SIZE]) {
    memset(out,0,GA_BANK_SIZE);memcpy(out,"GA01",4);w32(out+4,b->revision);out[8]=b->selected[0];out[9]=b->selected[1];
    size_t at=12;
    for(unsigned side=0;side<2;side++)for(unsigned i=0;i<GA_SLOTS;i++) {
        const struct ga_slot *s=&b->slots[side][i];out[at++]=s->used;
        if(s->used){memcpy(out+at,s->name,strlen(s->name));ga_encode(&s->action,out+at+GA_NAME_SIZE);}
        at+=GA_NAME_SIZE+GA_ACTION_SIZE;
    }
    w32(out+GA_BANK_SIZE-4,gr_crc(out,GA_BANK_SIZE-4));
}
bool ga_bank_decode(const uint8_t *in,size_t n,struct ga_bank *b) {
    if(n!=GA_BANK_SIZE || memcmp(in,"GA01",4) || in[10] || in[11] || r32(in+n-4)!=gr_crc(in,n-4))return false;
    memset(b,0,sizeof(*b));b->revision=r32(in+4);b->selected[0]=in[8];b->selected[1]=in[9];size_t at=12;
    for(unsigned side=0;side<2;side++)for(unsigned i=0;i<GA_SLOTS;i++) {
        struct ga_slot *s=&b->slots[side][i];if(in[at]>1)return false;s->used=in[at++];
        if(s->used) {
            memcpy(s->name,in+at,GA_NAME_SIZE);
            if(s->name[GA_NAME_SIZE-1] || !ga_decode(in+at+GA_NAME_SIZE,&s->action))return false;
        }
        at+=GA_NAME_SIZE+GA_ACTION_SIZE;
    }
    return ga_bank_valid(b);
}
