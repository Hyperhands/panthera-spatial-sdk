#include "gello/gello_action_bank.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static void near(float a,float b){assert(fabsf(a-b)<.0002f);}
int main(void){
 struct ga_action a;float q[6],ma;
 ga_builtin(2,&a);assert(ga_valid(&a));
 ga_sample(&a,0,q,&ma);near(q[0],0);near(q[1],0);near(q[2],0);for(unsigned j=3;j<6;j++)near(q[j],0);
 ga_sample(&a,20,q,&ma);near(q[0],75);near(q[1],0);near(q[2],0);
 ga_sample(&a,95,q,&ma);near(q[1],18);near(q[2],52);
 ga_sample(&a,100,q,&ma);near(q[1],20);near(q[2],55);
 for(unsigned k=0;k<=1000;k++){ga_sample(&a,k/10.f,q,&ma);for(unsigned j=3;j<6;j++)near(q[j],0);}
 ga_builtin(3,&a);ga_sample(&a,50,q,&ma);near(q[0],75);near(q[1],20);near(q[2],27.5);near(q[3],30.5);near(q[4],30);near(q[5],28.5);
 ga_builtin(1,&a);ga_sample(&a,95,q,&ma);near(q[0],75);near(q[1],13.f*20/18);near(q[2],53.f*55/58);near(q[3],56);
 a.curve=GA_LINEAR;a.current_ma=1750;ga_sample(&a,50,q,&ma);near(q[0],37.5);near(ma,1125);
 ga_builtin(4,&a);ga_sample(&a,100,q,&ma);for(unsigned j=2;j<6;j++)near(q[j],90);
 struct ga_bank b,decoded;ga_bank_init(&b);assert(ga_bank_valid(&b));
 assert(ga_bank_slot(&b,0,"我的捏合")==5);assert(ga_bank_slot(&b,0,"握拳")==4);
 b.slots[0][5]=(struct ga_slot){.used=true,.name="我的捏合",.action=a};b.selected[0]=5;b.revision++;
 uint8_t bytes[GA_BANK_SIZE];ga_bank_encode(&b,bytes);assert(ga_bank_decode(bytes,sizeof(bytes),&decoded));
 assert(decoded.selected[0]==5 && !strcmp(decoded.slots[0][5].name,"我的捏合") && ga_crc(&decoded.slots[0][5].action)==ga_crc(&a));
 for(unsigned i=0;i<GA_BANK_SIZE;i++){bytes[i]^=1;assert(!ga_bank_decode(bytes,sizeof(bytes),&decoded));bytes[i]^=1;}
 assert(!ga_name_valid("") && !ga_name_valid(" x") && !ga_name_valid("x ") && !ga_name_valid("a\nb") && !ga_name_valid("\xc0\x80") && !ga_name_valid("\xed\xa0\x80"));
 assert(ga_name_valid("<img src=x onerror=alert(1)>") && ga_name_valid("四指捏 🤏"));
 strcpy(b.slots[0][5].name,"握拳");assert(!ga_bank_valid(&b));
 ga_bank_init(&b);for(unsigned i=5;i<GA_SLOTS;i++){b.slots[0][i].used=true;snprintf(b.slots[0][i].name,GA_NAME_SIZE,"action%u",i);b.slots[0][i].action=a;}
 assert(ga_bank_slot(&b,0,"new")==-1);assert(ga_bank_slot(&b,0,"action7")==7);
 /* Delete from a full bank, round-trip storage, and reuse only the freed slot. */
 struct ga_bank before=b;assert(ga_bank_delete(&b,0,7));assert(ga_bank_valid(&b));
 assert(!b.slots[0][7].used && b.selected[0]==before.selected[0] && b.revision==before.revision+1);
 assert(!memcmp(b.slots[1],before.slots[1],sizeof(b.slots[1])));
 ga_bank_encode(&b,bytes);assert(ga_bank_decode(bytes,sizeof(bytes),&decoded));
 assert(!decoded.slots[0][7].used && ga_bank_slot(&decoded,0,"new")==7);
 before=b;assert(!ga_bank_delete(&b,2,1) && !ga_bank_delete(&b,0,GA_SLOTS) && !ga_bank_delete(&b,0,7));
 assert(!memcmp(&before,&b,sizeof(b)));
 /* Deleting the active action selects the original default without changing it. */
 b.selected[0]=6;struct ga_action fallback=b.slots[0][0].action;
 assert(ga_bank_delete(&b,0,6) && b.selected[0]==0 && ga_bank_valid(&b));
 assert(ga_crc(&fallback)==ga_crc(&b.slots[0][0].action));
 ga_bank_encode(&b,bytes);assert(ga_bank_decode(bytes,sizeof(bytes),&decoded));
 assert(decoded.selected[0]==0 && !decoded.slots[0][6].used);
 /* An edited slot zero can be reset; there is always a valid startup profile. */
 b.slots[0][0].action.current_ma=600;b.revision=UINT32_MAX;
 assert(ga_bank_delete(&b,0,0) && ga_bank_valid(&b) && b.revision==1);
 assert(ga_crc(&fallback)==ga_crc(&b.slots[0][0].action));
 puts("Actions: historical pinch, full range, current, UTF-8 names, durable bank CRC, capacity and persistent deletion passed");
}
