#pragma once
#include "gello_action.h"
#include <stddef.h>
#define GA_SLOTS 12u
#define GA_NAME_SIZE 49u
#define GA_BANK_SIZE (12u+2u*GA_SLOTS*(GA_NAME_SIZE+1u+GA_ACTION_SIZE)+4u)
struct ga_slot { bool used;char name[GA_NAME_SIZE];struct ga_action action; };
struct ga_bank {uint32_t revision;uint8_t selected[2];struct ga_slot slots[2][GA_SLOTS];};
void ga_bank_init(struct ga_bank *b);
bool ga_name_valid(const char *name);
int ga_bank_slot(const struct ga_bank *b,unsigned side,const char *name);
bool ga_bank_delete(struct ga_bank *b,unsigned side,unsigned slot);
bool ga_bank_valid(const struct ga_bank *b);
void ga_bank_encode(const struct ga_bank *b,uint8_t out[GA_BANK_SIZE]);
bool ga_bank_decode(const uint8_t *in,size_t n,struct ga_bank *b);
