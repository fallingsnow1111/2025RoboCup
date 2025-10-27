#ifndef __LOAD2_H__
#define __LOAD2_H__

#include "stm32f7xx.h"

void load2_init(void);
void load2_main(void);
void load2_find_zero(void);
void load2_next_flag(void);
void load2_last_flag(void);
void load2_next_nflag(uint8_t cnt);
void load2_last_nflag(uint8_t cnt);
void load2_to_place(uint8_t place);
void load2_scores_in(uint8_t id);
uint8_t load2_scores_getcnt(void);
void load2_scores_outid(uint8_t id);
void load2_scores_out(uint8_t id);
void load2_enable_flag(uint8_t sta);
uint8_t load2_check_flag(void);
void load_find_error(void);
void load_check2(void);
#endif
