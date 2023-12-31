#pragma once

// FIXME Not needed in header file?
#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

void app_main_gb_tgbdual(uint8_t load_state, uint8_t start_paused, uint8_t save_slot);
#if CHEAT_CODES == 1
void update_cheats_gb();
#endif

// Functions called by PA14 (GB serial link) IRQ
EXTERNC uint8_t get_current_sb();
EXTERNC void set_sb_and_raise_interrupt(uint8_t sb);


//#undef EXTERNC
