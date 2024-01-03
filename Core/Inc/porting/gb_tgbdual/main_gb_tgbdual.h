#pragma once

void app_main_gb_tgbdual(uint8_t load_state, uint8_t start_paused, uint8_t save_slot);
#if CHEAT_CODES == 1
void update_cheats_gb();
#endif

// Function called by PA14 (GB serial link) IRQ
uint8_t handle_incoming_serial_data(uint8_t data);
