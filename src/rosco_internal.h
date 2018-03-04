#ifndef LIBMEMS_INTERNAL_H
#define LIBMEMS_INTERNAL_H

#include <stdbool.h>

#include "ftd2xx.h"

bool mems_openserial(mems_info *info);
bool mems_send_command(mems_info *info, uint8_t cmd);
int16_t mems_read_serial(mems_info* info, uint8_t *buffer, uint16_t quantity);
int16_t mems_write_serial(mems_info* info, uint8_t *buffer, uint16_t quantity);
bool mems_lock(mems_info* info);
void mems_unlock(mems_info* info);
uint8_t temperature_value_to_degrees_f(uint8_t val);

uint32_t get_current_us();
uint32_t wait_until_us(uint32_t us);
void wait_for_us(uint32_t us);

#endif // LIBMEMS_INTERNAL_H

