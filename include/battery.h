
#ifndef __BATTERY_H__
#define __BATTERY_H__


#include <stdint.h>

void battery_init();

void battery_start();

int32_t battery_get_mv();

#endif /*__BATTERY_H__*/
