#ifndef __APP_BATTERY_H__
#define __APP_BATTERY_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int app_battery_init(void);
int app_battery_voltage_mv();
bool app_battery_charging();

#ifdef __cplusplus
}
#endif

#endif /*__APP_BATTERY_H__*/
