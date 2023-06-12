
#ifndef __APP_OT_H__
#define __APP_OT_H__

#include <stdint.h>
#include <openthread/thread.h>

#ifdef __cplusplus
extern "C" {
#endif

int app_ot_init(void);
otDeviceRole ot_app_role();
void quick_reboot_factoryreset();

#ifdef __cplusplus
}
#endif

#endif /*__APP_OT_H__*/
