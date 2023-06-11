
#ifndef __APP_BUTTON_H__
#define __APP_BUTTON_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


int app_button_init(void);

typedef void (*button_callback_handler_t)(void);

void app_button_set_short_callback(button_callback_handler_t cb);
void app_button_set_long_callback(button_callback_handler_t cb);
void app_button_set_short_timeout(int64_t timeout_ms);
void app_button_set_long_timeout(int64_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /*__APP_BUTTON_H__*/
