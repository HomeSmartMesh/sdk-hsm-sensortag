## usage
```bash
west build -t guiconfig
west build
west build -- -DCONF_FILE=prj-fixed.conf
west build -- -DOVERLAY_CONFIG="overlay-log.conf"
west build -- -DCONF_FILE=prj-fixed.conf -DOVERLAY_CONFIG=overlay-log.conf
west flash
nrfjprog -f nrf52 --eraseall
```

then allow it to join on the boarder router
```shell
sudo ot-ctl
commissioner start
commissioner joiner add * ABCDE2 3600
```

for a CLI dongle, a dataset is needed, see https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/protocols/thread/overview/commissioning.html#setting-up-the-commissioner


## Documentation
https://www.homesmartmesh.com/docs/microcontrollers/nrf52/thread_sensortag/#tag_sensors_broadcast

## watchdog usage
```conf
CONFIG_WATCHDOG=y
CONFIG_WDT_DISABLE_AT_BOOT=n
```

```C
#include <zephyr/drivers/watchdog.h>

#define WDT_MAX_WINDOW_MS  60000U
#define WDT_MIN_WINDOW_MS  0U
int wdt_channel_id;

void start_watchdog(const struct device *const wdt){
	struct wdt_timeout_cfg wdt_config = {
		.flags = WDT_FLAG_RESET_SOC,
		.window.min = WDT_MIN_WINDOW_MS,
		.window.max = WDT_MAX_WINDOW_MS,
	};

	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	if (wdt_channel_id < 0) {
		printk("Watchdog install error\n");
		return;
	}
	wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
}

void main(void)
	const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
    start_watchdog(wdt);

    while(1){
        wdt_feed(wdt, wdt_channel_id);
    }

```
