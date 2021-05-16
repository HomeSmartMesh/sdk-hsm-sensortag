## usage
```bash
west build -b nrf52840_sensortag -t guiconfig
west build -b nrf52840_sensortag -- -DCONF_FILE=prj.conf
west build -b nrf52840_sensortag -- -DCONF_FILE=prj-lowpower.conf
west flash
```

device_set_power_state has no influence on GPIO as PGIO has no enable and no influence on ADC as ADC is anyway always disabled between usage

## Documentation
https://www.homesmartmesh.com/docs/microcontrollers/nrf52/thread_sensortag/#tag_power

# Lowe power perf
* normal sleep : ~ 3.5 mA
* sleep with releasing HF clock : ~ 1.8 mA
## Zephyr patch
content of `power.c`

```c
/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <power/power.h>
#include <hal/nrf_power.h>
#include <hal/nrf_clock.h>

#include <logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

void sleep_prepare()
{
	nrf_clock_task_trigger(NRF_CLOCK,NRF_CLOCK_TASK_HFCLKSTOP);
}

void sleep_wakeup()
{
    nrf_clock_event_clear(NRF_CLOCK,NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_clock_task_trigger(NRF_CLOCK,NRF_CLOCK_TASK_HFCLKSTART);
	while(!nrf_clock_hf_is_running(NRF_CLOCK,NRF_CLOCK_HFCLK_HIGH_ACCURACY));
}

/* Invoke Low Power/System Off specific Tasks */
void pm_power_state_set(struct pm_state_info info)
{
	switch (info.state) {
	case PM_STATE_SOFT_OFF:
		nrf_power_system_off(NRF_POWER);
		break;
	case PM_STATE_RUNTIME_IDLE:
		sleep_prepare();
		break;
	case PM_STATE_STANDBY:
		sleep_prepare();
		break;
	default:
		LOG_DBG("Unsupported power state %u", info.state);
		break;
	}
}

/* Handle SOC specific activity after Low Power Mode Exit */
void pm_power_state_exit_post_ops(struct pm_state_info info)
{
	switch (info.state) {
	case PM_STATE_SOFT_OFF:
		/* Nothing to do. */
		break;
	case PM_STATE_RUNTIME_IDLE:
		sleep_wakeup();
		break;
	case PM_STATE_STANDBY:
		sleep_wakeup();
		break;
	default:
		LOG_DBG("Unsupported power state %u", info.state);
		break;
	}

	/*
	 * System is now in active mode. Reenable interrupts which were disabled
	 * when OS started idling code.
	 */
	irq_unlock(0);
}
```