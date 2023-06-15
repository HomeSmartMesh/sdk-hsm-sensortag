## usage
low power tested with `prj-simple.conf` k_sleep @3uA

```bash
west build -b nrf52840_sensortag -t guiconfig
west build -b nrf52840_sensortag
west build -b nrf52840_sensortag -- -DCONF_FILE=prj.conf

west build -b nrf52840_sensortag -- -DCONF_FILE=prj-low.conf
west flash
```

device_set_power_state has no influence on GPIO as PGIO has no enable and no influence on ADC as ADC is anyway always disabled between usage

## Documentation
https://www.homesmartmesh.com/docs/microcontrollers/nrf52/thread_sensortag/#tag_power

# Nordic ticket
https://devzone.nordicsemi.com/f/nordic-q-a/75774/nrf52840-zephyr-tickless-low-power-idle-always-ticking

## call tree
k_sleep (kernel.h) -> z_impl_k_sleep(sched.c) -> 
K_FOREVER : -> k_thread_suspend -> z_impl_k_thread_suspend -> z_reschedule_unlocked -> z_reschedule_irqlock
100 ms    : -> z_tick_sleep

# Lowe power perf
* normal sleep : ~ 3.5 mA
* sleep with releasing HF clock : ~ 1.8 mA

## low power sleep attempt

```c
//the fix is just not to use this function
void lp_sleep_ms(int sleep_ms)
{
	sys_clock_set_timeout(k_ms_to_ticks_ceil32(sleep_ms),false);//sleep @2.9 uA
	//----------sleep prepare--------------
	nrf_clock_task_trigger(NRF_CLOCK,NRF_CLOCK_TASK_HFCLKSTOP);
	nrf_power_task_trigger(NRF_POWER,NRF_POWER_TASK_LOWPWR);
	__WFE();
	__SEV();// Clear the internal event register.
	__WFE();
	//----------sleep wakeup--------------
    nrf_clock_event_clear(NRF_CLOCK,NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_clock_task_trigger(NRF_CLOCK,NRF_CLOCK_TASK_HFCLKSTART);
	while(!nrf_clock_hf_is_running(NRF_CLOCK,NRF_CLOCK_HFCLK_HIGH_ACCURACY));
}

#if STOP_TEST_OFF
	pm_power_state_force((struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});//PM_STATE_SOFT_OFF
#endif

```

## main policy
```c
#ifdef CONFIG_PM
static const struct pm_state_info pm_min_residency[] ={
	{PM_STATE_RUNTIME_IDLE		,1 ,1000 , 0},		//  1 ms
	{PM_STATE_STANDBY			,2 ,20000 , 0},	// 20 ms
	{PM_STATE_SUSPEND_TO_RAM	,3 ,100000 , 0},	// 100 ms
	{PM_STATE_SUSPEND_TO_DISK	,4 ,200000 , 0}	// 200 ms
};

struct pm_state_info pm_policy_next_state(int32_t ticks)
{
	int i;
	static struct pm_state_info current = {PM_STATE_ACTIVE,0, 1000};

	for (i = ARRAY_SIZE(pm_min_residency) - 1; i >= 0; i--) {
		if (!pm_constraint_get(pm_min_residency[i].state)) {
			continue;
		}

		if ((ticks == K_TICKS_FOREVER) || (ticks >= k_us_to_ticks_ceil32(pm_min_residency[i].min_residency_us))) 
		{
			if(current.state != pm_min_residency[i].state)
			{
				LOG_INF("Selected power state %d (ticks: %d, min_residency: %u)",pm_min_residency[i].state, ticks,pm_min_residency[i].min_residency_us);
				current = pm_min_residency[i];
			}
			return pm_min_residency[i];
		}
	}

	//LOG_DBG("No suitable power state found!");
	return (struct pm_state_info){PM_STATE_ACTIVE, 0, 0};
}
#endif
```
## Zephyr patch
content of `zephyr\soc\arm\nordic_nrf\nrf52\power.c`

```c
/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <pm/pm.h>
#include <hal/nrf_power.h>
#include <hal/nrf_clock.h>

#include <logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

#define DEBUG_PIN_29_SET 	(*(int * const)0x50000508) = 0x20000000
#define DEBUG_PIN_29_CLEAR 	(*(int *) 0x5000050C) = 0x20000000


void sleep_prepare()
{
	nrf_clock_task_trigger(NRF_CLOCK,NRF_CLOCK_TASK_HFCLKSTOP);

	nrf_power_task_trigger(NRF_POWER,NRF_POWER_TASK_LOWPWR);

	DEBUG_PIN_29_SET;

	__WFE();
	// Clear the internal event register.
	__SEV();
	__WFE();
}

void sleep_wakeup()
{
    nrf_clock_event_clear(NRF_CLOCK,NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_clock_task_trigger(NRF_CLOCK,NRF_CLOCK_TASK_HFCLKSTART);
	while(!nrf_clock_hf_is_running(NRF_CLOCK,NRF_CLOCK_HFCLK_HIGH_ACCURACY));
	DEBUG_PIN_29_CLEAR;
}

/* Invoke Low Power/System Off specific Tasks */
void pm_power_state_set(struct pm_state_info info)
{
	switch (info.state) {
	case PM_STATE_SOFT_OFF:
		nrf_power_system_off(NRF_POWER);
		break;
	case PM_STATE_RUNTIME_IDLE:
	case PM_STATE_STANDBY:
	case PM_STATE_SUSPEND_TO_RAM:
	case PM_STATE_SUSPEND_TO_DISK:
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
	case PM_STATE_STANDBY:
	case PM_STATE_SUSPEND_TO_RAM:
	case PM_STATE_SUSPEND_TO_DISK:
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