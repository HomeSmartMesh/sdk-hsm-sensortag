
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include <drivers/sensor.h>
#include <sensor/ms8607.h>
#include <stdio.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

void main(void)
{

	
	LOG_INF("MS8607 Temperature Humidity pressure sensor application");
	const struct device *dev = device_get_binding(DT_LABEL(DT_NODELABEL(adc)));
	if (!dev) {
		LOG_ERR("ms8607: device not found.");
		return;
	}

	if(ms8607_is_connected()){
		LOG_INF("ms8607> connected");
	}else{
		LOG_ERR("ms8607> not connected");
	}

	float t, p, h;

	while (1) {
		enum ms8607_status status = ms8607_read_temperature_pressure_humidity(&t,&p,&h);
		if(status == ms8607_status_ok){
			char degree=0xB0;
			printf("ms8607> t=%.2f %c  p=%.2f mbar  h=%.2f %%RH\n",t,degree,p,h);
		}else{
			LOG_ERR("ms8607> read sensors failed");
		}
		k_sleep(K_MSEC(5000));
	}
}
