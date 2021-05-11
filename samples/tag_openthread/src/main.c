
#include <zephyr.h>
#include <logging/log.h>
#include <net/socket.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define SLEEP_TIME_MS   10000

void send_udp()
{
	struct data *data = CONTAINER_OF(work, struct data, udp.recv);
	int ret = send(data->udp.sock, lorem_ipsum, data->udp.expecting, 0);
}

void main(void)
{
	LOG_INF("Hello from Sensor Tag");

	int count = 0;
	while (1) {
		send_udp();
		k_msleep(SLEEP_TIME_MS);
		LOG_INF("loop: %d",count++);
	}
}
