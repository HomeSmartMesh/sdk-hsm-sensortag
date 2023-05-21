//zephyr\samples\net\sockets\echo_client\src\udp.c
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <stdio.h>

LOG_MODULE_REGISTER(udp_client, LOG_LEVEL_INF);

bool udp_started = false;

#define CONFIG_PEER_PORT 4242
#if !defined(CONFIG_NET_CONFIG_PEER_IPV6_ADDR)
#define CONFIG_NET_CONFIG_PEER_IPV6_ADDR "ff02::1"
#endif
#define INVALID_SOCK (-1)

int sock = INVALID_SOCK;

int start_udp(void)
{
	int ret = 0;
	struct sockaddr_in6 addr6;

	LOG_INF("CONFIG_NET_IPV6");
	addr6.sin6_family = AF_INET6;
	addr6.sin6_port = htons(CONFIG_PEER_PORT);
	inet_pton(AF_INET6, CONFIG_NET_CONFIG_PEER_IPV6_ADDR,&addr6.sin6_addr);

	sock = socket(addr6.sin6_family, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("Failed to create UDP socket : %d", errno);
		return -errno;
	}

	/* Call connect so we can use send and recv. */
	ret = connect(sock, (struct sockaddr *)&addr6, sizeof(addr6));
	if (ret < 0) {
		LOG_ERR("Cannot connect to UDP remote : %d", errno);
		ret = -errno;
	}

	return ret;
}


int send_udp(uint8_t * data, uint8_t size)
{
    if(!udp_started){
        start_udp();
	    udp_started = true;
    }

	int ret;

	ret = send(sock, data, size, 0);

	LOG_DBG("UDP: Sent %d bytes", size);

	return ret < 0 ? -EIO : 0;

}

