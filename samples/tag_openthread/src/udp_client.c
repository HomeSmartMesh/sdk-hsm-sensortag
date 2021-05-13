//zephyr\samples\net\sockets\echo_client\src\udp.c
#include <zephyr.h>
#include <logging/log.h>
#include <net/socket.h>
#include <stdio.h>

LOG_MODULE_REGISTER(udp_client, LOG_LEVEL_INF);

bool udp_started = false;


/* Generated by http://www.lipsum.com/
 * 2 paragraphs, 179 words, 1160 bytes of Lorem Ipsum
 */
const char lorem_ipsum[] =
	"Lorem ipsum dolor sit amet, consectetur adipiscing elit. Quisque "
	"\n";
#define INVALID_SOCK (-1)
#define UDP_WAIT K_SECONDS(10)
#define PEER_PORT 4242

#if !defined(CONFIG_NET_CONFIG_PEER_IPV6_ADDR)
#define CONFIG_NET_CONFIG_PEER_IPV6_ADDR "ff02::1"
#endif

struct data {
	const char *proto;

	struct {
		int sock;
		/* Work controlling udp data sending */
		struct k_delayed_work recv;
		struct k_delayed_work transmit;
		uint32_t expecting;
		uint32_t counter;
		uint32_t mtu;
	} udp;

	struct {
		int sock;
		uint32_t expecting;
		uint32_t received;
		uint32_t counter;
	} tcp;
};

struct configs {
	struct data ipv6;
};
struct data ipv6= {
		.proto = "IPv6",
		.udp.sock = INVALID_SOCK,
		.tcp.sock = INVALID_SOCK,
};

static int send_udp_data(uint8_t * data, uint8_t size)
{
	int ret;

	ret = send(ipv6.udp.sock, data, size, 0);

	LOG_DBG("%s UDP: Sent %d bytes", ipv6.proto, size);

	return ret < 0 ? -EIO : 0;
}

static int start_udp_proto(struct sockaddr *addr, socklen_t addrlen)
{
	int ret;

	ipv6.udp.sock = socket(addr->sa_family, SOCK_DGRAM, IPPROTO_UDP);
	if (ipv6.udp.sock < 0) {
		LOG_ERR("Failed to create UDP socket (%s): %d", ipv6.proto,
			errno);
		return -errno;
	}

	/* Call connect so we can use send and recv. */
	ret = connect(ipv6.udp.sock, addr, addrlen);
	if (ret < 0) {
		LOG_ERR("Cannot connect to UDP remote (%s): %d", ipv6.proto,
			errno);
		ret = -errno;
	}

	return ret;
}


int start_udp(void)
{
    udp_started = true;
	int ret = 0;
	struct sockaddr_in6 addr6;

	if (IS_ENABLED(CONFIG_NET_IPV6)) {
        LOG_INF("CONFIG_NET_IPV6");
		addr6.sin6_family = AF_INET6;
		addr6.sin6_port = htons(PEER_PORT);
		inet_pton(AF_INET6, CONFIG_NET_CONFIG_PEER_IPV6_ADDR,&addr6.sin6_addr);

		ret = start_udp_proto((struct sockaddr *)&addr6, sizeof(addr6));
		if (ret < 0) {
			return ret;
		}
	}

	return ret;
}


int send_udp(uint8_t * data, uint8_t size)
{
    if(!udp_started){
        start_udp();
    }

	return send_udp_data(data, size);
}

