/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <errno.h>
#include <irq.h>
#include <sys/byteorder.h>
#include <nrf.h>
#include <esb.h>
#ifdef DPPI_PRESENT
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif
#include <helpers/nrfx_gppi.h>
#include <stddef.h>
#include <string.h>
#include <nrf_erratas.h>

/* Constants */

/* 2 Mb RX wait for acknowledgment time-out value.
 * Smallest reliable value: 160.
 */
#define RX_ACK_TIMEOUT_US_2MBPS 160
/* 1 Mb RX wait for acknowledgment time-out value. */
#define RX_ACK_TIMEOUT_US_1MBPS 300
/* 250 Kb RX wait for acknowledgment time-out value. */
#define RX_ACK_TIMEOUT_US_250KBPS 300
/* 1 Mb RX wait for acknowledgment time-out (combined with BLE). */
#define RX_ACK_TIMEOUT_US_1MBPS_BLE 300

/* Minimum retransmit time */
#define RETRANSMIT_DELAY_MIN 435

/* Interrupt flags */
/* Interrupt mask value for TX success. */
#define INT_TX_SUCCESS_MSK 0x01
/* Interrupt mask value for TX failure. */
#define INT_TX_FAILED_MSK 0x02
/* Interrupt mask value for RX_DR. */
#define INT_RX_DATA_RECEIVED_MSK 0x04

/* Mask value to signal updating BASE0 radio address. */
#define ADDR_UPDATE_MASK_BASE0 (1 << 0)
/* Mask value to signal updating BASE1 radio address. */
#define ADDR_UPDATE_MASK_BASE1 (1 << 1)
/* Mask value to signal updating radio prefixes. */
#define ADDR_UPDATE_MASK_PREFIX (1 << 2)

 /* The maximum value for PID. */
#define PID_MAX 3

#define BIT_MASK_UINT_8(x) (0xFF >> (8 - (x)))

#define RADIO_SHORTS_COMMON                                                    \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |         \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                                  \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

#ifdef CONFIG_ESB_SYS_TIMER0
#define ESB_SYS_TIMER NRF_TIMER0
#define ESB_SYS_TIMER_IRQn TIMER0_IRQn
#endif
#ifdef CONFIG_ESB_SYS_TIMER1
#define ESB_SYS_TIMER NRF_TIMER1
#define ESB_SYS_TIMER_IRQn TIMER1_IRQn
#endif
#ifdef CONFIG_ESB_SYS_TIMER2
#define ESB_SYS_TIMER NRF_TIMER2
#define ESB_SYS_TIMER_IRQn TIMER2_IRQn
#endif
#ifdef CONFIG_ESB_SYS_TIMER3
#define ESB_SYS_TIMER NRF_TIMER3
#define ESB_SYS_TIMER_IRQn TIMER3_IRQn
#endif
#ifdef CONFIG_ESB_SYS_TIMER4
#define ESB_SYS_TIMER NRF_TIMER4
#define ESB_SYS_TIMER_IRQn TIMER4_IRQn
#endif

/* Internal Enhanced ShockBurst module state. */
enum esb_state {
	ESB_STATE_IDLE,		/* Idle. */
	ESB_STATE_PTX_TX,       /* Transmitting without acknowledgment. */
	ESB_STATE_PTX_TX_ACK,   /* Transmitting with acknowledgment. */
	ESB_STATE_PTX_RX_ACK,   /* Transmitting with acknowledgment and
				 * reception of payload with the
				 * acknowledgment response.
				 */
	ESB_STATE_PRX,		/* Receiving packets without ACK. */
	ESB_STATE_PRX_SEND_ACK, /* Transmitting ACK in RX mode. */
};

/* Pipe info PID and CRC and acknowledgment payload. */
struct pipe_info {
	uint16_t crc;	  /* CRC of the last received packet.
			   * Used to detect retransmits.
			   */
	uint8_t pid;	  /* Packet ID of the last received packet
			   * Used to detect retransmits.
			   */
	bool ack_payload; /* State of the transmission of ACK payloads. */
};

/* Structure used by the PRX to organize ACK payloads for multiple pipes. */
struct payload_wrap {
	/* Pointer to the ACK payload. */
	struct esb_payload  *p_payload;
	/* Value used to determine if the current payload pointer is used. */
	bool in_use;
	/* Pointer to the next ACK payload queued on the same pipe. */
	struct payload_wrap *p_next;
};

/* First-in, first-out queue of payloads to be transmitted. */
struct payload_tx_fifo {
	 /* Payload queue */
	struct esb_payload *payload[CONFIG_ESB_TX_FIFO_SIZE];

	uint32_t back;	/* Back of the queue (last in). */
	uint32_t front;	/* Front of queue (first out). */
	uint32_t count;	/* Number of elements in the queue. */
};

/* First-in, first-out queue of received payloads. */
struct payload_rx_fifo {
	 /* Payload queue */
	struct esb_payload *payload[CONFIG_ESB_RX_FIFO_SIZE];

	uint32_t back;	/* Back of the queue (last in). */
	uint32_t front;	/* Front of queue (first out). */
	uint32_t count;	/* Number of elements in the queue. */
};

/* Enhanced ShockBurst address.
 *
 * Enhanced ShockBurst addresses consist of a base address and a prefix
 * that is unique for each pipe. See @ref esb_addressing in the ESB user
 * guide for more information.
 */
struct esb_address {
	uint8_t base_addr_p0[4];	/* Base address for pipe 0, in big endian. */
	uint8_t base_addr_p1[4];   /* Base address for pipe 1-7, in big endian. */
	uint8_t pipe_prefixes[8];	/* Address prefix for pipe 0 to 7. */
	uint8_t num_pipes;		/* Number of pipes available. */
	uint8_t addr_length;	/* Length of the address plus the prefix. */
	uint8_t rx_pipes_enabled;	/* Bitfield for enabled pipes. */
	uint8_t rf_channel;        /* Channel to use (between 0 and 100). */
};


static bool esb_initialized;
static struct esb_config esb_cfg;
static volatile enum esb_state esb_state = ESB_STATE_IDLE;

/* Default address configuration for ESB.
 * Roughly equal to the nRF24Lxx defaults, except for the number of pipes,
 * because more pipes are supported.
 */
__ALIGN(4)
static struct esb_address esb_addr = {
	.base_addr_p0 = {0xE7, 0xE7, 0xE7, 0xE7},
	.base_addr_p1 = {0xC2, 0xC2, 0xC2, 0xC2},
	.pipe_prefixes = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8},
	.addr_length = 5,
	.num_pipes = CONFIG_ESB_PIPE_COUNT,
	.rf_channel = 2,
	.rx_pipes_enabled = 0xFF
};

static esb_event_handler event_handler;
static struct esb_payload *current_payload;

/* FIFOs and buffers */
static struct payload_tx_fifo tx_fifo;
static struct payload_rx_fifo rx_fifo;
static uint8_t tx_payload_buffer[CONFIG_ESB_MAX_PAYLOAD_LENGTH + 2];
static uint8_t rx_payload_buffer[CONFIG_ESB_MAX_PAYLOAD_LENGTH + 2];

/* Random access buffer variables for ACK payload handling */
struct payload_wrap ack_pl_wrap[CONFIG_ESB_TX_FIFO_SIZE];
struct payload_wrap *ack_pl_wrap_pipe[CONFIG_ESB_PIPE_COUNT];

/* Run time variables */
static uint8_t pids[CONFIG_ESB_PIPE_COUNT];
static struct pipe_info rx_pipe_info[CONFIG_ESB_PIPE_COUNT];
static volatile uint32_t interrupt_flags;
static volatile uint32_t retransmits_remaining;
static volatile uint32_t last_tx_attempts;
static volatile uint32_t wait_for_ack_timeout_us;

static uint32_t radio_shorts_common = RADIO_SHORTS_COMMON;

/* PPI or DPPI instances */
#ifdef DPPI_PRESENT
typedef uint8_t ppi_channel_t;
#else
typedef nrf_ppi_channel_t ppi_channel_t;
#endif

static ppi_channel_t ppi_ch_radio_ready_timer_start;
static ppi_channel_t ppi_ch_radio_address_timer_stop;
static ppi_channel_t ppi_ch_timer_compare0_radio_disable;
static ppi_channel_t ppi_ch_timer_compare1_radio_txen;

static uint32_t ppi_all_channels_mask;

/* These function pointers are changed dynamically, depending on protocol
 * configuration and state. Note that they will be 0 initialized.
 */
static void (*on_radio_disabled)(void);
static void (*on_radio_end)(void);

/*  The following functions are assigned to the function pointers above. */
static void on_radio_disabled_tx_noack(void);
static void on_radio_disabled_tx(void);
static void on_radio_disabled_tx_wait_for_ack(void);
static void on_radio_disabled_rx(void);

/*  Function to do bytewise bit-swap on an unsigned 32-bit value */
static uint32_t bytewise_bit_swap(const uint8_t *input)
{
#if __CORTEX_M == (0x04U)
	uint32_t inp = (*(uint32_t *)input);

	return sys_cpu_to_be32((uint32_t)__RBIT(inp));
#else
	uint32_t inp = sys_cpu_to_le32(*(uint32_t *)input);

	inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
	inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
	inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
	return inp;
#endif
}

/* Convert a base address from nRF24L format to nRF5 format */
static uint32_t addr_conv(const uint8_t *addr)
{
	return __REV(bytewise_bit_swap(addr));
}

static inline void apply_errata143_workaround(void)
{
	/* Workaround for Errata 143
	 * Check if the most significant bytes of address 0 (including
	 * prefix) match those of another address. It's recommended to
	 * use a unique address 0 since this will avoid the 3dBm penalty
	 * incurred from the workaround.
	 */
	uint32_t base_address_mask =
		esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;

	/* Load the two addresses before comparing them to ensure
	 * defined ordering of volatile accesses.
	 */
	uint32_t addr0 = NRF_RADIO->BASE0 & base_address_mask;
	uint32_t addr1 = NRF_RADIO->BASE1 & base_address_mask;

	if (addr0 == addr1) {
		uint32_t prefix0 = NRF_RADIO->PREFIX0 & 0x000000FF;
		uint32_t prefix1 = (NRF_RADIO->PREFIX0 & 0x0000FF00) >> 8;
		uint32_t prefix2 = (NRF_RADIO->PREFIX0 & 0x00FF0000) >> 16;
		uint32_t prefix3 = (NRF_RADIO->PREFIX0 & 0xFF000000) >> 24;
		uint32_t prefix4 = NRF_RADIO->PREFIX1 & 0x000000FF;
		uint32_t prefix5 = (NRF_RADIO->PREFIX1 & 0x0000FF00) >> 8;
		uint32_t prefix6 = (NRF_RADIO->PREFIX1 & 0x00FF0000) >> 16;
		uint32_t prefix7 = (NRF_RADIO->PREFIX1 & 0xFF000000) >> 24;

		if (prefix0 == prefix1 || prefix0 == prefix2 ||
			prefix0 == prefix3 || prefix0 == prefix4 ||
			prefix0 == prefix5 || prefix0 == prefix6 ||
			prefix0 == prefix7) {
			/* This will cause a 3dBm sensitivity loss,
			 * avoid using such address combinations if possible.
			 */
			*(volatile uint32_t *)0x40001774 =
				((*(volatile uint32_t *)0x40001774) & 0xfffffffe) | 0x01000000;
		}
	}
}

static void update_rf_payload_format_esb_dpl()
{
    // modified to be byte alligned with no esb, simple custom HomeSmartMesh protocol
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
                       (8 << RADIO_PCNF0_LFLEN_Pos) |
                       (8 << RADIO_PCNF0_S1LEN_Pos) ;

    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
                       ((esb_addr.addr_length - 1)    << RADIO_PCNF1_BALEN_Pos)   |
                       (0                               << RADIO_PCNF1_STATLEN_Pos) |
                       (CONFIG_ESB_MAX_PAYLOAD_LENGTH   << RADIO_PCNF1_MAXLEN_Pos);

}

static void update_radio_addresses(uint8_t update_mask)
{
	if ((update_mask & ADDR_UPDATE_MASK_BASE0) != 0) {
		NRF_RADIO->BASE0 = addr_conv(esb_addr.base_addr_p0);
	}

	if ((update_mask & ADDR_UPDATE_MASK_BASE1) != 0) {
		NRF_RADIO->BASE1 = addr_conv(esb_addr.base_addr_p1);
	}

	if ((update_mask & ADDR_UPDATE_MASK_PREFIX) != 0) {
		NRF_RADIO->PREFIX0 =
			bytewise_bit_swap(&esb_addr.pipe_prefixes[0]);
		NRF_RADIO->PREFIX1 =
			bytewise_bit_swap(&esb_addr.pipe_prefixes[4]);
	}

	/* Workaround for Errata 143 */
#if NRF52_ERRATA_143_ENABLE_WORKAROUND
	if (nrf52_errata_143()) {
		apply_errata143_workaround();
	}
#endif
}

static void update_radio_tx_power(void)
{
	NRF_RADIO->TXPOWER = esb_cfg.tx_output_power
			     << RADIO_TXPOWER_TXPOWER_Pos;
}

static bool update_radio_bitrate(void)
{
	NRF_RADIO->MODE = esb_cfg.bitrate << RADIO_MODE_MODE_Pos;

	switch (esb_cfg.bitrate) {
	case ESB_BITRATE_2MBPS:
#if defined(CONFIG_SOC_SERIES_NRF52X) || defined(CONFIG_SOC_NRF5340_CPUNET)
	case ESB_BITRATE_2MBPS_BLE:
#endif
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_2MBPS;
		break;

	case ESB_BITRATE_1MBPS:
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_1MBPS;
		break;

#ifdef CONFIG_SOC_SERIES_NRF51X
	case ESB_BITRATE_250KBPS:
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_250KBPS;
		break;
#endif /* CONFIG_SOC_SERIES_NRF51X */

	case ESB_BITRATE_1MBPS_BLE:
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_1MBPS_BLE;
		break;

	default:
		/* Should not be reached */
		return false;
	}

	return true;
}

static bool update_radio_crc(void)
{
	switch (esb_cfg.crc) {
	case ESB_CRC_16BIT:
		NRF_RADIO->CRCINIT = 0xFFFFUL;  /* Initial value */
		NRF_RADIO->CRCPOLY = 0x11021UL; /* CRC poly: x^16+x^12^x^5+1 */
		break;

	case ESB_CRC_8BIT:
		NRF_RADIO->CRCINIT = 0xFFUL;  /* Initial value */
		NRF_RADIO->CRCPOLY = 0x107UL; /* CRC poly: x^8+x^2^x^1+1 */
		break;

	case ESB_CRC_OFF:
		break;

	default:
		return false;
	}

	NRF_RADIO->CRCINIT = 0xFFFFUL;  /* Initial value */
	NRF_RADIO->CRCPOLY = 0x11021UL; /* CRC poly: x^16+x^12^x^5+1 */
	NRF_RADIO->CRCCNF = ESB_CRC_16BIT << RADIO_CRCCNF_LEN_Pos;

	return true;
}

static bool update_radio_parameters(void)
{
	bool params_valid = true;

	update_radio_tx_power();
	params_valid &= update_radio_bitrate();
	params_valid &= update_radio_crc();
	update_rf_payload_format_esb_dpl();
	params_valid &=
	    (esb_cfg.retransmit_delay >= RETRANSMIT_DELAY_MIN);

	return params_valid;
}

static void reset_fifos(void)
{
	tx_fifo.back = 0;
	tx_fifo.front = 0;
	tx_fifo.count = 0;

	rx_fifo.back = 0;
	rx_fifo.front = 0;
	rx_fifo.count = 0;
}

static void initialize_fifos(void)
{
	static struct esb_payload rx_payload[CONFIG_ESB_RX_FIFO_SIZE];
	static struct esb_payload tx_payload[CONFIG_ESB_TX_FIFO_SIZE];

	reset_fifos();

	for (size_t i = 0; i < CONFIG_ESB_TX_FIFO_SIZE; i++) {
		tx_fifo.payload[i] = &tx_payload[i];
	}

	for (size_t i = 0; i < CONFIG_ESB_RX_FIFO_SIZE; i++) {
		rx_fifo.payload[i] = &rx_payload[i];
	}

	for (size_t i = 0; i < CONFIG_ESB_TX_FIFO_SIZE; i++) {
		ack_pl_wrap[i].p_payload = &tx_payload[i];
		ack_pl_wrap[i].in_use = false;
		ack_pl_wrap[i].p_next = 0;
	}

	for (size_t i = 0; i < CONFIG_ESB_PIPE_COUNT; i++) {
		ack_pl_wrap_pipe[i] = 0;
	}
}

static void tx_fifo_remove_last(void)
{
	if (tx_fifo.count == 0) {
		return;
	}

	uint32_t key = irq_lock();

	tx_fifo.count--;
	if (++tx_fifo.front >= CONFIG_ESB_TX_FIFO_SIZE) {
		tx_fifo.front = 0;
	}

	irq_unlock(key);
}

/*  Function to push the content of the rx_buffer to the RX FIFO.
 *
 *  The module will point the register NRF_RADIO->PACKETPTR to a buffer for
 *  receiving packets. After receiving a packet the module will call this
 *  function to copy the received data to the RX FIFO.
 *
 *  @param  pipe Pipe number to set for the packet.
 *  @param  pid  Packet ID.
 *
 *  @retval true   Operation successful.
 *  @retval false  Operation failed.
 */
static bool rx_fifo_push_rfbuf(uint8_t pipe)
{
	if (rx_fifo.count >= CONFIG_ESB_RX_FIFO_SIZE) {
		return false;
	}

	//esb_cfg.protocol == ESB_PROTOCOL_ESB_DPL
	if (rx_payload_buffer[0] > CONFIG_ESB_MAX_PAYLOAD_LENGTH) {
		return false;
	}
	rx_fifo.payload[rx_fifo.back]->length = rx_payload_buffer[0];

    //copy everything, length included although it is already available on the length param
	memcpy(rx_fifo.payload[rx_fifo.back]->data, &rx_payload_buffer[0],
	       rx_fifo.payload[rx_fifo.back]->length);

	rx_fifo.payload[rx_fifo.back]->rssi = NRF_RADIO->RSSISAMPLE;
	//TODO HSM Check ack : rx_fifo.payload[rx_fifo.back]->noack = !(rx_payload_buffer[1] & 0x01);

	if (++rx_fifo.back >= CONFIG_ESB_RX_FIFO_SIZE) {
		rx_fifo.back = 0;
	}
	rx_fifo.count++;

	return true;
}

static void sys_timer_init(void)
{
	/* Configure the system timer with a 1 MHz base frequency */
	ESB_SYS_TIMER->PRESCALER = 4;
	ESB_SYS_TIMER->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
	ESB_SYS_TIMER->SHORTS = TIMER_SHORTS_COMPARE1_CLEAR_Msk |
				TIMER_SHORTS_COMPARE1_STOP_Msk;
}

static void ppi_init(void)
{
#ifdef DPPI_PRESENT
	nrfx_dppi_channel_alloc(&ppi_ch_radio_ready_timer_start);
	nrfx_dppi_channel_alloc(&ppi_ch_radio_address_timer_stop);
	nrfx_dppi_channel_alloc(&ppi_ch_timer_compare0_radio_disable);
	nrfx_dppi_channel_alloc(&ppi_ch_timer_compare1_radio_txen);

	NRF_RADIO->PUBLISH_READY          = DPPIC_SUBSCRIBE_CHG_EN_EN_Msk | ppi_ch_radio_ready_timer_start;
	ESB_SYS_TIMER->SUBSCRIBE_START    = DPPIC_SUBSCRIBE_CHG_EN_EN_Msk | ppi_ch_radio_ready_timer_start;
	NRF_RADIO->PUBLISH_ADDRESS        = DPPIC_SUBSCRIBE_CHG_EN_EN_Msk | ppi_ch_radio_address_timer_stop;
	ESB_SYS_TIMER->SUBSCRIBE_SHUTDOWN = DPPIC_SUBSCRIBE_CHG_EN_EN_Msk | ppi_ch_radio_address_timer_stop;
	ESB_SYS_TIMER->PUBLISH_COMPARE[0] = DPPIC_SUBSCRIBE_CHG_EN_EN_Msk | ppi_ch_timer_compare0_radio_disable;
	NRF_RADIO->SUBSCRIBE_DISABLE      = DPPIC_SUBSCRIBE_CHG_EN_EN_Msk | ppi_ch_timer_compare0_radio_disable;
	ESB_SYS_TIMER->PUBLISH_COMPARE[1] = DPPIC_SUBSCRIBE_CHG_EN_EN_Msk | ppi_ch_timer_compare1_radio_txen;
	NRF_RADIO->SUBSCRIBE_TXEN         = DPPIC_SUBSCRIBE_CHG_EN_EN_Msk | ppi_ch_timer_compare1_radio_txen;
#else
	nrfx_ppi_channel_alloc(&ppi_ch_radio_ready_timer_start);
	nrfx_ppi_channel_alloc(&ppi_ch_radio_address_timer_stop);
	nrfx_ppi_channel_alloc(&ppi_ch_timer_compare0_radio_disable);
	nrfx_ppi_channel_alloc(&ppi_ch_timer_compare1_radio_txen);

	nrfx_ppi_channel_assign(ppi_ch_radio_ready_timer_start,
		(uint32_t)&NRF_RADIO->EVENTS_READY, (uint32_t)&ESB_SYS_TIMER->TASKS_START);
	nrfx_ppi_channel_assign(ppi_ch_radio_address_timer_stop,
		(uint32_t)&NRF_RADIO->EVENTS_ADDRESS, (uint32_t)&ESB_SYS_TIMER->TASKS_SHUTDOWN);
	nrfx_ppi_channel_assign(ppi_ch_timer_compare0_radio_disable,
		(uint32_t)&ESB_SYS_TIMER->EVENTS_COMPARE[0], (uint32_t)&NRF_RADIO->TASKS_DISABLE);
	nrfx_ppi_channel_assign(ppi_ch_timer_compare1_radio_txen,
		(uint32_t)&ESB_SYS_TIMER->EVENTS_COMPARE[1], (uint32_t)&NRF_RADIO->TASKS_TXEN);
#endif
	ppi_all_channels_mask = (1 << ppi_ch_radio_ready_timer_start) | (1 << ppi_ch_radio_address_timer_stop) |
							(1 << ppi_ch_timer_compare0_radio_disable) | (1 << ppi_ch_timer_compare1_radio_txen);
}

static void start_tx_transaction(void)
{
	last_tx_attempts = 1;
	/* Prepare the payload */
	current_payload = tx_fifo.payload[tx_fifo.front];

	memcpy(&tx_payload_buffer[0], current_payload->data,
			current_payload->length);

	NRF_RADIO->SHORTS = radio_shorts_common;
	NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
	on_radio_disabled = on_radio_disabled_tx_noack;
	esb_state = ESB_STATE_PTX_TX;

	NRF_RADIO->TXADDRESS = current_payload->pipe;
	NRF_RADIO->RXADDRESSES = 1 << current_payload->pipe;
	NRF_RADIO->FREQUENCY = esb_addr.rf_channel;

	NRF_RADIO->PACKETPTR = (uint32_t)tx_payload_buffer;

	NVIC_ClearPendingIRQ(RADIO_IRQn);
	irq_enable(RADIO_IRQn);

	NRF_RADIO->EVENTS_ADDRESS = 0;
	NRF_RADIO->EVENTS_PAYLOAD = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;

	NRF_RADIO->TASKS_TXEN = 1;
}

static void on_radio_disabled_tx_noack(void)
{
	interrupt_flags |= INT_TX_SUCCESS_MSK;
	tx_fifo_remove_last();

	if (tx_fifo.count == 0) {
		esb_state = ESB_STATE_IDLE;
		NVIC_SetPendingIRQ(ESB_EVT_IRQ);
	} else {
		NVIC_SetPendingIRQ(ESB_EVT_IRQ);
		start_tx_transaction();
	}
}

static void on_radio_disabled_tx(void)
{
	/* Remove the DISABLED -> RXEN shortcut, to make sure the radio stays
	 * disabled after the RX window
	 */
	NRF_RADIO->SHORTS = radio_shorts_common;

	/* Make sure the timer is started the next time the radio is ready,
	 * and that it will disable the radio automatically if no packet is
	 * received by the time defined in wait_for_ack_timeout_us
	 */
	ESB_SYS_TIMER->CC[0] = wait_for_ack_timeout_us;
	ESB_SYS_TIMER->CC[1] = esb_cfg.retransmit_delay - 130;
	ESB_SYS_TIMER->TASKS_CLEAR = 1;
	ESB_SYS_TIMER->EVENTS_COMPARE[0] = 0;
	ESB_SYS_TIMER->EVENTS_COMPARE[1] = 0;

	/* Remove */
	ESB_SYS_TIMER->TASKS_START = 1;

	nrfx_gppi_channels_enable(ppi_all_channels_mask);
	nrfx_gppi_channels_disable(1 << ppi_ch_timer_compare1_radio_txen);

	NRF_RADIO->EVENTS_END = 0;

	update_rf_payload_format_esb_dpl();//TODO update might not be required here

	NRF_RADIO->PACKETPTR = (uint32_t)rx_payload_buffer;
	on_radio_disabled = on_radio_disabled_tx_wait_for_ack;
	esb_state = ESB_STATE_PTX_RX_ACK;
}

static void on_radio_disabled_tx_wait_for_ack(void)
{
	/* This marks the completion of a TX_RX sequence (TX with ACK) */

	/* Make sure the timer will not deactivate the radio if a packet is
	 * received.
	 */
	nrfx_gppi_channels_disable(ppi_all_channels_mask);

	/* If the radio has received a packet and the CRC status is OK */
	if (NRF_RADIO->EVENTS_END && NRF_RADIO->CRCSTATUS != 0) {
		ESB_SYS_TIMER->TASKS_SHUTDOWN = 1;

		interrupt_flags |= INT_TX_SUCCESS_MSK;
		last_tx_attempts = esb_cfg.retransmit_count -
				   retransmits_remaining + 1;

		tx_fifo_remove_last();

		if (esb_cfg.protocol != ESB_PROTOCOL_ESB &&
		    rx_payload_buffer[0] > 0) {
			if (rx_fifo_push_rfbuf((uint8_t)NRF_RADIO->TXADDRESS)) {
				interrupt_flags |=
					INT_RX_DATA_RECEIVED_MSK;
			}
		}

		if ((tx_fifo.count == 0) ||
		    (esb_cfg.tx_mode == ESB_TXMODE_MANUAL)) {
			esb_state = ESB_STATE_IDLE;
			NVIC_SetPendingIRQ(ESB_EVT_IRQ);
		} else {
			NVIC_SetPendingIRQ(ESB_EVT_IRQ);
			start_tx_transaction();
		}
	} else {
		if (retransmits_remaining-- == 0) {
			ESB_SYS_TIMER->TASKS_SHUTDOWN = 1;

			/* All retransmits are expended, and the TX operation is
			 * suspended
			 */
			last_tx_attempts = esb_cfg.retransmit_count + 1;
			interrupt_flags |= INT_TX_FAILED_MSK;

			esb_state = ESB_STATE_IDLE;
			NVIC_SetPendingIRQ(ESB_EVT_IRQ);
		} else {
			/* There are still more retransmits left, TX mode should
			 * be entered again as soon as the system timer reaches
			 * CC[1].
			 */
			NRF_RADIO->SHORTS = radio_shorts_common |
					    RADIO_SHORTS_DISABLED_RXEN_Msk;
			update_rf_payload_format_esb_dpl();
			NRF_RADIO->PACKETPTR = (uint32_t)tx_payload_buffer;
			on_radio_disabled = on_radio_disabled_tx;
			esb_state = ESB_STATE_PTX_TX_ACK;
			ESB_SYS_TIMER->TASKS_START = 1;
			nrfx_gppi_channels_enable(1 << ppi_ch_timer_compare1_radio_txen);
			if (ESB_SYS_TIMER->EVENTS_COMPARE[1]) {
				NRF_RADIO->TASKS_TXEN = 1;
			}
		}
	}
}

static void clear_events_restart_rx(void)
{
	NRF_RADIO->SHORTS = radio_shorts_common;
	update_rf_payload_format_esb_dpl();
	NRF_RADIO->PACKETPTR = (uint32_t)rx_payload_buffer;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_DISABLE = 1;

	while (NRF_RADIO->EVENTS_DISABLED == 0) {
		/* wait for register to settle */
	}

	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->SHORTS = radio_shorts_common |
			    RADIO_SHORTS_DISABLED_TXEN_Msk;

	NRF_RADIO->TASKS_RXEN = 1;
}

static void on_radio_disabled_rx(void)
{

	if (NRF_RADIO->CRCSTATUS == 0) {
		clear_events_restart_rx();
		return;
	}

	if (rx_fifo.count >= CONFIG_ESB_RX_FIFO_SIZE) {
		clear_events_restart_rx();
		return;
	}

	clear_events_restart_rx();

	/* Push the new packet to the RX buffer and trigger a received
		* event if the operation was
		* successful.
		*/
	if (rx_fifo_push_rfbuf(NRF_RADIO->RXMATCH)) {
		interrupt_flags |= INT_RX_DATA_RECEIVED_MSK;
		NVIC_SetPendingIRQ(ESB_EVT_IRQ);
	}
}

/* Retrieve interrupt flags and reset them.
 *
 * @param[out] interrupts	Interrupt flags.
 */
static void get_and_clear_irqs(uint32_t *interrupts)
{
	__ASSERT_NO_MSG(interrupts != NULL);

	uint32_t key = irq_lock();

	*interrupts = interrupt_flags;
	interrupt_flags = 0;

	irq_unlock(key);
}

static void RADIO_IRQHandler(void)
{
    //nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_SLEEP);
    //nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_SLEEP);

	if (NRF_RADIO->EVENTS_READY &&
	    (NRF_RADIO->INTENSET & RADIO_INTENSET_READY_Msk)) {
		NRF_RADIO->EVENTS_READY = 0;
		ESB_SYS_TIMER->TASKS_START;
	}

	if (NRF_RADIO->EVENTS_END &&
	    (NRF_RADIO->INTENSET & RADIO_INTENSET_END_Msk)) {
		NRF_RADIO->EVENTS_END = 0;
		/* Call the correct on_radio_end function, depending on the
		 * current protocol state.
		 */
		if (on_radio_end) {
			on_radio_end();
		}
	}

	if (NRF_RADIO->EVENTS_DISABLED &&
	    (NRF_RADIO->INTENSET & RADIO_INTENSET_DISABLED_Msk)) {
		NRF_RADIO->EVENTS_DISABLED = 0;
		/* Call the correct on_radio_disable function, depending on the
		 * current protocol state.
		 */
		if (on_radio_disabled) {
			on_radio_disabled();
		}
	}
}

static void ESB_EVT_IRQHandler(void)
{
	uint32_t interrupts;
	struct esb_evt event;

	event.tx_attempts = last_tx_attempts;

	get_and_clear_irqs(&interrupts);
	if (event_handler != NULL) {
		if (interrupts & INT_TX_SUCCESS_MSK) {
			event.evt_id = ESB_EVENT_TX_SUCCESS;
			event_handler(&event);
		}
		if (interrupts & INT_TX_FAILED_MSK) {
			event.evt_id = ESB_EVENT_TX_FAILED;
			event_handler(&event);
		}
		if (interrupts & INT_RX_DATA_RECEIVED_MSK) {
			event.evt_id = ESB_EVENT_RX_RECEIVED;
			event_handler(&event);
		}
	}
}

static void ESB_SYS_TIMER_IRQHandler(void)
{
}

int esb_init(const struct esb_config *config)
{
	if (config == NULL) {
		return -EINVAL;
	}

	if (esb_initialized) {
		esb_disable();
	}

	event_handler = config->event_handler;

	memcpy(&esb_cfg, config, sizeof(esb_cfg));

	interrupt_flags = 0;

	update_radio_parameters();

	/* Configure radio address registers according to ESB default values */
	NRF_RADIO->BASE0 = 0xE7E7E7E7;
	NRF_RADIO->BASE1 = 0x43434343;
	NRF_RADIO->PREFIX0 = 0x23C343E7;
	NRF_RADIO->PREFIX1 = 0x13E363A3;

	//will be overridden by further user's call to set_base_address_X

	initialize_fifos();
	sys_timer_init();
	ppi_init();

	IRQ_DIRECT_CONNECT(RADIO_IRQn, config->radio_irq_priority,
			   RADIO_IRQHandler, 0);
	IRQ_DIRECT_CONNECT(ESB_EVT_IRQ, config->event_irq_priority,
			   ESB_EVT_IRQHandler, 0);
	IRQ_DIRECT_CONNECT(ESB_SYS_TIMER_IRQn, config->event_irq_priority,
			   ESB_SYS_TIMER_IRQHandler, 0);

	irq_enable(RADIO_IRQn);
	irq_enable(ESB_EVT_IRQ);
	irq_enable(ESB_SYS_TIMER_IRQn);

	esb_state = ESB_STATE_IDLE;
	esb_initialized = true;

#ifdef CONFIG_SOC_NRF52832
	if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004500) {
		/* Check if the device is an nRF52832 Rev. 2. */
		/* Workaround for nRF52832 rev 2 errata 182 */
		*(volatile uint32_t *)0x4000173C |= (1 << 10);
	}
#endif

	return 0;
}

int esb_suspend(void)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	/*  Clear PPI */
	nrfx_gppi_channels_disable(ppi_all_channels_mask);

	esb_state = ESB_STATE_IDLE;

	return 0;
}

void esb_disable(void)
{
	/*  Clear PPI */
	nrfx_gppi_channels_disable(ppi_all_channels_mask);

	esb_state = ESB_STATE_IDLE;
	esb_initialized = false;

	reset_fifos();

	/*  Disable the radio */
	irq_disable(ESB_EVT_IRQ);

	NRF_RADIO->SHORTS =
	    RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos |
	    RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos;
}

bool esb_is_idle(void)
{
	return (esb_state == ESB_STATE_IDLE);
}

int esb_write_payload(const struct esb_payload *payload)
{
	if (!esb_initialized) {
		return -EACCES;
	}
	if (payload == NULL) {
		return -EINVAL;
	}
	//now the payload contains HSM header
	if (payload->length == 0 ||
	    payload->length > CONFIG_ESB_MAX_PAYLOAD_LENGTH ||
	    (esb_cfg.protocol == ESB_PROTOCOL_ESB &&
	     payload->length > esb_cfg.payload_length)) {
		return -EMSGSIZE;
	}
	if (tx_fifo.count >= CONFIG_ESB_TX_FIFO_SIZE) {
		return -ENOMEM;
	}
	if (payload->pipe >= CONFIG_ESB_PIPE_COUNT) {
		return -EINVAL;
	}

	uint32_t key = irq_lock();

	memcpy(tx_fifo.payload[tx_fifo.back], payload,
		sizeof(struct esb_payload));

	if (++tx_fifo.back >= CONFIG_ESB_TX_FIFO_SIZE) {
		tx_fifo.back = 0;
	}

	tx_fifo.count++;

	irq_unlock(key);

	if (esb_cfg.tx_mode == ESB_TXMODE_AUTO &&
	    esb_state == ESB_STATE_IDLE) {
		start_tx_transaction();
	}

	return 0;
}

int esb_read_rx_payload(struct esb_payload *payload)
{
	if (!esb_initialized) {
		return -EACCES;
	}
	if (payload == NULL) {
		return -EINVAL;
	}

	if (rx_fifo.count == 0) {
		return -ENODATA;
	}

	uint32_t key = irq_lock();

    struct esb_payload * p_exit_payload = rx_fifo.payload[rx_fifo.front];//exit_point replaced by "Front of queue (first out)."
    payload->length  = p_exit_payload->length;
    payload->pipe    = p_exit_payload->pipe;
    payload->rssi    = p_exit_payload->rssi;

    memcpy( payload->data,p_exit_payload->data,payload->length);

	if (++rx_fifo.front >= CONFIG_ESB_RX_FIFO_SIZE) {
		rx_fifo.front = 0;
	}

	rx_fifo.count--;

	irq_unlock(key);

	return 0;
}

int esb_start_tx(void)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	if (tx_fifo.count == 0) {
		return -ENODATA;
	}

	start_tx_transaction();

	return 0;
}

int esb_start_rx(void)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	NRF_RADIO->INTENCLR = 0xFFFFFFFF;
	NRF_RADIO->EVENTS_DISABLED = 0;
	on_radio_disabled = on_radio_disabled_rx;

	NRF_RADIO->SHORTS = radio_shorts_common |
			    RADIO_SHORTS_DISABLED_TXEN_Msk;
	NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
	esb_state = ESB_STATE_PRX;

	NRF_RADIO->RXADDRESSES = esb_addr.rx_pipes_enabled;
	NRF_RADIO->FREQUENCY = esb_addr.rf_channel;
	NRF_RADIO->PACKETPTR = (uint32_t)rx_payload_buffer;

	NVIC_ClearPendingIRQ(RADIO_IRQn);
	irq_enable(RADIO_IRQn);

	NRF_RADIO->EVENTS_ADDRESS = 0;
	NRF_RADIO->EVENTS_PAYLOAD = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;

	NRF_RADIO->TASKS_RXEN = 1;

	return 0;
}

int esb_stop_rx(void)
{
	if (esb_state != ESB_STATE_PRX && esb_state != ESB_STATE_PRX_SEND_ACK) {
		return -EINVAL;
	}

	NRF_RADIO->SHORTS = 0;
	NRF_RADIO->INTENCLR = 0xFFFFFFFF;
	on_radio_disabled = NULL;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_DISABLE = 1;
	while (NRF_RADIO->EVENTS_DISABLED == 0) {
		/* wait for register to settle */
	}

	esb_state = ESB_STATE_IDLE;

	return 0;
}

int esb_flush_tx(void)
{
	if (!esb_initialized) {
		return -EACCES;
	}

	uint32_t key = irq_lock();

	tx_fifo.count = 0;
	tx_fifo.back = 0;
	tx_fifo.front = 0;

	irq_unlock(key);

	return 0;
}

int esb_pop_tx(void)
{
	if (!esb_initialized) {
		return -EACCES;
	}
	if (tx_fifo.count == 0) {
		return -ENODATA;
	}

	uint32_t key = irq_lock();

	if (++tx_fifo.back >= CONFIG_ESB_TX_FIFO_SIZE) {
		tx_fifo.back = 0;
	}
	tx_fifo.count--;

	irq_unlock(key);

	return 0;
}

int esb_flush_rx(void)
{
	if (!esb_initialized) {
		return -EACCES;
	}

	uint32_t key = irq_lock();

	rx_fifo.count = 0;
	rx_fifo.back = 0;
	rx_fifo.front = 0;

	memset(rx_pipe_info, 0, sizeof(rx_pipe_info));

	irq_unlock(key);

	return 0;
}

int esb_set_address_length(uint8_t length)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (!(length > 2 && length < 6)) {
		return -EINVAL;
	}

	esb_addr.addr_length = length;

	update_rf_payload_format_esb_dpl();

	return 0;
}

int esb_set_base_address_0(const uint8_t *addr)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (addr == NULL) {
		return -EINVAL;
	}

	memcpy(esb_addr.base_addr_p0, addr, sizeof(esb_addr.base_addr_p0));

	update_radio_addresses(ADDR_UPDATE_MASK_BASE0);

	return 0;
}

int esb_set_base_address_1(const uint8_t *addr)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (addr == NULL) {
		return -EINVAL;
	}

	memcpy(esb_addr.base_addr_p1, addr, sizeof(esb_addr.base_addr_p1));

	update_radio_addresses(ADDR_UPDATE_MASK_BASE1);

	return 0;
}

int esb_set_prefixes(const uint8_t *prefixes, uint8_t num_pipes)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (prefixes == NULL) {
		return -EINVAL;
	}
	if (!(num_pipes <= CONFIG_ESB_PIPE_COUNT)) {
		return -EINVAL;
	}

	memcpy(esb_addr.pipe_prefixes, prefixes, num_pipes);
	esb_addr.num_pipes = num_pipes;
	esb_addr.rx_pipes_enabled = BIT_MASK_UINT_8(num_pipes);

	update_radio_addresses(ADDR_UPDATE_MASK_PREFIX);

	return 0;
}

int esb_update_prefix(uint8_t pipe, uint8_t prefix)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (pipe >= CONFIG_ESB_PIPE_COUNT) {
		return -EINVAL;
	}

	esb_addr.pipe_prefixes[pipe] = prefix;

	update_radio_addresses(ADDR_UPDATE_MASK_PREFIX);

	return 0;
}

int esb_enable_pipes(uint8_t enable_mask)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if ((enable_mask | BIT_MASK_UINT_8(CONFIG_ESB_PIPE_COUNT)) !=
	    BIT_MASK_UINT_8(CONFIG_ESB_PIPE_COUNT)) {
		return -EINVAL;
	}

	esb_addr.rx_pipes_enabled = enable_mask;

	return 0;
}

int esb_set_rf_channel(uint32_t channel)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (channel > 100) {
		return -EINVAL;
	}

	esb_addr.rf_channel = channel;

	return 0;
}

int esb_get_rf_channel(uint32_t *channel)
{
	if (channel == NULL) {
		return -EINVAL;
	}

	*channel = esb_addr.rf_channel;

	return 0;
}

int esb_set_tx_power(enum esb_tx_power tx_output_power)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	if (esb_cfg.tx_output_power != tx_output_power) {
		esb_cfg.tx_output_power = tx_output_power;
		update_radio_tx_power();
	}

	return 0;
}

int esb_set_retransmit_delay(uint16_t delay)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (delay < RETRANSMIT_DELAY_MIN) {
		return -EINVAL;
	}

	esb_cfg.retransmit_delay = delay;

	return 0;
}

int esb_set_retransmit_count(uint16_t count)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	esb_cfg.retransmit_count = count;

	return 0;
}

int esb_set_bitrate(enum esb_bitrate bitrate)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	esb_cfg.bitrate = bitrate;

	return update_radio_bitrate() ? 0 : -EINVAL;
}

int esb_reuse_pid(uint8_t pipe)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (!(pipe < CONFIG_ESB_PIPE_COUNT)) {
		return -EINVAL;
	}

	pids[pipe] = (pids[pipe] + PID_MAX) % (PID_MAX + 1);

	return 0;
}
