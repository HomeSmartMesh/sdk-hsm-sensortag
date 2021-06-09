/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <irq.h>
#include <logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <stdio.h>

#include "simplemesh.h"

LOG_MODULE_REGISTER(esb_ptrx, LOG_LEVEL_DBG);

static struct esb_payload tx_payload;
static struct esb_payload rx_payload;
static message_t rx_msg;

static volatile bool esb_completed = false;
static volatile bool esb_tx_complete = false;
static uint8_t g_ttl = 2;

bool UICR_is_listening()
{
	return true;
}

void mesh_pre_tx()
{
    if(UICR_is_listening())
    {
        esb_stop_rx();
        LOG_DBG("switch to IDLE mode that aloows TX");
    }

}

void mesh_post_tx()
{
    if(UICR_is_listening())
    {
        esb_start_rx();
        LOG_DBG("switch to RX mode");
    }
	esb_tx_complete = true;
}


void mesh_message_2_esb_payload(message_t *msg, struct esb_payload *p_tx_payload)
{
    //esb only parameters
    p_tx_payload->noack    = true;//Never request an ESB acknowledge
    p_tx_payload->pipe     = 0;//pipe is the selection of the address to use

    p_tx_payload->data[1] = msg->control;
    p_tx_payload->data[2] = msg->pid;
    p_tx_payload->data[3] = msg->source;

    uint8_t start_payload;
    if(MESH_IS_BROADCAST(msg->control))
    {
        start_payload = MESH_Broadcast_Header_Length;
    }
    else
    {
        p_tx_payload->data[4] = msg->dest;
        start_payload = MESH_P2P_Header_Length;
    }

    //this is the total ESB packet length
    p_tx_payload->length   = msg->payload_length + start_payload;
    p_tx_payload->data[0] = p_tx_payload->length;
    memcpy( p_tx_payload->data+start_payload,
            msg->payload,
            msg->payload_length);
}

void mesh_esb_2_message_payload(struct esb_payload *p_rx_payload,message_t *msg)
{
    msg->control = p_rx_payload->data[1];
    msg->pid = p_rx_payload->data[2];
    msg->source = p_rx_payload->data[3];

    msg->rssi = p_rx_payload->rssi;
    //dest is processed by esb rx function
    uint8_t payload_start;
    if(MESH_IS_BROADCAST(msg->control))
    {
        msg->dest = 255;
        payload_start = 4;
    }
    else
    {
        msg->dest = p_rx_payload->data[4];
        payload_start = 5;
    }
    msg->payload_length = p_rx_payload->length - payload_start;

    if(msg->payload_length > 0)
    {
        msg->payload = p_rx_payload->data + payload_start;
    }
}

void mesh_tx_message(message_t* p_msg)
{
    mesh_pre_tx();

    mesh_message_2_esb_payload(p_msg,&tx_payload);

    esb_completed = false;//reset the check
    LOG_DBG("TX esb payload length = %d",tx_payload.data[0]);
    //should not wait for esb_completed here as does not work from ISR context

    //NRF_ESB_TXMODE_AUTO is used no need to call nrf_esb_start_tx()
    //which could be used for a precise time transmission
    esb_write_payload(&tx_payload);
    
}

void mesh_bcast_data(uint8_t pid,uint8_t * data,uint8_t size)
{
    message_t msg;

    msg.control = 0x80 | g_ttl;         // broadcast | ttl = g_ttl
    msg.pid     = pid;
    msg.source  = 0xFF;//TODO config node id
    msg.payload = data;
    msg.payload_length = size;

    mesh_tx_message(&msg);
}
//limited to 255
void mesh_bcast_text(char *text)
{
    uint8_t size = strlen(text);
    if(size>MAX_MESH_MESSAGE_SIZE)//truncate in case of long message
    {
        text[MAX_MESH_MESSAGE_SIZE-1] = '>';
        size = MAX_MESH_MESSAGE_SIZE;
    }
    mesh_bcast_data(Mesh_Pid_Text,(uint8_t*)text,size);
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

void mesh_consume_rx_messages()
{
    while(esb_read_rx_payload(&rx_payload) == 0)
    {
        mesh_esb_2_message_payload(&rx_payload,&rx_msg);
        LOG_INF("RX> source:%d , pid:0x%02X , length:%d",rx_msg.source,rx_msg.pid, rx_msg.payload_length);
		if(rx_msg.pid == Mesh_Pid_Text)
		{
			printk("text = '%s'",(char*)rx_msg.payload);
		}
    }
}

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
        mesh_post_tx();
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		(void) esb_flush_tx();
		mesh_post_tx();
		break;
	case ESB_EVENT_RX_RECEIVED:
		LOG_DBG("RX RECEIVED");
		mesh_consume_rx_messages();
		break;
	default:
		LOG_ERR("ESB Unhandled Event (%d)",event->evt_id);
		break;
	}
	esb_completed = true;
}


int esb_initialize(enum esb_mode mode)
{
	int err;
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.retransmit_delay = 600;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.mode = mode;
	config.event_handler = event_handler;
	config.selective_auto_ack = true;

	err = esb_init(&config);
	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	LOG_INF("setting channel 2");
    err = esb_set_rf_channel(2);
	if (err) {
		return err;
	}

	return 0;
}


void sm_start_rx()
{
	int err;
	LOG_INF("rx test INF");
	LOG_DBG("rx test DBG");
	LOG_INF("Enhanced ShockBurst prx sample");

	err = clocks_start();
	if (err) {
		return;
	}

	err = esb_initialize(ESB_MODE_PRX);
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return;
	}
	LOG_INF("Initialization complete");

	LOG_INF("Setting up for packet receiption");
	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return;
	}

	return;
}

void sm_start_tx(void)
{
	int err;
	static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
		0x01, 0x00, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08);


	LOG_INF("tx test INF");
	LOG_DBG("tx test DBG");
	LOG_INF("Enhanced ShockBurst ptx sample");

	err = clocks_start();
	if (err) {
		return;
	}

	err = esb_initialize(ESB_MODE_PTX);
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return;
	}

	LOG_INF("Initialization complete");
	LOG_INF("Sending test packet");

	tx_payload.noack = false;
	k_sleep(K_MSEC(300));
	int tx_count = 0;
	while (1) {
		LOG_INF("looping");
		char message[20];
		sprintf(message,"tx loop %d",tx_count);
		mesh_bcast_text(message);

		tx_count++;
		k_sleep(K_MSEC(500));
	}
}
