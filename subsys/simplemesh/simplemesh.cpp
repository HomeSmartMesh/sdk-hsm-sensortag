/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
extern "C"{
	#include <drivers/clock_control.h>
	#include <drivers/clock_control/nrf_clock_control.h>
}

#include <irq.h>
#include <logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <stdio.h>

#include "simplemesh.h"

LOG_MODULE_REGISTER(simplemesh, LOG_LEVEL_INF);

#define STACKSIZE 4096
#define RX_PRIORITY 90
#define TX_PRIORITY 80

#define MAX_MESH_FILE_SIZE 8096
uint8_t rx_file_buffer[MAX_MESH_FILE_SIZE];
uint8_t tx_file_buffer[MAX_MESH_FILE_SIZE];

int esb_initialize();
void simplemesh_rx_thread();
void mesh_send_data(sm::pid pid,uint8_t dest,uint8_t * data,uint8_t size);
uint8_t mesh_assign_short_id(std::string &longid);
void mesh_set_node_id(std::string &longid, uint8_t shortid);
bool mesh_request_node_id();

K_SEM_DEFINE(sem_rx, 0, 1);
K_THREAD_DEFINE(sm_receiver, STACKSIZE, simplemesh_rx_thread, 
                NULL, NULL, NULL, RX_PRIORITY, 0, 0);

K_SEM_DEFINE(sem_id_set, 0, 1);
K_SEM_DEFINE(sem_ack, 0, 1);

static struct esb_payload tx_payload;
static struct esb_payload rx_payload;
static message_t rx_msg;

static mesh_rx_handler_t m_app_rx_handler = NULL;

static volatile bool esb_enabled = false;
static volatile bool esb_completed = false;
static volatile bool esb_tx_complete = false;
static uint8_t g_ttl = 2;
static uint8_t g_node_id = 0xff;
static uint8_t g_node_is_router = false;
static uint8_t g_retries = 3;
static uint8_t g_coordinator = true;

std::map<std::string,uint8_t> nodes_ids;

void mesh_pre_tx()
{
	if(!esb_enabled)
	{
		esb_initialize();
	}
	#ifdef CONFIG_SM_LISTENER
        esb_stop_rx();
        LOG_DBG("switch to IDLE mode that allows TX");
    #endif
}

void mesh_post_tx()
{
	#ifdef CONFIG_SM_LISTENER
        esb_start_rx();
        LOG_DBG("switch to RX mode");
	#else
	esb_disable();
	esb_enabled = false;
    #endif
	esb_tx_complete = true;
}

//this is completing the payload with an offset to avoid compying on a new buffer
void mesh_buffer_2_esb_payload(uint8_t *buffer,uint8_t offset,uint8_t size, struct esb_payload *p_tx_payload)
{
    memcpy( p_tx_payload->data+sm::p2p_header_length + offset,buffer,size);
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
        start_payload = sm::bcast_header_length;
    }
    else
    {
        p_tx_payload->data[4] = msg->dest;
        start_payload = sm::p2p_header_length;
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

void mesh_tx_ack(message_t& msg, uint8_t ttl)
{
    message_t ack;
    ack.control = sm::control::ack | ttl;
    ack.pid     = msg.pid;
    ack.source  = msg.dest;
    ack.dest    = msg.source;
    ack.payload = nullptr;
    ack.payload_length = 0;

    mesh_tx_message(&ack);
}

//limited to 255
void mesh_bcast_data(sm::pid pid,uint8_t * data,uint8_t size)
{
    message_t msg;
	if(size > sm::max_msg_size)
	{
		return;
	}
    msg.control = sm::control::broadcast | g_ttl;
    msg.pid     = (uint8_t)pid;
    msg.source  = g_node_id;
    msg.payload = data;
    msg.payload_length = size;

    mesh_tx_message(&msg);
}
//limited to 255
void mesh_bcast_text(const char *text)
{
    uint8_t size = strlen(text);
    if(size>sm::max_msg_size)//truncate in case of long message
    {
		LOG_ERR("message truncated at %d from %d", size,sm::max_msg_size);
        size = sm::max_msg_size;
    }
    mesh_bcast_data(sm::pid::text,(uint8_t*)text,size);
}

bool mesh_send_data_ack(sm::pid pid,uint8_t dest, uint8_t* data, uint8_t size)
{
    message_t msg;
	if(size > sm::max_msg_size)
	{
		return false;
	}
    msg.control = sm::control::msg_needs_ack | g_ttl;
    msg.pid     = (uint8_t)pid;
    msg.source  = g_node_id;
    msg.dest  = dest;
    msg.payload = data;
    msg.payload_length = size;

	bool success = false;
	for(uint8_t i=0;i<g_retries;i++){
		mesh_tx_message(&msg);
		if(k_sem_take(&sem_ack,K_MSEC(50)) == 0){
			success = true;
			continue;
		}
	}

	return success;
}

void take_node_id()
{
	std::string uid = sm_get_uid();
	uint8_t len_node_id = uid.length();
	if(rx_msg.payload_length != len_node_id+3){
		return;
	}
	std::string this_node_id_str((char*)rx_msg.payload,len_node_id);
	if(this_node_id_str.compare(uid) != 0){
		return;
	}

	sscanf((char*)(rx_msg.payload+len_node_id+1),"%2hhx",&g_node_id);
	k_sem_give(&sem_id_set);
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
		k_sem_give(&sem_rx);
		break;
	default:
		LOG_ERR("ESB Unhandled Event (%d)",event->evt_id);
		break;
	}
	esb_completed = true;
}

int esb_initialize()
{
	int err;
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.retransmit_delay = 600;
	config.bitrate = ESB_BITRATE_2MBPS;
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

	LOG_INF("setting channel (%d) RF Freq %d MHz",CONFIG_SM_CHANNEL,2400+CONFIG_SM_CHANNEL);
    err = esb_set_rf_channel(CONFIG_SM_CHANNEL);
	if (err) {
		return err;
	}

	#ifdef CONFIG_SM_LISTENER
		LOG_INF("Setting up for packet receiption");
		err = esb_start_rx();
		if (err) {
			LOG_ERR("RX setup failed, err %d", err);
			return err;
		}
	#endif

	esb_enabled = true;
	return 0;
}

void sm_start()
{
	int err;
	err = clocks_start();
	if (err) {
		return;
	}

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return;
	}
	LOG_INF("sm_start initialization complete");
	
	if(g_coordinator){
		if(mesh_send_data_ack(sm::pid::ping,0,nullptr,0)){//if there is another coordinator
			g_coordinator = false;
		}else{// I'm coordinator now
			std::string uid = sm_get_uid();
			g_node_id = 0;
			nodes_ids[uid] = g_node_id;
			printk("sm_start> '%s' acting as coordinator with short id 0\n",uid.c_str());
		}
	}
	if(!g_coordinator){
		if(mesh_request_node_id()){//with retries
			if(k_sem_take(&sem_id_set,K_MSEC(50)) == 0){
				printk("sm_start> obtained node id : %d\n",g_node_id);
			}else{
				printk("sm_start> waiting for node id set timedout\n");
			}
		}else{
			printk("sm_start> mesh_request_node_id() failed\n");
		}
	}

}

void sm_set_callback_rx_message(mesh_rx_handler_t rx_handler)
{
    m_app_rx_handler = rx_handler;
}

void mesh_rx_handler(message_t &msg)
{
    if(msg.dest == g_node_id)
    {
        if(MESH_WANT_ACKNOWLEDGE(msg.control)){
            mesh_tx_ack(msg,g_ttl);
        }
        else if(MESH_IS_ACKNOWLEDGE(msg.control)){
            k_sem_give(&sem_ack);
        }else if(msg.pid == (uint8_t)sm::pid::node_id_set){
			take_node_id();
        }else if(msg.pid == (uint8_t)sm::pid::node_id_get){
			if(g_coordinator){
				std::string longid((char*)rx_msg.payload,rx_msg.payload_length);
				uint8_t newid = mesh_assign_short_id(longid);
				mesh_set_node_id(longid,newid);
			}
		}
    }
    //only re-route messaegs directed to other than the current node itself
    else if(g_node_is_router)
    {
        //mesh_forward_message(msg);//a mesh forward is destructive, to be done at last step
    }
}

void simplemesh_rx_thread()
{
	while(true)
	{
		k_sem_take(&sem_rx, K_FOREVER);
		while(esb_read_rx_payload(&rx_payload) == 0)
		{
			mesh_esb_2_message_payload(&rx_payload,&rx_msg);
			//-------------App-------------
			if(m_app_rx_handler != NULL)//app can sniff or create a custom dongle
			{
				m_app_rx_handler(&rx_msg);
			}
			//-------------Dongle-------------
			else if(rx_msg.pid == (uint8_t)(sm::pid::text))
			{
				printk("%s\n",(char*)rx_msg.payload);
			}
			else
			{
				printk("%d:{\"pid\":0x%02X,\"length\":%d}",rx_msg.source,rx_msg.pid, rx_msg.payload_length);
			}
			//-------------Routing-------------
			mesh_rx_handler(rx_msg);
		}
	}
}

void mesh_send_packet(sm::pid pid,uint8_t dest,uint8_t * data,uint32_t size)
{
    message_t msg;
	msg.control = 0x80 | g_ttl;         // broadcast | ttl = g_ttl
	msg.pid     = (uint8_t)(pid);
	msg.source  = 0xFF;//TODO config node id
	msg.payload = data;
	msg.payload_length = size;
	mesh_tx_message(&msg);
}

//blocking unlimited size
void mesh_send_text(uint8_t dest,std::string &text)
{
	uint8_t* data = (uint8_t*)text.c_str();
	uint32_t size = text.length();
	uint8_t seq_size = sm::max_msg_size;
	uint32_t nb_seq   = (size / seq_size);
	uint8_t last_seq_size = sm::max_msg_size;
	if((size % seq_size) != 0){
		nb_seq++;
		last_seq_size = size % seq_size;
	}

	for(uint32_t i=0;i<nb_seq;i++){
		if(i == nb_seq-1){//if last
			seq_size = last_seq_size;
		}
		if(!mesh_send_data_ack(sm::pid::text,dest,data,seq_size)){
			LOG_ERR("mesh_tx_file_ack() failed");
			continue;
		}
		if(i != nb_seq-1){//if not lat
			data += seq_size;
		}
	}
}

//a node id is needed before it's possible to send and receive directed messages
bool mesh_request_node_id()
{
	std::string uid = sm_get_uid();
	return mesh_send_data_ack(sm::pid::node_id_get,0,(uint8_t*)uid.c_str(),uid.length());
}

void mesh_set_node_id(std::string &longid, uint8_t shortid)
{
    message_t msg;
	msg.control = sm::control::broadcast | 2;//Peer to peer with 1 time to live
	msg.pid = (uint8_t)(sm::pid::node_id_set);
	msg.source = g_node_id;//does not matter
	std::string uid = sm_get_uid();
	char shortid_char[3];
	sprintf(shortid_char,"%02x",shortid);
	std::string shortid_str(shortid_char,2);
	std::string response = uid + ":" +shortid_str;
	msg.payload = (uint8_t*)response.c_str();
	msg.payload_length = response.length();
	mesh_tx_message(&msg);
}

uint8_t mesh_assign_short_id(std::string &longid)
{
	if(nodes_ids.find(longid) != nodes_ids.end()){
		return nodes_ids[longid];
	}else{
		uint8_t newid = nodes_ids.size()+1;
		nodes_ids[longid] = newid;
		return newid;
	}
}
