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

LOG_MODULE_REGISTER(simplemesh, LOG_LEVEL_DBG);

#ifdef CONFIG_SM_GPIO_DEBUG
	#include <drivers/gpio.h>

	const struct device *sm_gpio_dev;
	//0.625 us per toggle
	//#define PIN_SM_SET 		gpio_pin_set(sm_gpio_dev, CONFIG_SM_PIN_APP, 1)
	//#define PIN_SM_CLEAR 	gpio_pin_set(sm_gpio_dev, CONFIG_SM_PIN_APP, 0)
	#define PIN_SM_SET 		(*(int * const)0x50000508) 	= 1<<CONFIG_SM_PIN_APP
	#define PIN_SM_CLEAR 	(*(int *) 0x5000050C) 		= 1<<CONFIG_SM_PIN_APP

	void sm_gpio_init(const struct device *gpio_dev)
	{
		sm_gpio_dev = gpio_dev;
	}
#else
	#define PIN_SM_SET
	#define PIN_SM_CLEAR
#endif

//----------------------------- rx event, thread and handler -----------------------------
void simplemesh_rx_thread();
void mesh_rx_handler(message_t &msg);

//----------------------------- startup init -----------------------------
int esb_initialize();

//----------------------------- protocol specific hanlding -----------------------------
uint8_t coord_assign_short_id(std::string &longid);
void mesh_set_node_id(std::string &longid, uint8_t shortid);
bool mesh_request_node_id();
uint8_t take_node_id(message_t &msg);
//----------------------------- -------------------------- -----------------------------
#define STACKSIZE 8192
#define RX_PRIORITY 20

K_SEM_DEFINE(sem_rx, 0, 2);//can be assigned while already given
K_THREAD_DEFINE(sm_receiver, STACKSIZE, simplemesh_rx_thread, 
                NULL, NULL, NULL, RX_PRIORITY, 0, 0);

K_SEM_DEFINE(sem_id_set, 0, 1);
K_SEM_DEFINE(sem_ack, 0, 1);

static struct esb_payload tx_payload;
static struct esb_payload rx_payload;
static message_t rx_msg;
static message_t tx_msg;
int64_t rx_timestamp;

static mesh_rx_handler_t m_app_rx_handler = NULL;

static volatile bool esb_enabled = false;
static volatile bool esb_completed = false;
static volatile bool esb_tx_complete = false;
static uint8_t g_ttl = 2;
static uint8_t g_node_id = 0xff;
static uint8_t g_node_is_router = false;
static uint8_t g_retries = 3;
static uint16_t g_retries_timeout_ms = 200;
static uint16_t g_set_id_timeout_ms = 300;

static std::string g_node_uid = "";

#ifdef CONFIG_SM_LISTENER
	static uint8_t g_active_listener = true;
#else
	static uint8_t g_active_listener = false;
#endif
#if CONFIG_SM_COORDINATOR
	static uint8_t g_coordinator = true;
#endif
std::map<std::string,uint8_t> nodes_ids;

void mesh_pre_tx()
{
	if(!esb_enabled)
	{
		esb_initialize();
	}
	if(g_active_listener){
        esb_stop_rx();
	}
}

void mesh_post_tx()
{
	#ifdef CONFIG_SM_LISTENER
		if(g_active_listener){
			esb_start_rx();
			LOG_DBG("switch to RX mode");
		}
	#else
		esb_disable();
		esb_enabled = false;
    #endif
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

//does not copy, must keep context, use tx_msg
void mesh_tx_message(message_t* p_msg)
{
    mesh_pre_tx();

    mesh_message_2_esb_payload(p_msg,&tx_payload);

    esb_completed = false;//reset the check
    LOG_DBG("TX writing (%d) %d bytes",tx_payload.data[2],tx_payload.data[0]);
    //should not wait for esb_completed here as does not work from ISR context

    //NRF_ESB_TXMODE_AUTO is used no need to call nrf_esb_start_tx()
    //which could be used for a precise time transmission
    esb_write_payload(&tx_payload);
    
}

void mesh_tx_ack(message_t& msg, uint8_t ttl)
{
    tx_msg.control = sm::control::ack | ttl;
    tx_msg.pid     = msg.pid;
    tx_msg.source  = msg.dest;
    tx_msg.dest    = msg.source;
    tx_msg.payload = nullptr;
    tx_msg.payload_length = 0;

    mesh_tx_message(&tx_msg);
}
//limited to 255
void mesh_bcast_data(sm::pid pid,uint8_t * data,uint8_t size)
{
	if(size > sm::max_msg_size)
	{
		return;
	}
    tx_msg.control = sm::control::broadcast | g_ttl;
    tx_msg.pid     = (uint8_t)pid;
    tx_msg.source  = g_node_id;
    tx_msg.payload = data;
    tx_msg.payload_length = size;

    mesh_tx_message(&tx_msg);
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
	if(size > sm::max_msg_size)
	{
		return false;
	}
    tx_msg.control = sm::control::msg_needs_ack | g_ttl;
    tx_msg.pid     = (uint8_t)pid;
    tx_msg.source  = g_node_id;
    tx_msg.dest  = dest;
    tx_msg.payload = data;
    tx_msg.payload_length = size;

	bool success = false;
	for(uint8_t i=0;i<g_retries;i++){
		mesh_tx_message(&tx_msg);
		if(k_sem_take(&sem_ack,K_MSEC(g_retries_timeout_ms)) == 0){
			LOG_DBG("sem_ack obtained");
			success = true;
			continue;
		}
		LOG_DBG("sem_ack timedout");
	}

	return success;
}

//----------------------------- rx event, thread and handler -----------------------------

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS pid(%d)",tx_msg.pid);
        mesh_post_tx();
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED pid(%d)",tx_msg.pid);
		(void) esb_flush_tx();
		mesh_post_tx();
		break;
	case ESB_EVENT_RX_RECEIVED://not yet parsed in rx_msg
		rx_timestamp = k_uptime_ticks();
		LOG_DBG("RX Event");
		k_sem_give(&sem_rx);
		break;
	default:
		LOG_ERR("ESB Unhandled Event (%d)",event->evt_id);
		break;
	}
	esb_completed = true;
}

//thread jitter guard
int64_t sm_rx_sync_ms(int64_t delay)
{
	int64_t target_ticks_delay = rx_timestamp + k_ms_to_ticks_floor64(delay);
	if(target_ticks_delay < k_uptime_ticks()){
		int64_t delta = k_uptime_ticks() - target_ticks_delay;
		printf("sm> /!\\ target_ticks_delay missed by (%d) us\n", (int)k_ticks_to_us_floor64(delta));
	}
	while((target_ticks_delay) > k_uptime_ticks());
	return target_ticks_delay;
}

int64_t sm_sync_ms(int64_t start,int64_t delay)
{
	int64_t target_ticks_delay = start + k_ms_to_ticks_floor64(delay);
	if(target_ticks_delay < k_uptime_ticks()){
		int64_t delta = k_uptime_ticks() - target_ticks_delay;
		printf("sm> /!\\ target_ticks_delay missed by (%d) us\n", (int)k_ticks_to_us_floor64(delta));
	}
	while((target_ticks_delay) > k_uptime_ticks());
	return target_ticks_delay;
}

void simplemesh_rx_thread()
{
	while(true)
	{
		if(k_sem_take(&sem_rx,K_MSEC(100)) == 0){
			while(esb_read_rx_payload(&rx_payload) == 0)
			{
				mesh_esb_2_message_payload(&rx_payload,&rx_msg);
				LOG_DBG("RX pid(%u) size(%u)",rx_msg.pid,rx_msg.payload_length);
				//-------------App-------------
				if(m_app_rx_handler != NULL)//app gets all
				{
					m_app_rx_handler(&rx_msg);
				}
				//-------------Dongle-------------
				else if(rx_msg.pid == (uint8_t)(sm::pid::text))
				{
					std::string text((char*)rx_msg.payload,rx_msg.payload_length);
					printf("%s\n",text.c_str());
				}
				else if(rx_msg.pid == (uint8_t)(sm::pid::node_id_get))
				{
					std::string text((char*)rx_msg.payload,rx_msg.payload_length);
					printf("node_id_get:%s\n",text.c_str());
				}
				else if(rx_msg.pid == (uint8_t)(sm::pid::node_id_set))
				{
					std::string text((char*)rx_msg.payload,rx_msg.payload_length);
					printf("node_id_set:%s\n",text.c_str());
				}
				else
				{
					printf("%d:{\"pid\":0x%02X,\"length\":%d}",rx_msg.source,rx_msg.pid, rx_msg.payload_length);
				}
				//-------------Routing-------------
				mesh_rx_handler(rx_msg);
				PIN_SM_SET;
				k_busy_wait(50);
				PIN_SM_CLEAR;
			}
		}
	}
}

void mesh_rx_handler(message_t &msg)
{
    if(MESH_IS_BROADCAST(msg.control)){
        if(msg.pid == (uint8_t)sm::pid::node_id_get){
			#ifdef CONFIG_SM_COORDINATOR
			printf("node_id_get\n");
			if(g_coordinator){
				LOG_DBG("coordinator received get node id");
				std::string longid((char*)rx_msg.payload,rx_msg.payload_length);
				uint8_t newid = coord_assign_short_id(longid);
				mesh_set_node_id(longid,newid);
			}else{
				LOG_DBG("get node id ignored, not coordinator");
			}
			#endif
        }else if(msg.pid == (uint8_t)sm::pid::node_id_set){
			#ifdef CONFIG_SM_SNIFFER
			//ignore as a sniffer
			#else
			LOG_DBG("received set node it");
			g_node_id = take_node_id(msg);//on error no effect with old id
			k_sem_give(&sem_id_set);//unleash the higher prio waiting for 'g_node_id'
			json j;
			j["shortid"] = g_node_id;
			mesh_bcast_json(j);
			#endif
        }
	}
    else if(msg.dest == g_node_id){
		LOG_DBG("dest == self (%d) for pid (%d)",g_node_id,msg.pid);
        if(MESH_WANT_ACKNOWLEDGE(msg.control)){
			LOG_DBG("sending acknowledge back to (%d)",msg.source);
            mesh_tx_ack(msg,g_ttl);
        }
        else if(MESH_IS_ACKNOWLEDGE(msg.control)){
			LOG_DBG("acknowledge received from (%d)",msg.source);
            k_sem_give(&sem_ack);
        }
    }
    //only re-route messaegs directed to other than the current node itself
    else if(g_node_is_router)
    {
        //mesh_forward_message(msg);//a mesh forward is destructive, to be done at last step
    }
}

//----------------------------- startup init -----------------------------
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
	
	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	char uid_text[20];
	int str_len = sprintf(uid_text,"%08lX%08lX",id0,id1);
	g_node_uid = std::string(uid_text,str_len);

	#if CONFIG_SM_SNIFFER
		LOG_DBG("sniffer does not need a nodeid");
	#else
		LOG_DBG("always start by requesting short id from [0]");
		if(mesh_request_node_id()){//with broadcast retries and sem waiting for 'set id'
			if(g_coordinator){
				g_coordinator = false;//request node id got acknowledged
				printf("sm_start> no longer coordinator\n");
			}
		}else{
			if(g_coordinator){
				std::string uid = sm_get_uid();
				g_node_id = 0;
				nodes_ids[uid] = g_node_id;
				printf("sm_start> [%s] acting as coordinator with short id [0]\n",uid.c_str());
				json j;
				j["shortid"] = g_node_id;
				mesh_bcast_json(j);
			}else{
				printf("sm_start> waiting for coordinator, short id [%d]\n",g_node_id);
			}
		}
	#endif

}

void sm_set_callback_rx_message(mesh_rx_handler_t rx_handler)
{
    m_app_rx_handler = rx_handler;
}

//----------------------------- protocol specific hanlding -----------------------------

void mesh_send_packet(sm::pid pid,uint8_t dest,uint8_t * data,uint32_t size)
{
	tx_msg.control = 0x80 | g_ttl;         // broadcast | ttl = g_ttl
	tx_msg.pid     = (uint8_t)(pid);
	tx_msg.source  = 0xFF;//TODO config node id
	tx_msg.payload = data;
	tx_msg.payload_length = size;
	mesh_tx_message(&tx_msg);
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

uint8_t take_node_id(message_t &msg)
{
	std::string uid = sm_get_uid();
	uint8_t len_node_id = uid.length();
	if(msg.payload_length != len_node_id+3){
		LOG_ERR("unexpected set node id length %d instead of %d",msg.payload_length,len_node_id+3);
		return g_node_id;
	}
	std::string this_node_id_str((char*)msg.payload,len_node_id);
	if(this_node_id_str.compare(uid) != 0){
		LOG_ERR("uid mismatch");
		printf("sm> Error : uid[%s] received[%s]\n",uid.c_str(),this_node_id_str.c_str());
		return g_node_id;
	}

	uint8_t new_node_id;
	sscanf((char*)(msg.payload+len_node_id+1),"%2hhx",&new_node_id);
	return new_node_id;
}

//a node id is needed before it's possible to send and receive directed messages
bool mesh_request_node_id()
{

    tx_msg.control = sm::control::broadcast | g_ttl;
    tx_msg.pid     = (uint8_t)sm::pid::node_id_get;
    tx_msg.source  = g_node_id;
    tx_msg.dest  = 0;

	std::string uid = sm_get_uid();
    tx_msg.payload = (uint8_t*)uid.c_str();
    tx_msg.payload_length = uid.length();

	bool success = false;
	for(uint8_t i=0;i<g_retries;i++){
		mesh_tx_message(&tx_msg);
		if(k_sem_take(&sem_id_set,K_MSEC(g_set_id_timeout_ms)) == 0){//result sscanf in g_node_id
			printf("sm> obtained node id set to [%u]\n",g_node_id);
			return true;
		}else{
			printf("sm> sem_id_set timeout\n");
		}
	}
	printf("sm> id request retries out\n");

	return success;
}

void mesh_set_node_id(std::string &longid, uint8_t shortid)
{
	tx_msg.control = sm::control::broadcast | 2;//Peer to peer with 1 time to live
	tx_msg.pid = (uint8_t)(sm::pid::node_id_set);
	tx_msg.source = g_node_id;//does not matter
	char shortid_char[3];
	sprintf(shortid_char,"%02x",shortid);
	std::string shortid_str(shortid_char,2);
	std::string response = longid + ":" +shortid_str;
	tx_msg.payload = (uint8_t*)response.c_str();
	tx_msg.payload_length = response.length();
	mesh_tx_message(&tx_msg);
	printf("sm> assigned node id (%u) to [%s]\n",shortid,longid.c_str());
}

uint8_t coord_assign_short_id(std::string &longid)
{
	if(nodes_ids.find(longid) != nodes_ids.end()){//remote node restart proof, keeps same id
		return nodes_ids[longid];
	}else{
		uint8_t newid = nodes_ids.size();//((size-1):index+1)
		nodes_ids[longid] = newid;
		return newid;
	}
}

std::string sm_get_uid()
{
	return g_node_uid;
}

uint8_t sm_get_sid()
{
	return g_node_id;
}

void sm_diag(json &data)
{
	std::string rf_cmd = data["rf_cmd"];
	if(rf_cmd.compare("ping") == 0){
		json rf_cmd_response;
		rf_cmd_response["rf_cmd"] = "pong";
		rf_cmd_response["rssi"] = rx_msg.rssi;
		rf_cmd_response["time"] = rx_timestamp;
		mesh_bcast_json(rf_cmd_response);
		printf("sm> ping -> pong ; rssi=-%d dBm; time = %d (1/%d ms)\n",rx_msg.rssi, (int)rx_timestamp,k_ms_to_ticks_floor32(1));
	}else if(rf_cmd.compare("target_ping") == 0){
		//Forward the request to the target
		json ping_request;
		ping_request["rf_cmd"] = "ping";
		std::string target = data["target"];
		mesh_bcast_json_to(ping_request, target);
		printf("sm> ping ; target=%s\n",target.c_str());
		//then repsond for self diag
		json rf_cmd_response;
		rf_cmd_response["rf_cmd"] = "pinger";
		rf_cmd_response["rssi"] = rx_msg.rssi;
		rf_cmd_response["time"] = rx_timestamp;
		mesh_bcast_json(rf_cmd_response);
		printf("sm> pinger ; rssi=-%d dBm; time = %d (1/%d ms)\n",rx_msg.rssi, (int)rx_timestamp,k_ms_to_ticks_floor32(1));
	}
}


void sm_start_rx()
{
	g_active_listener = true;
	esb_start_rx();
}

void sm_stop_rx()
{
	g_active_listener = false;
	esb_stop_rx();
}
