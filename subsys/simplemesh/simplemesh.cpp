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
bool rx_routing_handler(message_t &msg);
void rx_gateway_handler(message_t &msg);
void rx_json_handler(message_t &msg);
void rx_file_header_handler(uint8_t * data,uint8_t size);
void rx_file_section_handler(message_t &section_msg);

//----------------------------- startup init -----------------------------
int esb_initialize();

//----------------------------- protocol specific hanlding -----------------------------
uint8_t coord_assign_short_id(std::string &longid);
void mesh_set_node_id(std::string &longid, uint8_t shortid);
bool mesh_request_node_id();
bool take_node_id(message_t &msg,uint8_t &nodeid);

static mesh_rx_json_handler_t m_app_rx_json_handler = NULL;
json request;
bool critical_parse=false;

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
file_t file_data;

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
static std::string base_topic = "sm";
static std::string broadcast_topic_start = base_topic + "{";
static std::string self_topic;


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

void mesh_bcast_packet(sm::pid pid,uint8_t * data,uint8_t size)
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

void mesh_send_packet(sm::pid pid,uint8_t dest,uint8_t * data,uint8_t size)
{
	tx_msg.control = sm::control::msg_no_ack | g_ttl;
	tx_msg.pid     = (uint8_t)(pid);
	tx_msg.source  = g_node_id;
	tx_msg.dest  = dest;
	tx_msg.payload = data;
	tx_msg.payload_length = size;
	mesh_tx_message(&tx_msg);
}

bool mesh_send_packet_ack(sm::pid pid,uint8_t dest, uint8_t* data, uint8_t size)
{
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
			break;
		}
		printf("sem_ack_error:timedout (%d)\n",i);
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
int64_t sm_rx_sync_ms(int lseq,int64_t delay)
{
	int64_t target_ticks_delay = rx_timestamp + k_ms_to_ticks_floor64(delay);
	if(target_ticks_delay < k_uptime_ticks()){
		int64_t delta = k_uptime_ticks() - target_ticks_delay;
		printf("sm> /!\\ target_ticks_delay missed by (%d) us at seq(%d)\n", (int)k_ticks_to_us_floor64(delta),lseq);
	}
	while((target_ticks_delay) > k_uptime_ticks());
	return target_ticks_delay;
}

int64_t sm_sync_ms(int lseq,int64_t start,int64_t delay)
{
	int64_t target_ticks_delay = start + k_ms_to_ticks_floor64(delay);
	if(target_ticks_delay < k_uptime_ticks()){
		int64_t delta = k_uptime_ticks() - target_ticks_delay;
		printf("sm> /!\\ target_ticks_delay missed by (%d) us at seq(%d)\n", (int)k_ticks_to_us_floor64(delta),lseq);
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
				if(rx_routing_handler(rx_msg))//id get set / ack
				{
					//handled in function call
				}else if(m_app_rx_json_handler != NULL)//for sm::pid::json
				{
					rx_json_handler(rx_msg);
				}
				else{
					rx_gateway_handler(rx_msg);
				}
			}
		}
	}
}

void rx_json_handler(message_t &msg)
{
	if(msg.pid == (uint8_t)(sm::pid::json)){
		std::string payload((char*)msg.payload,msg.payload_length);
		if(is_broadcast(payload) || is_self(payload)){
			size_t json_begin = payload.find("{");
			std::string topic = payload.substr(0,json_begin);
			std::string json_body = payload.substr(json_begin);
			critical_parse = true;//exceptions config not supported, ends in libc-hooks.c "exit\n"
			#ifdef CONFIG_EXCEPTIONS
				try{
					request = json::parse(json_body);
				}catch(json::parse_error& ex){
					printf("json::parse threw an exception at byte %d\n",ex.byte);
				}
			#else
				request = json::parse(json_body);
			#endif
			critical_parse = false;
			m_app_rx_json_handler(msg.source,topic,request);
		}
	}
}

void rx_gateway_handler(message_t &rx_msg)
{
	if(rx_msg.pid == (uint8_t)(sm::pid::text))
	{
		std::string text((char*)rx_msg.payload,rx_msg.payload_length);
		printf("%s\n",text.c_str());
	//}else if(rx_msg.pid == (uint8_t)(sm::pid::node_id_get))
	//{
	//	std::string text((char*)rx_msg.payload,rx_msg.payload_length);
	//	printf("node_id_get:%s\n",text.c_str());
	//}
	//else if(rx_msg.pid == (uint8_t)(sm::pid::node_id_set))
	//{
	//	std::string text((char*)rx_msg.payload,rx_msg.payload_length);
	//	printf("node_id_set:%s\n",text.c_str());
	}else if(rx_msg.pid == (uint8_t)sm::pid::file_info){
		rx_file_header_handler(rx_msg.payload,rx_msg.payload_length);
	}else if(rx_msg.pid == (uint8_t)sm::pid::file_section){
		rx_file_section_handler(rx_msg);
	}
	else
	{
		printf("%d:{\"pid\":0x%02X,\"length\":%d}\n",rx_msg.source,rx_msg.pid, rx_msg.payload_length);
	}
}

void rx_file_header_handler(uint8_t * data,uint8_t size)
{
	std::string payload((char*)data,size);
	size_t json_begin = payload.find("{");
	std::string topic = payload.substr(0,json_begin);
	std::string json_body = payload.substr(json_begin);
	critical_parse = true;
	request = json::parse(json_body);
	critical_parse = false;
	file_data.name 			= request["name"];
	file_data.size 			= request["size"];
	file_data.seq_size 		= request["seq_size"];
	file_data.nb_seq   		= request["nb_seq"];
	file_data.last_seq_size = request["last_seq_size"];
	file_data.seq_count		= 0;
	file_data.buffer_p		= file_data.buffer;
}

void rx_file_section_handler(message_t &section_msg)
{
	uint8_t this_seq_size = file_data.seq_size;
	if(file_data.seq_count > file_data.nb_seq-1){
		printf("file_error;seq_count:%u;nb_seq:%u\n",file_data.seq_count,file_data.nb_seq);
		return;
	}else if(file_data.seq_count == file_data.nb_seq-1){
		this_seq_size = file_data.last_seq_size;
	}

	if(section_msg.payload_length != this_seq_size){
		printf("file_error;seq:%u;seq_size:%u;payload_length:%u\n",file_data.seq_count,this_seq_size,section_msg.payload_length);
		return;
	}

	memcpy(file_data.buffer_p,section_msg.payload,this_seq_size);
	file_data.buffer_p+=this_seq_size;
	file_data.seq_count++;

	if(file_data.seq_count == file_data.nb_seq){
		printf("uwb_cir_acc;size:%u;nb_seq:%u;data:",file_data.size,file_data.nb_seq);
		for(uint32_t i=0;i<file_data.size;i++){
			printf("%02X",file_data.buffer[i]);
		}
		printf("\n");
	}
}

bool rx_routing_handler(message_t &msg)
{
	LOG_DBG("dest(%d) ; self(%d) ; pid(%d) ; ctl(%d)",msg.dest, g_node_id, msg.pid, msg.control);
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
			return true;
        }else if(msg.pid == (uint8_t)sm::pid::node_id_set){
			LOG_DBG("received set node it");
			bool taken = take_node_id(msg,g_node_id);
			if(taken){
				k_sem_give(&sem_id_set);//unleash the higher prio waiting for 'g_node_id'
				json j;
				j["rf_cmd"] = "sid";
				j["sid"] = g_node_id;
				mesh_bcast_json(j);
				mesh_bcast_json(j);
			}
			return true;
        }
	}
    else if(msg.dest == g_node_id){
		LOG_DBG("dest == self (%d) for pid (%d)",g_node_id,msg.pid);
        if(MESH_WANT_ACKNOWLEDGE(msg.control)){
			LOG_DBG("sending acknowledge back to (%d)",msg.source);
            mesh_tx_ack(msg,g_ttl);
			return false;//still have to process the received message further
        }
        else if(MESH_IS_ACKNOWLEDGE(msg.control)){
			LOG_DBG("acknowledge received from (%d)",msg.source);
            k_sem_give(&sem_ack);
			return true;
        }
    }
    //only re-route messaegs directed to other than the current node itself
    else if(g_node_is_router)
    {
        //mesh_forward_message(msg);//a mesh forward is destructive, to be done at last step
    }
	return false;
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
	self_topic = base_topic + "/" + g_node_uid;

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
			j["rf_cmd"] = "sid";
			j["sid"] = g_node_id;
			mesh_bcast_json(j);
		}else{
			printf("sm_start> waiting for coordinator, short id [%d]\n",g_node_id);
		}
	}

}

void sm_set_callback_rx_json(mesh_rx_json_handler_t rx_json_handler)
{
	m_app_rx_json_handler = rx_json_handler;
}


//----------------------------- protocol specific hanlding -----------------------------
//self_uid -> dest
void mesh_send_file_info(uint8_t dest_id, json &data)
{
	std::string message = sm_get_topic() + data.dump();
	mesh_send_packet_ack(sm::pid::file_info,dest_id,(uint8_t*)message.c_str(),message.length());
}

//blocking unlimited size
void mesh_send_file(const char * name, uint8_t dest,uint8_t* data, uint32_t size)
{
	uint8_t seq_size = sm::max_msg_size;
	uint32_t nb_seq   = (size / seq_size);
	uint8_t last_seq_size = sm::max_msg_size;
	if(size <= seq_size){
		nb_seq = 1;
		last_seq_size = size;
	}else if((size % seq_size) != 0){
		nb_seq++;
		last_seq_size = size % seq_size;
	}

	json info;
	info["name"] 			= name;
	info["size"] 			= size;
	info["nb_seq"] 			= nb_seq;
	info["seq_size"] 		= seq_size;
	info["last_seq_size"] 	= last_seq_size;
	mesh_send_file_info(dest,info);

	for(uint32_t i=0;i<nb_seq;i++){
		if(i == nb_seq-1){//if last
			seq_size = last_seq_size;
		}
		if(!mesh_send_packet_ack(sm::pid::file_section,dest,data,seq_size)){
			LOG_ERR("mesh_tx_file_ack() failed");
			continue;
		}
		if(i != nb_seq-1){//if not lat
			data += seq_size;
		}
	}
}

//--json
//self_uid -> bcast
void mesh_bcast_json(json &data)
{
	std::string message = sm_get_topic() + data.dump();
	mesh_bcast_packet(sm::pid::text,(uint8_t*)message.c_str(),message.length());
}

//target_uid -> bcast
void mesh_bcast_json_to(json &data,std::string &target)
{
	std::string message = sm_get_base_topic() + "/" + target + data.dump();
	mesh_bcast_packet(sm::pid::text,(uint8_t*)message.c_str(),message.length());
}

//self_uid -> dest
void mesh_send_json(uint8_t dest_id, json &data,bool ack = false)
{
	std::string message = sm_get_topic() + data.dump();
	if(ack){
		mesh_send_packet_ack(sm::pid::json,dest_id,(uint8_t*)message.c_str(),message.length());
	}else{
		mesh_send_packet(sm::pid::json,dest_id,(uint8_t*)message.c_str(),message.length());
	}
}

//--------------------------------- coordinator management functions ---------------------------------

bool take_node_id(message_t &msg,uint8_t &nodeid)
{
	std::string uid = sm_get_uid();
	uint8_t len_node_id = uid.length();
	if(msg.payload_length != len_node_id+3){
		LOG_ERR("unexpected set node id length %d instead of %d",msg.payload_length,len_node_id+3);
		return false;
	}
	std::string this_node_id_str((char*)msg.payload,len_node_id);
	if(this_node_id_str.compare(uid) != 0){
		LOG_DBG("not for this node id : uid[%s] received[%s]\n",uid.c_str(),this_node_id_str.c_str());
		return false;
	}

	sscanf((char*)(msg.payload+len_node_id+1),"%2hhx",&nodeid);
	return true;
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

bool is_self(std::string &payload)
{
	return (payload.rfind(self_topic,0) == 0);
}

bool is_broadcast(std::string &payload)
{
	return (payload.rfind(broadcast_topic_start,0) == 0);
}

std::string sm_get_base_topic()
{
	return base_topic;
}

std::string sm_get_topic()
{
	return self_topic;
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
		rf_cmd_response["sid"] = g_node_id;
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
		rf_cmd_response["sid"] = g_node_id;
		rf_cmd_response["time"] = rx_timestamp;
		mesh_bcast_json(rf_cmd_response);
		printf("sm> pinger ; rssi=-%d dBm; time = %d (1/%d ms)\n",rx_msg.rssi, (int)rx_timestamp,k_ms_to_ticks_floor32(1));
	}else if(rf_cmd.compare("sid") == 0){
		if(data.contains("sid")){
			if((g_node_id != 0) && (data["sid"] != 0)){
				g_node_id = data["sid"];
			}else{
				printf("sm> change of coordinator short id 0 not supported\n");
			}
		}
		json rf_cmd_response;
		rf_cmd_response["rf_cmd"] = "sid";
		rf_cmd_response["sid"] = g_node_id;
		mesh_bcast_json(rf_cmd_response);
		printf("sm> short id = %d\n",g_node_id);
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

