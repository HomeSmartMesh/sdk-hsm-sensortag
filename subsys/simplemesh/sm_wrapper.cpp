#include <zephyr.h>
#include <stdio.h>
#include "simplemesh.h"

static mesh_rx_json_handler_t m_app_rx_json_handler = NULL;
std::string base_topic = "sm";
std::string broadcast_topic_start = base_topic + "{";
std::string self_topic;

std::string sm_get_uid()
{
	char uid_text[20];
	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	sprintf(uid_text,"%08lX%08lX",id0,id1);
	return std::string(uid_text);
}

std::string sm_get_topic()
{
	return base_topic + "/" + sm_get_uid();
}

bool is_self(std::string &payload)
{
	return (payload.rfind(self_topic,0) == 0);
}

bool is_broadcast(std::string &payload)
{
	return (payload.rfind(broadcast_topic_start,0) == 0);
}

void mesh_rx_handler(message_t* msg)
{
	if(msg->pid == (uint8_t)(sm::pid::text)){
		std::string payload((char*)msg->payload,msg->payload_length);
		if(is_broadcast(payload) || is_self(payload)){
			size_t json_begin = payload.find("{");
			std::string topic = payload.substr(0,json_begin);
			std::string json_body = payload.substr(json_begin);
			json request = json::parse(json_body);
			m_app_rx_json_handler(topic,request);
		}
	}
}

void sm_set_callback_rx_json(mesh_rx_json_handler_t rx_json_handler)
{
	sm_set_callback_rx_message(mesh_rx_handler);
	m_app_rx_json_handler = rx_json_handler;
	self_topic = sm_get_topic();
}

void mesh_bcast_string(std::string text)
{
	mesh_bcast_text(text.c_str());
}

void mesh_bcast_json(json &data)
{
	std::string message = self_topic + data.dump();
	mesh_bcast_text(message.c_str());
}

void mesh_send_json(uint8_t dest_id, json &data)
{
	std::string message = self_topic + data.dump();
	mesh_send_text(dest_id,message);
}
