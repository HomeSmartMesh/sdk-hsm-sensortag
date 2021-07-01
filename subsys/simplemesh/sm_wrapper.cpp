#include <zephyr.h>
#include <stdio.h>
#include "simplemesh.h"

static mesh_rx_json_handler_t m_app_rx_json_handler = NULL;
std::string base_topic = "sm/";
std::string self_topic;

std::string sm_get_uid()
{
	char text[20];
	sprintf(text,"%08lX%08lX",(long unsigned int)NRF_FICR->DEVICEID[0],(long unsigned int)NRF_FICR->DEVICEID[1]);
	return std::string(text);
}

std::string sm_get_topic()
{
	return base_topic + sm_get_uid();
}

bool is_self(std::string &payload)
{
	return (payload.rfind(self_topic)!=std::string::npos);
}

void mesh_rx_handler(message_t* msg)
{
	if(msg->pid == (uint8_t)(sm::pid::text)){
		msg->payload[msg->payload_length] = '\0';
		std::string payload = (char*)msg->payload;
		if(is_self(payload)){
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

void mesh_send_json(json &data,uint8_t node_id)
{
	std::string message = self_topic + data.dump();
	mesh_send_data(sm::pid::text,(uint8_t)node_id,(uint8_t*)message.c_str(),message.length());
}
