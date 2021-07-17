#include <zephyr.h>
#include <stdio.h>
#include "simplemesh.h"

static mesh_rx_json_handler_t m_app_rx_json_handler = NULL;
json request;

void mesh_rx_handler(message_t* msg)
{
	if(msg->pid == (uint8_t)(sm::pid::text)){
		std::string payload((char*)msg->payload,msg->payload_length);
		if(is_broadcast(payload) || is_self(payload)){
			size_t json_begin = payload.find("{");
			std::string topic = payload.substr(0,json_begin);
			std::string json_body = payload.substr(json_begin);
			request = json::parse(json_body);
			m_app_rx_json_handler(topic,request);
		}
	}
}

void sm_set_callback_rx_json(mesh_rx_json_handler_t rx_json_handler)
{
	sm_set_callback_rx_message(mesh_rx_handler);
	m_app_rx_json_handler = rx_json_handler;
}

void mesh_bcast_string(std::string text)
{
	mesh_bcast_text(text.c_str());
}

void mesh_bcast_json(json &data)
{
	std::string message = sm_get_topic() + data.dump();
	mesh_bcast_text(message.c_str());
}

void mesh_bcast_json_to(json &data,std::string &target)
{
	std::string message = sm_get_base_topic() + "/" + target + data.dump();
	mesh_bcast_text(message.c_str());
}

void mesh_send_json(uint8_t dest_id, json &data)
{
	std::string message = sm_get_topic() + data.dump();
	mesh_send_text(dest_id,message);
}
