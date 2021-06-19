#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>
#include <simplemesh.h>
#include <json.hpp>
#include <string>

using json = nlohmann::json;

std::string uid;
std::string self_topic;

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

std::string get_uid()
{
	char text[25];
	sprintf(text,"%04lX%04lX",(long unsigned int)NRF_FICR->DEVICEID[0],(long unsigned int)NRF_FICR->DEVICEID[1]);
	return std::string(text);
}

bool is_self(std::string &payload)
{
	return (payload.rfind(self_topic)!=std::string::npos);
}

void topic_json_handler(std::string &topic, json &data)
{
	printk("topic = %s ; json :\n",topic.c_str());
	printk("%s\n",data.dump(2).c_str());
}

void rx_handler(message_t* msg)
{
	if(msg->pid == Mesh_Pid_Text){
		msg->payload[msg->payload_length] = '\0';
		std::string payload = (char*)msg->payload;
		if(is_self(payload)){
			size_t json_begin = payload.find("{");
			std::string topic = payload.substr(0,json_begin);
			std::string json_body = payload.substr(json_begin);
			json request = json::parse(json_body);
			topic_json_handler(topic,request);
		}else{
			printk("(%s) not for self\n",(char*)msg->payload);
		}
	}
}

void init_uid()
{
	uid = get_uid();
	self_topic = "sm/" + uid;
}

void main(void)
{
	#ifdef CONFIG_USB
		int ret;
		ret = usb_enable(NULL);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB");
			return;
		}
	#endif
	init_uid();

	sm_start(rx_handler);

	printk("Hello Simple from DWM1001 UID (%s)",uid.c_str());

	int loop = 0;
	while (1) {
		char message[50];
		sprintf(message,"sm/%s{\"alive\":%d}",uid.c_str(),loop);
		mesh_bcast_text(message);
		printk("%s\n",message);
		k_sleep(K_SECONDS(5));
		loop++;
	}
}
