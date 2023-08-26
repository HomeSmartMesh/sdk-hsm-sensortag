
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/openthread.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>

#include <stdio.h>

#include <sensor/veml6030.h>
#include <sensor/ms8607.h>

#include <openthread/thread.h>
#include <openthread/coap.h>
#include <openthread/srp_client.h>

#include "battery.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

char device_id[20];

const struct device *light_dev = DEVICE_DT_GET_ONE(vishay_veml6030);

static otCoapResource battery_resource = {
	.mUriPath = "battery",
	.mHandler = NULL,
	.mContext = NULL,
	.mNext = NULL,
};

static otCoapResource veml6030_resource = {
	.mUriPath = "veml6030",
	.mHandler = NULL,
	.mContext = NULL,
	.mNext = NULL,
};

static otCoapResource ms8607_resource = {
	.mUriPath = "ms8607",
	.mHandler = NULL,
	.mContext = NULL,
	.mNext = NULL,
};

static otSrpClientService sensortag_service = {
    .mInstanceName = "",
    .mName = "_sensortag._udp",
    .mPort = 5683,
};

void report_sensors(int count,bool send){
	battery_start();
	k_sleep(K_MSEC(10));
	int32_t voltage_mv = battery_get_mv();
	float voltage = voltage_mv;
	voltage /= 1000;
	float light = veml6030_auto_measure(light_dev);
	float t, p, h;
	enum ms8607_status status = ms8607_read_temperature_pressure_humidity(&t, &p, &h);
	if (status == ms8607_status_ok) {
        LOG_DBG("ms8607> t=%.2f h=.%2f p=%.2f", t, p, h);
    } else {
		LOG_ERR("ms8607> status = %d", status);
	}

	char message[250];
	int size = sprintf(message,"thread_tags/%s{\"alive\":%d,\"voltage\":%.3f,\"light\":%0.3f,\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f}",
								device_id, count, voltage, light, t, h, p);
	LOG_INF("%s",message);
}

static void battery_handler(void *context, otMessage *message, const otMessageInfo *message_info)
{
    otMessage *responseMessage = otCoapNewMessage(context, NULL);

    if (otCoapMessageGetCode(message) != OT_COAP_CODE_GET) {
		LOG_ERR("Battery handler - Unexpected CoAP code");
        otCoapMessageInitResponse(responseMessage,
                message,
                OT_COAP_TYPE_ACKNOWLEDGMENT,
                OT_COAP_CODE_METHOD_NOT_ALLOWED);
        otCoapSendResponse(context, responseMessage, message_info);
        return;
	}

	int32_t voltage_mv = battery_get_mv();
	float voltage = voltage_mv/1000.0;
	char content[100];
	int n = sprintf(content,"{\"device_id\":\"%s\", \"voltage\":%.3f}", device_id, voltage);

    // send response message
    otCoapMessageInitResponse(
            responseMessage,
            message,
            otCoapMessageGetType(message) == OT_COAP_TYPE_CONFIRMABLE
                ? OT_COAP_TYPE_ACKNOWLEDGMENT
                : OT_COAP_TYPE_NON_CONFIRMABLE,
            OT_COAP_CODE_CONTENT);
    otCoapMessageAppendContentFormatOption(responseMessage, OT_COAP_OPTION_CONTENT_FORMAT_JSON);
    otCoapMessageSetPayloadMarker(responseMessage);
    otMessageAppend(responseMessage, content, n);
}

static void veml6030_handler(void *context, otMessage *message, const otMessageInfo *message_info)
{
    otMessage *responseMessage = otCoapNewMessage(context, NULL);

    if (otCoapMessageGetCode(message) != OT_COAP_CODE_GET) {
		LOG_ERR("Battery handler - Unexpected CoAP code");
        otCoapMessageInitResponse(responseMessage,
                message,
                OT_COAP_TYPE_ACKNOWLEDGMENT,
                OT_COAP_CODE_METHOD_NOT_ALLOWED);
        otCoapSendResponse(context, responseMessage, message_info);
        return;
	}

	float light = veml6030_auto_measure(light_dev);
	char content[100];
	int n = sprintf(content,"{\"device_id\":\"%s\", \"light\":%.3f}", device_id, light);

    // send response message
    otCoapMessageInitResponse(
            responseMessage,
            message,
            otCoapMessageGetType(message) == OT_COAP_TYPE_CONFIRMABLE
                ? OT_COAP_TYPE_ACKNOWLEDGMENT
                : OT_COAP_TYPE_NON_CONFIRMABLE,
            OT_COAP_CODE_CONTENT);
    otCoapMessageAppendContentFormatOption(responseMessage, OT_COAP_OPTION_CONTENT_FORMAT_JSON);
    otCoapMessageSetPayloadMarker(responseMessage);
    otMessageAppend(responseMessage, content, n);
}

static void ms8607_handler(void *context, otMessage *message, const otMessageInfo *message_info)
{
    otMessage *responseMessage = otCoapNewMessage(context, NULL);

    if (otCoapMessageGetCode(message) != OT_COAP_CODE_GET) {
		LOG_ERR("Battery handler - Unexpected CoAP code");
        otCoapMessageInitResponse(responseMessage,
                message,
                OT_COAP_TYPE_ACKNOWLEDGMENT,
                OT_COAP_CODE_METHOD_NOT_ALLOWED);
        otCoapSendResponse(context, responseMessage, message_info);
        return;
	}

	float t, p, h;
	enum ms8607_status status = ms8607_read_temperature_pressure_humidity(&t, &p, &h);
	char content[100];
	int n = sprintf(content,"{\"device_id\":\"%s\", \"temperature\":%.3f, \"pressure\":\"\", \"humidity\":\"%.3f\"}", device_id, t, p, h);

    // send response message
    otCoapMessageInitResponse(
            responseMessage,
            message,
            otCoapMessageGetType(message) == OT_COAP_TYPE_CONFIRMABLE
                ? OT_COAP_TYPE_ACKNOWLEDGMENT
                : OT_COAP_TYPE_NON_CONFIRMABLE,
            OT_COAP_CODE_CONTENT);
    otCoapMessageAppendContentFormatOption(responseMessage, OT_COAP_OPTION_CONTENT_FORMAT_JSON);
    otCoapMessageSetPayloadMarker(responseMessage);
    otMessageAppend(responseMessage, content, n);
}

int coap_init(otInstance *instance)
{
	battery_resource.mContext = instance;
	battery_resource.mHandler = battery_handler;
	otCoapAddResource(instance, &battery_resource);

	veml6030_resource.mContext = instance;
	veml6030_resource.mHandler = veml6030_handler;
	otCoapAddResource(instance, &veml6030_resource);

	ms8607_resource.mContext = instance;
	ms8607_resource.mHandler = ms8607_handler;
	otCoapAddResource(instance, &ms8607_resource);
}

void main(void)
{
	battery_init();
	if (ms8607_is_connected()) {
		LOG_INF("ms8607> connected");
	} else {
		LOG_ERR("ms8607> not connected");
	}
	battery_start();
	k_sleep(K_MSEC(10));

    otInstance* instance = openthread_get_default_instance();
    coap_init(instance);

	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	sprintf(device_id, "%04lX%04lX", id0, id1);
	LOG_INF("Device ID: %s", device_id);

    otSrpClientAddService(instance, &sensortag_service);

    otSrpClientSetHostName(instance, device_id);
    otSrpClientEnableAutoHostAddress(instance);
    otSrpClientEnableAutoStartMode(instance, NULL, NULL);
}
