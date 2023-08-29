# Openthread sensor service

This sample app serves a coap server, which sensor information can be fetched.

Three coap resources are available:

- GET /battery
- GET /ms8607
- GET /veml6030

which can be called with coap client. In openthread cli, you can use ot coap like following:

```
uart:~$ ot coap get fd3b:f3a8:9441:0:0:ff:fe00:bc02 battery con
Done
coap response from fd3b:f3a8:9441:0:0:ff:fe00:bc02 with payload: 7b226465766963655f6964

uart:~$ ot coap get fd3b:f3a8:9441:0:0:ff:fe00:bc02 ms8607 con
Done
coap response from fd3b:f3a8:9441:0:0:ff:fe00:bc02 with payload: 7b226465766963655f6964

uart:~$ ot coap get fd3b:f3a8:9441:0:0:ff:fe00:bc02 veml6030 con
Done
coap response from fd3b:f3a8:9441:0:0:ff:fe00:bc02 with payload: 7b226465766963655f6964
```

Note that the openthread cli doesn't show contents. If you have a proper border router configured, you can get result from coap server from any machine from your local network.

But retrieving the ip address of that device is still a problem, right? Well, that's what SRP is for.

If this app starts and has a SRP Server enabled in this thread network, this application registers a service over service registration protocol (SRP).

For example, I have a second node in this network with SRP Server enabled. Then this service can be queried with

```
uart:~$ ot dns browse _sensortag._udp.default.service.arpa
DNS browse response for _sensortag._udp.default.service.arpa.
F53FC2A0AD18E29D
    Port:5683, Priority:0, Weight:0, TTL:6092
    Host:F53FC2A0AD18E29D.default.service.arpa.
    HostAddress:fd3b:f3a8:9441:0:fc64:52fe:dc7a:b450 TTL:6092
    TXT:[] TTL:6092
```

and with proper border router configured, you can browse it with avahi on linux.


```
$ avahi-browse -t -r _sensortag._udp
TODO
```

## Border Router

I tested with two border router types:

- Google Nest Hub
- Apple Homepod Mini

## Build & Flash

```
# I use preconfigured 
$ west build -b nrf52840_sensortag -- -DOVERLAY_CONFIG="overlay-ot-fixed.conf"
$ west flash
```
