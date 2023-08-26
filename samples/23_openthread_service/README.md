# Openthread sensor service

This sample app serves a coap server, which sensor information can be fetched.

## Build & Flash

```
$ west build -b nrf52840_sensortag -- -DEXTRA_CONF_FILE="overlay-ot-fixed.conf"
$ west flash
```

