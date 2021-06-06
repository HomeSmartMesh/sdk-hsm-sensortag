## usage
```bash
west build -b nrf52840_sensortag -t guiconfig
west build -b nrf52840_sensortag
west build -b nrf52840_sensortag -- -DCONF_FILE=prj.conf
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-tracing.conf"
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-debug.conf"
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-debug.conf overlay-tracing.conf"
west flash
```

## Documentation
https://www.homesmartmesh.com/docs/microcontrollers/nrf52/thread_sensortag/#tag_openthread_udp



otSysDeinit -> nrf5RadioDeinit -> nrf_802154_deinit -> nrf_802154_core_deinit -> nrf_802154_trx_disable -> nrf_radio_power_set -> 


* otSysDeinit : only in platoform includes
* nrf5RadioDeinit : not compiled for nRF52840
* nrf_802154_deinit : calls nrf_802154_core_deinit
* nrf_802154_core_deinit : conditional call for nrf_802154_trx_disable();
* nrf_802154_init : does not call nrf_802154_trx_enable
* nrf_802154_trx_disable : on nrf_802154_trx.h which is not exposed


* net_init -> l3_init -> init_rx_queues -> net_if_post_init -> net_if_up -> openthread_start  -> otThreadSetLinkMode
(RDN: set to 011)
then in main RDN reads 111

* openthread_enable -> openthread_start -> otThreadSetLinkMode -> 
* NcpBase::HandlePropertySet<SPINEL_PROP_THREAD_MODE> -> otThreadSetLinkMode
