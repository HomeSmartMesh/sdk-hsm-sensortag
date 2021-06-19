## usage
```bash
west build -t guiconfig -b nrf52840dongle_nrf52840

west build -b nrf52840dongle_nrf52840 -- -DCONF_FILE=prj.conf
west build -b nrf52840dongle_nrf52840 -- -DCONF_FILE="prj.conf overlay-tracing.conf"
west build -b nrf52840dongle_nrf52840 -- -DCONF_FILE="prj.conf overlay-usb.conf"

west flash
```


## Documentation
https://www.homesmartmesh.com/docs/microcontrollers/nrf52/thread_sensortag/#tag_openthread_udp


# issue
* can't go in low power
https://groups.google.com/g/openthread-users/c/3xxCPvRz8qM

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

# not needed function
```c
void openthread_set_mtd()
{
	otInstance *openthread = openthread_get_default_instance();
    bool rxOnWhenIdle = false;
    bool deviceType   = false;//Not FTD just MTD
    bool networkData  = false;//No full Network Data
	otLinkModeConfig linkMode = {rxOnWhenIdle, deviceType, networkData};
	otThreadSetLinkMode(openthread,linkMode);
}
```
