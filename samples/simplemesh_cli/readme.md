## usage
```bash
west build -t guiconfig -b nrf52840dongle_nrf52840

west build -b nrf52840dongle_nrf52840 -- -DCONF_FILE=prj.conf
west build -b decawave_dwm1001_dev -- -DCONF_FILE=prj-dwm.conf

west build -b nrf52840dongle_nrf52840 -- -DCONF_FILE="prj.conf overlay-tracing.conf"
west build -b nrf52840dongle_nrf52840 -- -DCONF_FILE="prj.conf overlay-usb.conf"

west flash

west flash --snr 260103215

```
