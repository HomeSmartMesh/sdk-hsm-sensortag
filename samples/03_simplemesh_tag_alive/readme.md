## usage
```bash
west build -b nrf52840_sensortag -t guiconfig

west build -b nrf52840_sensortag -- -DCONF_FILE=prj.conf
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-tracing.conf"

west flash
```
