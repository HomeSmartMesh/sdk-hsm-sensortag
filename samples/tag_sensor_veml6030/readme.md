## usage
```bash
west build -t guiconfig
west build -b nrf52840_sensortag -- -DCONF_FILE=prj-shell.conf
west build -b nrf52840_sensortag -- -DCONF_FILE=prj-log.conf
west flash
```
