## usage
```bash
west build -t guiconfig
west build -b nrf52840_sensortag -- -DCONF_FILE=prj-log.conf
west build -b nrf52840_sensortag -- -DCONF_FILE=prj-shell.conf
west flash
```
## Documentation
https://www.homesmartmesh.com/docs/microcontrollers/nrf52/thread_sensortag/#tag_sensor_veml6030
