## usage
```bash
west build -b nrf52840_sensortag -t guiconfig
west build -b nrf52840_sensortag
west build -b nrf52840_sensortag -- -DCONF_FILE=prj-fixed.conf
west build -b nrf52840_sensortag -- -DCONF_FILE=prj.conf
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-debug.conf"
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-tracing.conf"
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-debug.conf overlay-tracing.conf"
west flash
nrfjprog -f nrf52 --eraseall
```

## Documentation
https://www.homesmartmesh.com/docs/microcontrollers/nrf52/thread_sensortag/#tag_sensors_broadcast
