## usage
```bash
west build -t guiconfig
west build
west build -- -DCONF_FILE=prj-fixed.conf
west build -- -DOVERLAY_CONFIG="overlay-log.conf"
west build -- -DCONF_FILE="prj.conf overlay-log.conf"
west build -- -DCONF_FILE="prj.conf overlay-log.conf overlay-tracing.conf"
west flash
nrfjprog -f nrf52 --eraseall
```

## Documentation
https://www.homesmartmesh.com/docs/microcontrollers/nrf52/thread_sensortag/#tag_sensors_broadcast
