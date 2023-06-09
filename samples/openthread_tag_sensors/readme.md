## usage
```bash
west build -t guiconfig
west build
west build -- -DCONF_FILE=prj-fixed.conf
west build -- -DOVERLAY_CONFIG="overlay-log.conf"
west build -- -DCONF_FILE=prj-fixed.conf -DOVERLAY_CONFIG=overlay-log.conf
west flash
nrfjprog -f nrf52 --eraseall
```

then allow it to join on the boarder router
```shell
sudo ot-ctl
commissioner start
commissioner joiner add * ABCDE2 3600
```

## Documentation
https://www.homesmartmesh.com/docs/microcontrollers/nrf52/thread_sensortag/#tag_sensors_broadcast
