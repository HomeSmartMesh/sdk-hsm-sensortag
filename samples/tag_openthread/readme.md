## usage
```bash
west build -t guiconfig
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-shell.conf"
west flash
```

send udp
https://github.com/openthread/openthread/blob/main/src/cli/README_UDP.md


## Documentation
https://www.homesmartmesh.com/docs/microcontrollers/nrf52/thread_sensortag/#tag_power
