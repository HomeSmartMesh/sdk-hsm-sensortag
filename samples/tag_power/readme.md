## usage
```bash
west build -t guiconfig
west build -b nrf52840_sensortag -- -DCONF_FILE=prj.conf
west flash
```

device_set_power_state has no influence on GPIO as PGIO has no enable and no influence on ADC as ADC is anyway always disabled between usage

## Documentation
https://www.homesmartmesh.com/docs/microcontrollers/nrf52/thread_sensortag/#tag_power
