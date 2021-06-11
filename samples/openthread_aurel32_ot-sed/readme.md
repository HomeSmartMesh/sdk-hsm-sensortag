west build -b nrf52840_sensortag -- -DCONF_FILE=prj.conf
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-network.conf"
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-network.conf overlay-piodebug.conf"


1) west updated zephyr to 79a6c07536bc14583198f8e3555df6134d8822cf
2) delete build, using panid 0x1212 chan 18
3) high activity, no sign on chan 18 but ultra low power on deep sleep OK
4) nrfjprog --eraseall, west flash
5) high power no signs on chan 18
