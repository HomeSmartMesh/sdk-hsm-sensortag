west build -b nrf52840_sensortag -- -DCONF_FILE=prj.conf
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-network.conf"
west build -b nrf52840_sensortag -- -DCONF_FILE="prj.conf overlay-network.conf overlay-piodebug.conf"
