# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_MS8607 ms8607.c)
zephyr_library_sources_ifdef(CONFIG_MS8607 ms8607_i2c.c)
