#
# Copyright (c) 2018 - 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

add_subdirectory_ifdef(CONFIG_ESB		    esb)
add_subdirectory_ifdef(CONFIG_SIMPLEMESH	simplemesh)
add_subdirectory_ifdef(CONFIG_APP_OT app_ot)
add_subdirectory_ifdef(CONFIG_APP_LED	app_led)

add_subdirectory_ifdef(CONFIG_APP_FLASH app_flash)
zephyr_include_directories_ifdef(CONFIG_APP_FLASH app_flash)
