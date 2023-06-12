#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>

#include "flash_settings_storage.h"

#define SETTINGS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(storage_partition)
#define SETTINGS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(storage_partition)
#define FLASH_PAGE_SIZE 4096

// SETTINGS_PARTITION_OFFSET 0xF0000 is using ~ 310 bytes for thread credentials
// last block 0xFF000 is not used
#define SETTINGS_QUICK_REBOOT_COUNT (SETTINGS_PARTITION_OFFSET + 0x7000)

static const struct device *flash_dev = SETTINGS_PARTITION_DEVICE;

void fss_write_data(uint8_t *data,uint16_t size){
	if (flash_erase(flash_dev, SETTINGS_QUICK_REBOOT_COUNT, FLASH_PAGE_SIZE) != 0) {
		printf("   Flash erase failed!\n");
	} else {
		printf(" * Succeeded Flash erase partition 'storage_partition' @0x%x 4KB Page!\n",SETTINGS_QUICK_REBOOT_COUNT);
	}
    off_t offset = SETTINGS_QUICK_REBOOT_COUNT;
    if (flash_write(flash_dev, offset, data,size) != 0) {
        printf("   Flash write failed!\n");
        return;
    }
}

void fss_read_data(uint8_t *data,uint16_t size){
	if (!device_is_ready(flash_dev)) {
		printf("Flash device not ready\n");
		return;
	}

    const off_t offset = SETTINGS_QUICK_REBOOT_COUNT;
    printf(" * reading from 'storage_partition' 0x%lx",offset);
    if (flash_read(flash_dev, offset, data,size) != 0){
        printf("   Flash read failed!\n");
        return;
    }
}

void fss_write_word(uint32_t *data){
    fss_write_data((uint8_t*)data,4);
}
void fss_read_word(uint32_t *data){
    fss_read_data((uint8_t*)data,4);
}
