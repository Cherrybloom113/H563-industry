#ifndef _BOOTLOADER_H
#define _BOOTLOADER_H

#include <stdint.h>

typedef struct FirmwareInfo {
	uint32_t version;
	uint32_t file_len;
	uint32_t load_addr;
	uint32_t crc32;
	uint8_t file_name[16];
}FirmwareInfo, *PFirmwareInfo;


void BootLoaderTask(void *param);

#endif
