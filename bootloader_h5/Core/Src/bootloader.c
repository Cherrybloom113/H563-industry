#include "stdint.h"
#include "draw.h"
#include "uart_device.h"
#include "ux_api.h"
#include "modbus.h"
#include "errno.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "bootloader.h"

#define CFG_OFFSET		0x081FE000
#define UPDATE_TIMEOUT		1000
static struct UART_Device *g_pUpdateUART;

static uint32_t BE32toLE32(uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3] << 0);
}

static int GetLocalFirmwareInfo(PFirmwareInfo ptFirmwareInfo)
{
	ptFirmwareInfo = (PFirmwareInfo)CFG_OFFSET;

	if(ptFirmwareInfo->version == 0xffffffff)
		return -1;

	return 0;
}


static int GetServerFirmwareInfo(PFirmwareInfo ptFirmwareInfo)
{
	uint8_t data = '1';
	uint8_t buf[32];
	
	/* send '1' to pc */
	if(0 != g_pUpdateUART->Send(g_pUpdateUART, &data, 1, UPDATE_TIMEOUT))
		return -1;

	/* we use 0x5a to sync */
	while(1)
	{
		if(0 != g_pUpdateUART->RecvByte(g_pUpdateUART, &data, UPDATE_TIMEOUT * 10))
			return -1;

		if(data != 0x5a)
		{
			buf[0] = data;
			break;
		}
	}

	for(int i = 1;i < 32;i++)
	{
		if(0 != g_pUpdateUART->RecvByte(g_pUpdateUART, &buf[i], UPDATE_TIMEOUT*10))
			return -1;
	}

	/* transfer data to struct */
	ptFirmwareInfo->version = BE32toLE32(&buf[0]);
	ptFirmwareInfo->file_len = BE32toLE32(&buf[4]);
	ptFirmwareInfo->load_addr = BE32toLE32(&buf[8]);
	ptFirmwareInfo->crc32 = BE32toLE32(&buf[12]);
	strncpy((char *)ptFirmwareInfo->file_name, (char *)&buf[16], 16);

	return 0;
	
}

static int GetServerFirmware(uint8_t *buf, uint32_t len)
{
	uint8_t data = '2';
	
	/* send '2' to  */
	if(0 != g_pUpdateUART->Send(g_pUpdateUART, &data, 1, UPDATE_TIMEOUT))
		return -1;

	/* recv data to buf */
	for(int i = 0; i < len; i++)
	{
		if(0 != g_pUpdateUART->RecvByte(g_pUpdateUART, &buf[i], UPDATE_TIMEOUT*10))
			return -1;
	}

	return 0;
	
}



static uint32_t CaculateCRC32(uint8_t *buf, uint32_t len) {
    uint32_t crc=0xFFFFFFFF;

    for(size_t i=0;i<len;i++) {
            char ch=buf[i];
            for(size_t j=0;j<8;j++) {
                    uint32_t b=(ch^crc)&1;
                    crc>>=1;
                    if(b) crc=crc^0xEDB88320;
                    ch>>=1;
            }
    }

    return ~crc;
}


void BootLoaderTask(void *param)
{
	int err;
	int needUpdate;
	uint32_t crc32;
	FirmwareInfo localFirmInfo;
	FirmwareInfo serverFirmInfo;
	
	struct UART_Device *pUSBDevice = GetUartDevice("usb");

	g_pUpdateUART = pUSBDevice;

	pUSBDevice->Init(pUSBDevice, 115200, 'N', 8, 1);

	/* wait for init */
	vTaskDelay(10000);
	
	while(1)
	{
		/* read local FirmwareInfo */
		err = GetLocalFirmwareInfo(&localFirmInfo);
		if(err)
		{
			/* update */
			needUpdate = 1;
		}

		/* read server FirmwareInfo */
		err = GetServerFirmwareInfo(&serverFirmInfo);
		if(err)
		{
			/* print err and boot to app */
			pUSBDevice->Send(pUSBDevice, "GetServerFirmwareInfo error", strlen("GetServerFirmwareInfo error"), UPDATE_TIMEOUT);
			needUpdate = 0;
		}
		else
		{
			/* compare version */
			if(serverFirmInfo.version > localFirmInfo.version)
				needUpdate = 1;
//			else							 /* read mem's count is Not sure maybe too big */
//				needUpdate = 0;
		}


		/* update mode */
		if(needUpdate)
		{
			uint8_t *firmwareBuf = (uint8_t *)pvPortMalloc(serverFirmInfo.file_len);
			if(!firmwareBuf)
			{
				pUSBDevice->Send(pUSBDevice, "pvPortMalloc error", strlen("pvPortMalloc error"), UPDATE_TIMEOUT);

			}
			
			err = GetServerFirmware(firmwareBuf,serverFirmInfo.file_len);
			if(!err)
			{
				crc32 = CaculateCRC32(firmwareBuf, serverFirmInfo.file_len);

				if(crc32 == serverFirmInfo.crc32)
				{
					/* download ok */
					pUSBDevice->Send(pUSBDevice, "download ok", strlen("download ok"), UPDATE_TIMEOUT);

				}
				else
				{
					/* download fail */
				}
			}
			else
			{
				pUSBDevice->Send(pUSBDevice, "GetServerFirmware error", strlen("GetServerFirmware error"), UPDATE_TIMEOUT);
			}
			
		}
		else
		{
			/* jump to app */
		}
	}


}


