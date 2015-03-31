/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* by grqd_xp                                                            */
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/
#include <time.h>
#include <string.h>
#include <HAL/Interface/SDCard.h>
#include "stm324xg_eval_sdio_sd.h"
#include "stm324xg_eval.h"

/*-----------------------------------------------------------------------*/
/* Correspondence between physical drive number and physical drive.      */
/* Note that Tiny-FatFs supports only single drive and always            */
/* accesses drive number 0.                                              */

#define SECTOR_SIZE 512U
int read_count = 0;
int write_count = 0;

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                   */

DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{
	SD_CardInfo SDCardInfo;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// SDIO Interrupt ENABLE
	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	// DMA2 STREAMx Interrupt ENABLE
	NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	if (
		SD_Init() != SD_OK
		|| SD_GetCardInfo(&SDCardInfo) != SD_OK
//		|| SD_SelectDeselect((uint32_t) (SDCardInfo.RCA << 16)) != SD_OK
		|| SD_EnableWideBusOperation(SDIO_BusWide_4b) != SD_OK
//	||	SD_SetDeviceMode(SD_DMA_MODE) != SD_OK
	)
	{
		return RES_ERROR;
	}

	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{	
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

SD_Error Status;
DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{
	int timeout = 50000;
	if(count==1)
	{
		if (SD_ReadBlock((uint8_t*)buff,((int64_t)sector) << 9 , SECTOR_SIZE) != SD_OK)
			return RES_ERROR;
	}
	else
	{
		if (SD_ReadMultiBlocks((uint8_t*)buff,((int64_t)sector) << 9 ,SECTOR_SIZE,count) != SD_OK)
			return RES_ERROR;;
	}
	Status = SD_WaitReadOperation();
	while(SD_GetStatus() != SD_TRANSFER_OK)
		if (timeout-- == 0)
			return RES_ERROR;

	read_count++;

	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{
	int timeout = 50000;
	if(count==1)
	{
		if (SD_WriteBlock((uint8_t*)buff,((int64_t)sector) << 9 ,SECTOR_SIZE) != SD_OK)
			return RES_ERROR;
	}
	else
	{
		if (SD_WriteMultiBlocks((uint8_t*)buff,((int64_t)sector) << 9 ,SECTOR_SIZE,count) != SD_OK)
			return RES_ERROR;
	}
	Status = SD_WaitWriteOperation();
	while(SD_GetStatus() != SD_TRANSFER_OK)
		if (timeout-- == 0)
			return RES_ERROR;

	write_count++;
	return RES_OK;
}
#endif /* _READONLY */


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{	
	SD_CardInfo SDCardInfo;
	switch(ctrl)
	{
	case GET_SECTOR_SIZE:
	case GET_BLOCK_SIZE:
		if (buff)
			*(DWORD*)buff = 512;
		break;
	case GET_SECTOR_COUNT:
		if (buff)
		{
			uint32_t DeviceSizeMul = (SDCardInfo.SD_csd.DeviceSizeMul + 2);

			if(SDCardInfo.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
			{
				*(DWORD*)buff = (SDCardInfo.SD_csd.DeviceSize + 1) * 1024;
			}
			else
			{
				uint32_t NumberOfBlocks  = ((1 << (SDCardInfo.SD_csd.RdBlockLen)) / 512);
				*(DWORD*)buff = ((SDCardInfo.SD_csd.DeviceSize + 1) * (1 << DeviceSizeMul) << (NumberOfBlocks/2));
			}
		}
		break;
	case CTRL_SYNC:
		break;
	default:
		return RES_PARERR;
	}
	
	return RES_OK;
}

DWORD make_fattime(int sec, int min, int hour, int day, int mon, int year)
{
return ((year+1900-1980) << 25)
| ((mon+1) << 21)
| ((day) << 16)
| ((hour) << 11)
| ((min) << 5)
| ((sec) << 1)
;
	
}

DWORD get_fattime(void)
{
	return 0;
	/*
	time_t current_time;
	nmeaINFO *info = GPS_GetInfo();
	nmeaTIME *time =  info->utc2.year == 0 ? &info->utc : &info->utc2;
	struct tm _tm;
	
	_tm.tm_sec = time->sec;
	_tm.tm_min = time->min;
	_tm.tm_hour = time->hour;
	_tm.tm_mday = time->day;
	_tm.tm_mon = time->mon;
	_tm.tm_year = time->year;
	
	current_time = mktime(&_tm);
	
	current_time += 8 * 3600;	// CHINA = UTC+8
	_tm = *localtime(&current_time);
	
	return make_fattime(_tm.tm_sec, _tm.tm_min, _tm.tm_hour, _tm.tm_mday, _tm.tm_mon, _tm.tm_year);
	*/
}


void SDIO_IRQHandler(void)
{
  SD_ProcessIRQSrc();
}

void SD_SDIO_DMA_IRQHANDLER(void)
{
  SD_ProcessDMAIRQ();
}
