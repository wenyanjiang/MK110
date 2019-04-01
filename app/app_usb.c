#include <string.h>
#include "type.h"
#include "delay.h"
#include "debug.h"
#include "app_usb.h"
#include "data.h"
#include "dev_pwm.h"
#include "watchdog.h"

#include "data.h"
#include "drv_config.h"


#include "dev_spi.h"
#include "dev_usb.h"
#include "dev_pwm.h"

#include "oxifp_type_define.h"
#include "uarts.h"
#include "uarts_interface.h"
#include "OTG_device_hcd.h"

#include "app_spi.h"

#define	LOG_USB_CMD_EN	0


#define	REG_OTG_INTRRX		          	(*(volatile unsigned short*)(0x40000300 + 0x04))

#define REST_CHIP               	0
#define WRITE_REG               	1
#define READ_REG                	2
#define READ_CHIP_ID            	3
#define READ_FIRMWARE_VERSION   	4
#define READ_CALIBRATION_DATA   	5
#define WRITE_CALIBRATION_DATA  	6
#define GOTO_TEST_MODE          	7
#define QUERY_WAKE_UP           	8
#define START_TRANSFER_DATA     	9
#define QUERY_TOUCH             	10
#define UP_DATA_CODE         		11
#define WRITE_DELAY					12
#define READ_DELAY					13
#define WRITE_LOCK					14
#define ONE_KEY_RESET				15
#define GOTO_SLEEP					16
#define WRITE_SN1					17
#define READ_SN1					18
#define WRITE_SN2					19
#define READ_SN2					20
#define WRITE_SRAM_DATA				21
#define READ_SRAM_DATA				22
#define FLASH_INIT					23
#define WRITE_GAIN_AVR				24
#define READ_GAIN_AVR				25
#define READ_FRAME_THEN_PMU_STATUS	26
#define PB_ENROLL					27
#define PB_VERIFY					28
#define PB_DEL_FINGER				29
#define CANCEL_STATE				30
#define SEND_8BIT_DATA				31
#define LIST_FINGERS				32
#define DOWLOAD_IMAGE_ENROLL  		33
#define DOWLOAD_IMAGE_VERIFY  		34
#define SEND_8BIT_PB_DATA			35
#define GET_PB_ALGO_DATA			36
#define SET_CALI_FRAME_DATA			37
#define SET_WINDOW_WIDTH			38

uint8_t* DataBuffer = 0;
static uint32_t usbPackCount = 0;

///extern uint8_t* frameBuffer;

extern unsigned char dark_line_flag;
volatile u32 g_vu32GlobalTick;

unsigned char Chip_ID[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
const uint8_t IDDATA[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

u8 gu8CmdBuf[64];
u32 gu32Len;

void ReadChipId()
{
	Chip_ID[0] = Efuse_ReadData(10);
	Chip_ID[1] = Efuse_ReadData(11);
	Chip_ID[2] = Efuse_ReadData(12);
	Chip_ID[3] = Efuse_ReadData(13);
	Chip_ID[4] = Efuse_ReadData(14);
	Chip_ID[5] = Efuse_ReadData(15);
}

typedef void (*shell_cmd_func) ( unsigned char*);
typedef struct CLICmds_st
{
        shell_cmd_func       CmdHandler;
        const char           *CmdUsage;
} shell_cmds;

uint8_t ResBuf[64];




/* CMD index: 0 */
void _app_Usb_ResetChip (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
/*
	oxi_chip_reset(Arg[0],Arg[1]);
	mv_init_oxi_chip(0);
*/
	//DEV_PMU_Reset(Buf[0], Buf[1]);
	//APP_PMU_InitPMU(0);

	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 1 */
void _app_Usb_WriteReg (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
	if(Buf[2] == 0x00)
	{
		//APP_PMU_WriteReg_SaveFlash(Buf[0],Buf[1]);
		if(Buf[0] == 0x1c)
		{
			setLedBrightness(Buf[1]);
		}else if(Buf[0] == 0x9d){
		///	setHighVoltage(Buf[1]);
		}else{
			Dev_SPIM1_Write(Buf[0],Buf[1]);
		}
	#if LOG_TOT_EN
		DBG("save %x %x \n", Buf[0], Buf[1]);
	#endif
	}
	else
	{
		//DEV_PMU_WriteReg(Buf[0],Buf[1]);
	#if LOG_TOT_EN
		DBG("not save %x %x \n", Buf[0], Buf[1]);	
	#endif
	}

	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 2 */
void _app_Usb_ReadReg (uint8_t *Buf)
{
	u16 RegData;
#if LOG_TOT_EN	
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif	

	//delayUS(2*1000);

	//RegData = DEV_PMU_ReadReg(Buf[0]);
	if(Buf[0] == 0x1c)
	{
		RegData = getLedBrightness();
	}else if(Buf[0] == 0x9d){
		RegData = getHighVoltage();
	}else{
		Dev_SPIM1_Read(Buf[0],&Buf[1]);
	}

	ResBuf[9] = RegData;
	ResBuf[10] = RegData>>8;
	
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 3 */
void _app_Usb_ReadChipID (uint8_t *Buf)
{
	u16 PMUid;
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif	
	memcpy(ResBuf + 0 ,Chip_ID,6);
	memcpy(ResBuf + 8,Chip_ID,6);
	//PMUid=DEV_PMU_ReadReg(0x01);
	PMUid = 0x001b;
	
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 4 */
void _app_Usb_ReadFirmwareVersion (uint8_t *Buf)
{
	u8 u8VerBuf[4];
#if LOG_TOT_EN	
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
/*
	
	memcpy(ResBuf + 8, FirmwareVersion, 4);
	
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
	*/
}

/* CMD index: 5 */
void _app_Usb_ReadCalibrationData (uint8_t *Buf)
{
	uint32_t Address;
	uint32_t Length;
	uint32_t Crc;
	uint32_t Temp0;
	uint32_t Temp1;
	uint32_t i;
	u32 Len;
	OTG_DEVICE_ERR_CODE enUSBErrorID;
	
	volatile uint32_t vu32StartTick;
	volatile uint32_t vu32EndTick;

#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif	
	DataBuffer = frameBuffer;

	Temp0 = PAK_COUNT_GAIN_8BIT;
	for(i=0 ;i<PAK_COUNT_GAIN_8BIT ;i++)
	{
	///	OnchipFlashRead(CALIDATA_ONCHIP_ADDRESS + i*4096, DataBuffer+i*4096, 4096);
	}

	for(i = 0;i < Temp0; i++)
	{
		vu32StartTick = GetSysTick1MsCnt();
		while(1)
		{
			Len = 0;
			vu32EndTick = GetSysTick1MsCnt();
			if (vu32EndTick > vu32StartTick + 500) 
			{
			#if LOG_TOT_EN
					DBG("wait the CMD of PC timeout !!!");
			#endif
					break;
			}
			
			OTG_DeviceSetupReceive(Setup, 8, &Len);
			if(!Len)
			{
				continue;
			}
			/*
			if(OTG_DeviceSetupReceive(Setup, 8,&Len) != DEVICE_NONE_ERR) 
			{
				continue;
			}
			*/
			if(Setup[3] == 0x23)
			{
				enUSBErrorID = OTG_DeviceControlSend((unsigned char*)DataBuffer+i*4096, 4096, 10);
				if(enUSBErrorID == DEVICE_NONE_ERR)
				{
					OTG_DeviceControlSend(DataBuffer,0,10);
					break;
				}
				else
				{
				#if LOG_TOT_EN
					DBG("ERROR ControlSend,error ID=%d\n",enUSBErrorID);
				#endif
				}
			}
			else
			{
			#if LOG_TOT_EN
				DBG("ERROR\n");
			#endif
			}
		}
	}
	Crc = 0;
	
	Crc = CRC16(DataBuffer, (PAK_COUNT_GAIN_8BIT*4096),0);
#if LOG_TOT_EN
 	DBG("gain Crc = 0x%x\n",Crc);
#endif
 	memcpy(ResBuf+8,(unsigned char*)&Crc,4);

	OTG_DeviceBulkSend(0x82,ResBuf,64,10);

}

/* CMD index: 6 */
void _app_Usb_WriteCalibrationData (uint8_t *Buf)
{
	uint32_t i,j,k,Crc,Crc1;
	unsigned char dark_line;
	unsigned char bak;
	u32 Len;
	u16* ptr =0;
	u16 data =0;
	unsigned char dark_line_flag;
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	DBG("write GAIN dark_line=%d PAK_COUNT_GAIN=%d\n", ResBuf[8],PAK_COUNT_GAIN);
	#endif
#endif
	DataBuffer = frameBuffer;
#if 1
	dark_line = ResBuf[8];
	dark_line_flag = dark_line;
	k = 0;
	Crc = 0;
	Crc1 = 0;

	ptr =(u16*)DataBuffer;

	for(i = 0;i < PAK_COUNT_GAIN_8BIT; i++)
	{
		for(j=0;j<64;j++)
		{
			OTG_DeviceBulkReceive(0x03, DataBuffer + j*64, 64, &Len, 100);
			if(Len != 64)
			{
			#if LOG_TOT_EN
				DBG("receive data error, Len=%d, i=%d, j=%d\n",Len,i,j);
			#endif
			}
		}

	///	OnchipFlashErase(CALIDATA_ONCHIP_ADDRESS + i*4096, 4096);
	///	OnchipFlashWrite(CALIDATA_ONCHIP_ADDRESS + i*4096, DataBuffer, 4096);
	///	OnchipFlashRead(CALIDATA_ONCHIP_ADDRESS + i*4096 , DataBuffer+4096, 4096);
		if (memcmp(DataBuffer, DataBuffer+ 4096, 4096) != 0) 
		{
		#if LOG_TOT_EN
			DBG("gain write in on-chip flash error,i=%d\n",i);
		#endif
		}else{
		#if LOG_TOT_EN
			DBG("gain write in on-chip flash sucess,i=%d\n",i);
		#endif
		}
		WDG_Feed();
	}
	Crc = 0;
	WDG_Feed();
	for(i =0 ;i<PAK_COUNT_GAIN_8BIT ;i++)
	{
	///	OnchipFlashRead(CALIDATA_ONCHIP_ADDRESS + i * 4096 , DataBuffer+i*4096, 4096);
	}
	Crc = CRC16(DataBuffer, (256*360), 0);
	#if LOG_TOT_EN
	DBG("test crc  Crc = 0x%x\n",Crc);	
	#endif
	memcpy(ResBuf+8,(unsigned char*)&Crc,4);

	i = (PAK_COUNT_GAIN*4096);
	DataBuffer[0] = i;
	DataBuffer[1] = i >> 8;
	DataBuffer[2] = i >> 16;
	DataBuffer[3] = i >> 24;
	DataBuffer[4] = Crc;
	DataBuffer[5] = Crc >> 8;
	DataBuffer[6] = Crc >> 16;
	DataBuffer[7] = Crc >> 24;
	DataBuffer[8] = 1;
	DataBuffer[9] = 0;	
	DataBuffer[10] = dark_line_flag;
	Crc = REG_OTG_INTRRX;
#else
	dark_line = ResBuf[8];
	dark_line_flag = dark_line;
	k = 0;
	Crc = 0;
	Crc1 = 0;
	
	ptr =(u16*)DataBuffer;
	
	for(i = 0;i < PAK_COUNT_GAIN; i++)
	{
		for(j=0;j<64;j++)
		{
			OTG_DeviceBulkReceive(0x03, DataBuffer + j*64, 64, &Len, 100);
			if(Len != 64)
			{
				DBG("receive data error, Len=%d, i=%d, j=%d\n",Len,i,j);
			}
		}

		OnchipFlashErase(CALIDATA_ONCHIP_ADDRESS + i*4096, 4096);
		OnchipFlashWrite(CALIDATA_ONCHIP_ADDRESS + i*4096, DataBuffer, 4096);
		OnchipFlashRead(CALIDATA_ONCHIP_ADDRESS + i*4096 , DataBuffer+4096, 4096);
		if (memcmp(DataBuffer, DataBuffer+ 4096, 4096) != 0) 
		{
			DBG("gain write in on-chip flash error,i=%d\n",i);
		}else{
			DBG("gain write in on-chip flash sucess,i=%d\n",i);
		}
	}
	Crc = 0;

	for(i =0 ;i<PAK_COUNT_GAIN ;i++)
	{
		OnchipFlashRead(CALIDATA_ONCHIP_ADDRESS + i * 4096 , DataBuffer+i*4096, 4096);
	}
	Crc = CRC16(DataBuffer, (PAK_COUNT_GAIN*4096), 0);
	DBG("test crc  Crc = 0x%x\n",Crc);	
	memcpy(ResBuf+8,(unsigned char*)&Crc,4);
	
	i = (PAK_COUNT_GAIN*4096);
	DataBuffer[0] = i;
	DataBuffer[1] = i >> 8;
	DataBuffer[2] = i >> 16;
	DataBuffer[3] = i >> 24;
	DataBuffer[4] = Crc;
	DataBuffer[5] = Crc >> 8;
	DataBuffer[6] = Crc >> 16;
	DataBuffer[7] = Crc >> 24;
	DataBuffer[8] = 1;
	DataBuffer[9] = 0;	
	DataBuffer[10] = dark_line_flag;
	Crc = REG_OTG_INTRRX;
#endif

	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 7 */
void _app_Usb_GotoTestMode (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif	
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}


/* CMD index: 9 */
void _app_Usb_StartTransferData (uint8_t *Buf)
{
	uint32 i = 0;
#if LOG_TOT_EN
#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
#endif
#endif

	{
		DEV_USB_SendRawImage(frameBuffer, PAK_COUNT_16BIT);
		
	}

	//OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 10 */
void _app_Usb_QueryTouch (uint8_t *Buf)
{
///	#if LOG_USB_CMD_EN DBG("%s\n",__FUNCTION__); #endif
///
///	if (DEV_GPIO_CheckTouch(30) == 1)
///	{
///		ResBuf[8] = 0xFF;
//	///	DBG("get touch!!\n");
///	} 
///	else 
///	{
///		ResBuf[8] = 0x00;
///	}
//
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 11 */
void _app_Usb_UpDataCode (uint8_t *Buf)
{
	uint32_t i,j,Crc;
	uint32_t CodeSize;
	uint32_t CodeCrc;
	uint32_t Len;

	DataBuffer = frameBuffer;

	memcpy((uint8_t*)&CodeSize,Buf,4);
	memcpy((uint8_t*)&CodeCrc,Buf+4,4);
#if LOG_TOT_EN
	DBG("UP_DATA_CODE CodeSize=%d CodeCrc=%d\n", CodeSize, CodeCrc);
#endif
	for(i=0;i<CodeSize/4096;i++)
	{
		for(j=0;j<64;j++)
		{
			OTG_DeviceBulkReceive(0x03,DataBuffer+j*64,64,&Len,100);
		}

		OnchipFlashErase(UPDATE_FIRMWARE_ADDR + i*4096,4096);
		OnchipFlashWrite(UPDATE_FIRMWARE_ADDR + i*4096, DataBuffer,4096);
		WDG_Feed();
	}
	Crc = CRC16((unsigned char*)(UPDATE_FIRMWARE_ADDR),CodeSize,0);
#if LOG_TOT_EN
	DBG("%08X  %08X  %08X\n",CodeSize,CodeCrc,Crc);
#endif
	if(CodeCrc == Crc)
	{
	#if LOG_TOT_EN
		DBG("WRITE MVA OK\n");
	#endif
		ResBuf[8] = 0x55;
	}
	else
	{
	#if LOG_TOT_EN
		DBG("WRITE MVA Error\n");
	#endif
		ResBuf[8] = 0xAA;
	}
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
	if(CodeCrc == Crc)
	{
		*(volatile uint32_t *)0x4002E800 = 0xFF;
		*(volatile uint32_t *)0x4002E824 = UPDATE_FIRMWARE_ADDR;
		*(volatile uint32_t *)0x4002E800 = 0;
		*(volatile uint32_t *)0x40022008 = 1;
		*(volatile uint32_t *)0x40022008 = 0;
	}
	Crc = REG_OTG_INTRRX;//Çå³ý±êÖ¾
}

/* CMD index: 12 */
void _app_Usb_WriteDelay (uint8_t *Buf)
{
#if LOG_TOT_EN

	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 13 */
void _app_Usb_ReadDelay (uint8_t *Buf)
{
#if LOG_TOT_EN

#if LOG_USB_CMD_EN 
DBG("%s\n",__FUNCTION__); 
#endif
#endif
OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 14 */
void _app_Usb_WriteLock (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 15 */
void _app_Usb_OneKeyReset (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
	//APP_PMU_OneKeyInitPMU(0);
	//fp_remove_all_finger_templates();
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}
/* CMD index: 8 */
void _app_Usb_QueryWakeUp (uint8_t *Buf)
{
	int i;
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN
	DBG("%s\n",__FUNCTION__);
	#endif
#endif
///	WakeUpROIC();

///	g_vu32GlobalTick = gSysTick;
	
///	delayUS(10*1000);

///	delayUS(500);
	///PwmForceOutputExample();
/*
	for (i = 0; i < 2; i++)
	{
		if (DEV_PMU_CMD_Wakeup())
		{
			ResBuf[8] = 0xFF;  // wake up ok
			break;
		}
		else
		{
			ResBuf[8] = 0;
		}
		DelayMs(30);
	}

	if ((i >= 2)&&(ResBuf[8] == 0))
	{
		DEV_PMU_CMD_Restart();
		DEV_PMU_CMD_Sleep();
	}
	*/
	ResBuf[8] = 0xFF;

///	DBG("wake up:%d\n",gSysTick-g_vu32GlobalTick);

	g_vu32GlobalTick = gSysTick;
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}
/* CMD index: 16 */
void _app_Usb_GotoSleep (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
	//PwmForceDisable();
///	SleepROIC();
///	PwmLedDisable();
///	PwmHVDisable();
	//DEV_PMU_CMD_Sleep();
	//DelayMs(10);

	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
	
	//fp_update_after_verify(100);
}

/* CMD index: 17 */
void _app_Usb_WriteSN1 (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
	/*
	//mfs_read_data(SN1_START_ADDR, DataBuffer, 4096);
	memcpy(DataBuffer,ResBuf+8,12);
	mfs_sector_erase(SN1_START_ADDR);
	mfs_write_data(SN1_START_ADDR,DataBuffer,4096);
	mfs_read_data(SN1_START_ADDR, DataBuffer+4096, 4096);
	if (memcmp(DataBuffer, DataBuffer + 4096, 4096)) 
	{
		DBG("write sn1 error\n");
	} 
	else 
	{
		DBG("write sn1 ok\n");	
	}
	*/
	//PwmForceDisable();
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 18 */
void _app_Usb_ReadSN1 (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
	//mfs_read_data(SN1_START_ADDR, DataBuffer, 4096);
	memcpy(ResBuf+8,DataBuffer,12);

	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 19 */
void _app_Usb_WriteSN2 (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
#if 0	
	//mfs_read_data(SN2_START_ADDR, DataBuffer, 4096);
///	mfs_sector_erase(SN2_START_ADDR);
	memcpy(DataBuffer+64, ResBuf+8, 12);
///	mfs_write_data(SN2_START_ADDR,DataBuffer,4096);
	mfs_read_data(SN2_START_ADDR, DataBuffer+4096, 4096);
	if (memcmp(DataBuffer, DataBuffer + 4096, 4096)) 
	{
		DBG("write sn2 error\n");
	} 
	else 
	{
		DBG("write sn2 ok\n");	
	}		
#endif	
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 20 */
void _app_Usb_ReadSN2 (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
	//mfs_read_data(SN2_START_ADDR, DataBuffer, 4096);
	memcpy(ResBuf+8, DataBuffer+64, 12);
	
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 21 */
void _app_Usb_WriteSramData (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif	
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 22 */
void _app_Usb_ReadSramData (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif	
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 23 */
void _app_Usb_FlashInit (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif	
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 24 */
void _app_Usb_WriteGainAvr (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif	
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 25 */
void _app_Usb_ReadGainAvr (uint8_t *Buf)
{
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif	
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 26 */
void _app_Usb_ReadFrame_Then_PmuStatus (uint8_t *Buf)
{
	ST_SPIS_REC_CMD_HEADER* pstCmd;
#if LOG_TOT_EN
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
	//DBG("%s\n",__FUNCTION__);
	//DelayUs(100);
	pstCmd->u8ParaBuf[1] = 1;

	_app_Spis_ClrFrame(pstCmd);
	_app_Spis_CaptureGainImage(pstCmd);

		
	if (1)
	{
		ResBuf[8] = 0xFF;
		usbPackCount = 0;
	} 
	else 
	{
		ResBuf[8] = 0x00;  // send read frame failed
	}
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 27 */

/* CMD index: 30 */
void _app_Usb_CancelState (uint8_t *Buf)
{
	oxifp_error_t err;
#if LOG_TOT_EN	
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif
	//err = fp_cancel();

	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 31 */
void _app_Usb_Send8bitData (uint8_t *Buf)
{
	u32 i;
	u8* pu8TempImageBuf = NULL;
#if LOG_TOT_EN	
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif	
	
}

/* CMD index: 35 */
void _app_Usb_Send8bitPbData (uint8_t *Buf)
{
	u32 i;
	u8* pu8TempImageBuf = NULL;
#if LOG_TOT_EN	
	#if LOG_USB_CMD_EN 
	DBG("%s\n",__FUNCTION__); 
	#endif
#endif	

	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 40 */
void _app_Usb_SetImageSpec (uint8_t *Buf)
{
#if LOG_TOT_EN
#if LOG_USB_CMD_EN 
	fpDBG("%s\n",__FUNCTION__); 
#endif
#endif
	g_stImageSpecInfo.u32Width  = (((u16)Buf[0])<<8) | Buf[1];
	g_stImageSpecInfo.u32Height= (((u16)Buf[2])<<8) | Buf[3];
	g_stImageSpecInfo.u32Start_X= (((u16)Buf[4])<<8) | Buf[5];
	g_stImageSpecInfo.u32Start_Y= (((u16)Buf[6])<<8) | Buf[7];
	g_stImageSpecInfo.u32Len_X= (((u16)Buf[8])<<8) | Buf[9];
	g_stImageSpecInfo.u32Len_Y= (((u16)Buf[10])<<8) | Buf[11];

	g_stImageSpecInfo.u32Pack8BitCnt = (g_stImageSpecInfo.u32Len_X*g_stImageSpecInfo.u32Len_Y+4095)/4096;
	g_stImageSpecInfo.u32Pack16BitCnt = (g_stImageSpecInfo.u32Width*g_stImageSpecInfo.u32Height*2+4095)/4096;	
	g_stImageSpecInfo.u32PackGainCnt = g_stImageSpecInfo.u32Pack16BitCnt;


	ResBuf[7] = 0x00;
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}

/* CMD index: 41 */
void _app_Usb_GetImageSpec (uint8_t *Buf)
{
	u8 i;
	
#if LOG_TOT_EN
#if LOG_USB_CMD_EN 
	fpDBG("%s\n",__FUNCTION__); 
#endif
#endif
	ResBuf[8] = ((u8)(g_stImageSpecInfo.u32Width>>8))&0xFF;
	ResBuf[9] = ((u8)g_stImageSpecInfo.u32Width)&0xFF;
	ResBuf[10] = ((u8)(g_stImageSpecInfo.u32Height>>8))&0xFF;
	ResBuf[11] = ((u8)g_stImageSpecInfo.u32Height)&0xFF;
	
	ResBuf[12] = ((u8)((g_stImageSpecInfo.u32Start_X-2)>>8))&0xFF;
	ResBuf[13] = ((u8)(g_stImageSpecInfo.u32Start_X-2))&0xFF;

	ResBuf[14] = ((u8)(g_stImageSpecInfo.u32Start_Y>>8))&0xFF;
	ResBuf[15] = ((u8)g_stImageSpecInfo.u32Start_Y)&0xFF;
	
	ResBuf[16] = ((u8)((g_stImageSpecInfo.u32Len_X+2)>>8))&0xFF;
	ResBuf[17] = ((u8)(g_stImageSpecInfo.u32Len_X+2))&0xFF;

	ResBuf[18] = ((u8)(g_stImageSpecInfo.u32Len_Y>>8))&0xFF;
	ResBuf[19] = ((u8)g_stImageSpecInfo.u32Len_Y)&0xFF;


	ResBuf[7] = 0x00;
	OTG_DeviceBulkSend(0x82,ResBuf,64,10);
}



shell_cmds mv_shell[] =
{
	{_app_Usb_ResetChip , "_app_Usb_ResetChip "},//0
	{_app_Usb_WriteReg , "_app_Usb_WriteReg "},//1//
	{_app_Usb_ReadReg , "_app_Usb_ReadReg "},//2//
	{_app_Usb_ReadChipID , "_app_Usb_ReadChipID "},//3//3
	{_app_Usb_ReadFirmwareVersion , "_app_Usb_ReadFirmwareVersion "},//4//4
	{_app_Usb_ReadCalibrationData , "_app_Usb_ReadCalibrationData "},//5
	{_app_Usb_WriteCalibrationData , "_app_Usb_WriteCalibrationData "},//6
	{_app_Usb_GotoTestMode , "_app_Usb_GotoTestMode "},//7
	{_app_Usb_QueryWakeUp , "_app_Usb_QueryWakeUp "},//8
	{_app_Usb_StartTransferData , "_app_Usb_StartTransferData "},//9
	{_app_Usb_QueryTouch , "_app_Usb_QueryTouch "},//10
	{_app_Usb_UpDataCode , "_app_Usb_UpDataCode "},//11
	{_app_Usb_WriteDelay , "_app_Usb_WriteDelay "},//12
	{_app_Usb_ReadDelay , "_app_Usb_ReadDelay "},//13
	{_app_Usb_ReadDelay , "_app_Usb_ReadDelay "},//14
	{_app_Usb_OneKeyReset , "_app_Usb_OneKeyReset "},//15
	{_app_Usb_GotoSleep , "_app_Usb_GotoSleep "},//16
	{_app_Usb_WriteSN1 , "_app_Usb_WriteSN1 "},//17
	{_app_Usb_ReadSN1 , "_app_Usb_ReadSN1 "},//18
	{_app_Usb_WriteSN2 , "_app_Usb_WriteSN2 "},//19
	{_app_Usb_ReadSN2 , "_app_Usb_ReadSN2 "},//20
	{_app_Usb_WriteSramData , "_app_Usb_WriteSramData "},//21
	{_app_Usb_ReadSramData , "_app_Usb_ReadSramData "},//22
	{_app_Usb_FlashInit , "_app_Usb_FlashInit "},//23
	{NULL , "_app_Usb_WriteGainAvr "},//24
	{NULL , "_app_Usb_ReadGainAvr "},//25
	{_app_Usb_ReadFrame_Then_PmuStatus , "_app_Usb_ReadFrame_Then_PmuStatus "},//26
	{NULL , "_app_Usb_PbEnroll "},//27
	{NULL , "_app_Usb_PbVerify "},//28
	{NULL , "_app_Usb_PbDelFinger "},//29
	{_app_Usb_Send8bitData , "_app_Usb_CancelState "},//30
	{_app_Usb_Send8bitData , "_app_Usb_Send8bitData "},//31
	{NULL , "_app_Usb_ListFingers "},//32
	{NULL , "_app_Usb_DowloadImageEnroll "},//33
	{NULL , "_app_Usb_DowloadImageVerify "},//34
	{NULL , "_app_Usb_Send8bitPbData "},//35
	{NULL , "_app_Usb_GetPbAlgoData "},//36
	{NULL , "_app_Usb_SetCaliFrameData "},//37
	{NULL , "_app_Usb_reserve38 "},//38
	{NULL , "_app_Usb_reserve39 "},//38
	{_app_Usb_SetImageSpec , "_app_Usb_SetImageSpec "},//38
	{_app_Usb_GetImageSpec , "_app_Usb_GetImageSpec "},//38
};

void APP_Usb_ProcessCmd(uint8_t *CmdBuf)
{
	uint16_t  CmdIndex;
	uint8_t   Arg[56];
	size_t freeMemerySize = 0;

	memcpy(ResBuf,CmdBuf,56);
	
	if(memcmp(ResBuf,IDDATA,6) == 0)
	{
	///	DBG("ID is 0xFF\n");
		memcpy(ResBuf,Chip_ID,6);
		memcpy(ResBuf+8,Chip_ID,6);
		OTG_DeviceBulkSend(0x82,ResBuf,64,10);
	///	DBG("Chip_ID: %02X %02X %02X %02X %02X %02X\n" \
			,Chip_ID[0], Chip_ID[1],Chip_ID[2],Chip_ID[3],Chip_ID[4],Chip_ID[5]);
		return;
	}
	
	CmdIndex = ResBuf[6];
	CmdIndex = (CmdIndex<<8) + ResBuf[7];
	memcpy(Arg,ResBuf+8,56);
	if(CmdIndex != 10)
	{
	///	DBG("CmdIndex=%d\n",CmdIndex);
	}
///	DBG("CmdIndex = %d\r\n", CmdIndex);
	
	//freeMemerySize = xPortGetFreeHeapSize();
	//DBG("after malloc SIZE = %d",freeMemerySize);
	if(mv_shell[CmdIndex].CmdHandler != NULL)
	{
		mv_shell[CmdIndex].CmdHandler(Arg);
	}
	else
	{
	#if LOG_TOT_EN
		DBG("CmdIndex = %d is error !!!\r\n", CmdIndex);
	#endif
	}

}

void App_Usb_ProcessDataPackage(void)
{
	static bool sgbFlagUsbInit = FALSE;
	
	if(sgbFlagUsbInit == FALSE)
	{
		sgbFlagUsbInit = TRUE;
		UsbInit();
		ReadChipId();
	}
	
	Vendor_DeviceRequestProcess();
	
	if(REG_OTG_INTRRX == 8)
	{
		gu32Len = 0;
		OTG_DeviceBulkReceive(3,gu8CmdBuf,64,&gu32Len,1);
		if(gu32Len == 64)
		{
			APP_Usb_ProcessCmd(gu8CmdBuf);
		}
	}

}

void App_Usb_ResEssentialInfo(void)
{
	g_stImageSpecInfo.u32Width = 128;
	g_stImageSpecInfo.u32Height = 140;
	g_stImageSpecInfo.u32Start_X = 2;
	g_stImageSpecInfo.u32Start_Y = 0;
	g_stImageSpecInfo.u32Len_X = 126;
	g_stImageSpecInfo.u32Len_Y = 140;
	g_stImageSpecInfo.u32Pack8BitCnt = (g_stImageSpecInfo.u32Len_X*g_stImageSpecInfo.u32Len_Y+4095)/4096;
	g_stImageSpecInfo.u32Pack16BitCnt = (g_stImageSpecInfo.u32Width*g_stImageSpecInfo.u32Height*2+4095)/4096;
	g_stImageSpecInfo.u32PackGainCnt = g_stImageSpecInfo.u32Pack16BitCnt;
}


