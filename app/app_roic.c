#include <stdio.h>
#include <nds32_intrinsic.h>
#include "cache_tcm.h"
#include "gpio.h"
#include "uarts.h"
#include "uarts_interface.h"
#include "type.h"
#include "watchdog.h"
#include "debug.h"
#include "spi_flash.h"
#include "timeout.h"
#include "clk.h"
#include "pwm.h"
#include "sleep.h"
#include "delay.h"
#include "rtc.h"
#include "spim.h"
#include "irqn.h"
#include "oxifp_type_define.h"
#include "dev_pwm.h"
#include "data.h"
#include "pwm.h"
#include "pwc.h"
#include "dma.h"
#include "data.h"

#include "drv_config.h"

#include "dev_roic.h"
#include "dev_dma.h"
#include "dev_gpio.h"

#include "app_spi.h"

#define SPI_SYNC_BYTE 0xEB

///uint16_t i = 0,j = 0;
static uint16_t oeEnablePos = 0;
static uint8_t gainBuf[512];

uint16_t* ptrImage1 = 0;
uint16_t* ptrImage2 = 0;
uint16_t* ptrImage = 0;
uint8_t* ptrGain = 0;

int OnchipFlashBlockErase(uint32_t Page_Address,int size)
{
	int block_count,i,block_index;
	
	block_count=size/(64*1024);
	block_index=Page_Address/(64*1024);
	
#if LOG_TOT_EN
	DBG("block count:%d\n",block_count);
#endif

	for(i = 0; i < block_count; i++)
	{
		SpiFlashErase(BLOCK_ERASE,block_index+i,0);	
	}
}

int OnchipFlashErase(uint32_t Page_Address,int size)
{
	int sector,i,sector_num;
	sector=size/4096;
	sector_num=Page_Address/4096;
	for(i = 0;i < sector; i++)
	{
		SpiFlashErase(SECTOR_ERASE,sector_num+i,0);
	}
}

int OnchipFlashRead(uint32_t Page_Address,u8* ptr,int size)
{
	return SpiFlashRead(Page_Address,ptr,size,1000);
}
int OnchipFlashWrite(uint32_t Page_Address,u8* ptr,int size)
{
	int ret;
	ret= SpiFlashWrite(Page_Address,ptr,size,100);
	return ret;
}

typedef struct{
	uint32_t dMax;
	uint32_t dMin;
	uint32_t dFactor;
	uint32_t iWinLevel;
} dFactor_Agrs_t;
dFactor_Agrs_t gridAgrs[6];
static uint32_t dFactor = 0;
static uint32_t dMin = 0, dMax = 0;
uint8_t gainBuffer[256];
static unsigned int gainAddress = 0;

void App_Roic_ParaInit(void)
{
///	SPI_FLASH_INFO*  FlashInfo;	
	
/// FlashInfo = SpiFlashInfoGet();
#if LOG_TOT_EN
	DBG("Mid=%02X, did=%04XDDD\n",FlashInfo->Mid, FlashInfo->Did);
#endif
		
///	if(1)
///	{
	///	g_stClrFrmAfterClkWave.IntegralFramLineCnt = 270;
	///	g_stClrFrmAfterClkWave.CaptureFramLineCnt = 270;

	///	OnchipFlashErase(CLEAR_WAVEFORM_PARA_ADDR, 4096);
	///	OnchipFlashWrite(CLEAR_WAVEFORM_PARA_ADDR, (u8* )(&g_stClrFrmAfterClkWave), sizeof(g_stClrFrmAfterClkWave));
	#if LOG_TOT_EN
		DBG("write default para clr fram123\n");
	#endif
///	}

///	if(1)
///	{
		g_stCaptrAfterClkWave.IntegralFramLineCnt = INTEGRAL_LINE;//1380;//200ms 690;//100ms 759;//110ms;;//150ms 828;//120ms587;//85ms;//587;//150ms////1380;//600;///1380;//800;//500;
	///	g_stCaptrAfterClkWave.CaptureFramLineCnt = 270;
	
	///	OnchipFlashErase(CAPTURE_WAVEFORM_PARA_ADDR, 4096);
	///	OnchipFlashWrite(CAPTURE_WAVEFORM_PARA_ADDR, (u8* )(&g_stCaptrAfterClkWave), sizeof(g_stCaptrAfterClkWave));
	#if LOG_TOT_EN
		DBG("write default para capture fram\n");
	#endif
///	}
		
}



