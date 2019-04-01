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

#include "dev_dma.h"
#include "dev_roic.h"
#include "dev_gpio.h"

#include "app_spi.h"
#include "app_roic.h"

/*package header 2 bytes+identify 1 byte + pack No 2 bytes pack end 2 bytes*/
#define SPIS_DMA_RECV_TOT_LEN			(4*1024+7)
#define SPIS_DMA_RECV_DATA_LEN			(4*1024)
#define GAIN_PACK_MAX					55
#define RECV_GAIN_OUTTIME				10000/*5s*/

#define SEND_DATA_OVERTIME				5000/*unit:ms*/
#define RESPONSE_CMD_OVERTIME			1000//5000

EN_SPIS_RECV_STATUS g_enSpisRecvStus = EN_SPIS_RECV_NULL;
volatile ST_SPIS_ITEM gstSPIS_RX_Queue;
volatile ST_SPIS_REC_CMD_HEADER gstSPIS_CMD_Header;

static bool g_sbFlagRecvCmdTimeCntEn = FALSE;
volatile bool g_vbFlagRecvDMAErr = FALSE;
static bool g_sbFlagFristCapture = TRUE;
u32 g_u32RecvDataLen = 0;
volatile u32 g_vu32WaitCmdTick;
volatile bool g_sbFlagGetCmd = FALSE;

ST_WINDOW_INFO g_stWindowInfo_P1;
ST_WINDOW_INFO g_stWindowInfo_P2;

u8 u8RO1Buf[ROIC_WIDTH*2],u8RO2Buf[ROIC_WIDTH*2],u8RO3Buf[ROIC_WIDTH*2],u8RO4Buf[ROIC_WIDTH*2];

const u8 cu8HeaderBuf[2] = {0xEF, 0x01};
const u8 cu8EndBuf[2] = {0xAA, 0x00};

static void Jigsaw_Image(u8* ImageBuf, u32 Width, u32 Height)
{
	u32 i,j,k;
	u32 RowCnt = 0;
	u8* pu8TmpBuf;
	volatile u32 vu32RmDMASpcace;

	
	pu8TmpBuf = &TransferBuffer[2];
	memcpy(TransferBuffer, cu8HeaderBuf, 2);	
	memcpy(&TransferBuffer[25*1024+2],cu8EndBuf, 2);
	for(i=0; i<Height; i++)
	{
	///	memset(TransferBuffer, 0x00, Width);
		RowCnt++;
		for(j=0; j<Width/4; j++)
		{
			pu8TmpBuf[((RowCnt-1)%25)*1024+j] = (ImageBuf[i*Width + j]&0x05) | ((ImageBuf[i*Width + j]&0x50)>>3) \
								| ((ImageBuf[i*Width+256 +j]&0x05)<<4) |((ImageBuf[i*Width+256 +j]&0x50)<<1);
			
			pu8TmpBuf[((RowCnt-1)%25)*1024+256+j] = ((ImageBuf[i*Width +j]&0x0A)>>1) | ((ImageBuf[i*Width+ j]&0xA0)>>4) \
								| ((ImageBuf[i*Width+256 +j]&0x0A)<<3) |(ImageBuf[i*Width+256 +j]&0xA0);

			pu8TmpBuf[((RowCnt-1)%25)*1024+512+j] = (ImageBuf[i*Width+512 +j]&0x05) | ((ImageBuf[i*Width+512 + j]&0x50)>>3) \
								| ((ImageBuf[i*Width+768+j]&0x05)<<4) |((ImageBuf[i*Width + 768+j]&0x50)<<1);
			
			pu8TmpBuf[((RowCnt-1)%25)*1024+768+j] = ((ImageBuf[i*Width+512 +j]&0x0A)>>1) | ((ImageBuf[i*Width+512 +j]&0xA0)>>4) \
								| ((ImageBuf[i*Width + 768+j]&0x0A)<<3) |(ImageBuf[i*Width + 768+j]&0xA0);
			
		}
		
		for(k=0; k<Width; k++)
		{
			UART2_SendByte(TransferBuffer[k]);
		}
	}
}

void jigsaw_process(u16 x1_start, u16 y1_start, u16 x2_start, u16 y2_start, u8 duble_finger_en
									, ST_WINDOW_INFO* pstP1WindowInfo, ST_WINDOW_INFO* pstP2WindowInfo)
{
	u32 RealStarX1,RealStarX2;

	if(x1_start>= 0 && x1_start <= 98)
	{
		RealStarX1 = x1_start + COl_START;
	}
	else if(x1_start>= 99 && x1_start <= 224)
	{
		RealStarX1 = x1_start + COl_START+2;
	}
	else if(x1_start>= 225 && x1_start <= 350)
	{
		RealStarX1 = x1_start + COl_START+2+2;
	}
	else if(x1_start>= 351 && x1_start <= 449)
	{
		RealStarX1 = x1_start + COl_START+2+2+2;
	}
	
	memset(pstP1WindowInfo, 0x00, sizeof(pstP1WindowInfo));

	pstP1WindowInfo->u8FrstICNo = RealStarX1/ROIC_WIDTH;
	pstP1WindowInfo->u8FrstICStrt = RealStarX1%ROIC_WIDTH;
	pstP1WindowInfo->u8FrstICLen = ROIC_WIDTH-1-pstP1WindowInfo->u8FrstICStrt;
	if(pstP1WindowInfo->u8FrstICNo != 3)/*it is not the last ROIC*/
	{
		pstP1WindowInfo->u8ScndICNo = pstP1WindowInfo->u8FrstICNo+1;
		if(WINDOW_X > pstP1WindowInfo->u8FrstICLen+ROIC_WIDTH-2)
		{
			pstP1WindowInfo->u8ScndICLen = ROIC_WIDTH-2;
			pstP1WindowInfo->u8ThirdICNo = pstP1WindowInfo->u8ScndICNo+1;
			pstP1WindowInfo->u8ThirdICLen = WINDOW_X - pstP1WindowInfo->u8FrstICLen - pstP1WindowInfo->u8ScndICLen;
		}
		else
		{
			pstP1WindowInfo->u8ScndICLen = WINDOW_X - pstP1WindowInfo->u8FrstICLen;
		}
	}	

	pstP1WindowInfo->u16StartRow = y1_start + ROW_START;
	pstP1WindowInfo->u16EndRow = pstP1WindowInfo->u16StartRow + WINDOW_Y -1;

	if(duble_finger_en)
	{
		if(pstP2WindowInfo)
		{
				
			if(x2_start>= 0 && x2_start <= 98)
			{
				RealStarX2 = x2_start + COl_START;
			}
			else if(x2_start>= 99 && x2_start <= 224)
			{
				RealStarX2 = x2_start + COl_START+2;
			}
			else if(x2_start>= 225 && x2_start <= 350)
			{
				RealStarX2 = x2_start + COl_START+2+2;
			}
			else if(x2_start>= 351 && x2_start <= 449)
			{
				RealStarX2 = x2_start + COl_START+2+2+2;
			}
			
			memset(pstP2WindowInfo, 0x00, sizeof(pstP2WindowInfo));

			pstP2WindowInfo->u8FrstICNo = RealStarX2/ROIC_WIDTH;
			pstP2WindowInfo->u8FrstICStrt = RealStarX2%ROIC_WIDTH;
			pstP2WindowInfo->u8FrstICLen = ROIC_WIDTH-1-pstP1WindowInfo->u8FrstICStrt;
			if(pstP2WindowInfo->u8FrstICNo != 3)/*it is not the last ROIC*/
			{
				pstP2WindowInfo->u8ScndICNo = pstP2WindowInfo->u8FrstICNo+1;
				if(WINDOW_X > pstP2WindowInfo->u8FrstICLen+ROIC_WIDTH-2)
				{
					pstP2WindowInfo->u8ScndICLen = ROIC_WIDTH-2;
					pstP2WindowInfo->u8ThirdICNo = pstP2WindowInfo->u8ScndICNo+1;
					pstP2WindowInfo->u8ThirdICLen = WINDOW_X - pstP2WindowInfo->u8FrstICLen - pstP2WindowInfo->u8ScndICLen;
				}
				else
				{
					pstP2WindowInfo->u8ScndICLen = WINDOW_X - pstP2WindowInfo->u8FrstICLen;
				}
			}	

			pstP2WindowInfo->u16StartRow = y2_start + ROW_START;
			pstP2WindowInfo->u16EndRow = pstP2WindowInfo->u16StartRow + WINDOW_Y -1;
		}
	}
}

static void sApp_SPIS_IntRecCMDProcess(u8 para)
{
	u8 u8TempRes;
	u8 len,i;
	/*it will be wrong if these are not static type,because
	the value will be clear to zero in next calling*/
	static u8 u8DevAdressCounter = 0;
	static u32 u32PakegeSize;
	
	switch(g_enSpisRecvStus)
	{
		case EN_SPIS_RECV_NULL:
			if(para == 0xEF) // pakage header frist byte
			{
				g_enSpisRecvStus = EN_SPIS_RECV_HEADER_INCOMP;
				gstSPIS_RX_Queue.u16PushIndex = 0;
				gstSPIS_RX_Queue.u8SpisRxBuf[gstSPIS_RX_Queue.u16PushIndex++] =  para;
				gstSPIS_RX_Queue.u16RecCounter = 1;
				g_sbFlagRecvCmdTimeCntEn = TRUE;
				g_vu32WaitCmdTick = gSysTick;
			}
			break;
		
		case EN_SPIS_RECV_HEADER_INCOMP:
			if(para == 0x01)
			{
				g_enSpisRecvStus = EN_SPIS_RECV_START;
				gstSPIS_RX_Queue.u8SpisRxBuf[gstSPIS_RX_Queue.u16PushIndex++] =  para;
				gstSPIS_RX_Queue.u16RecCounter++;
			}
			else
			{
				g_enSpisRecvStus = EN_SPIS_RECV_NULL;
				gstSPIS_RX_Queue.u16PushIndex = 0;
				gstSPIS_RX_Queue.u16RecCounter = 0;
				g_sbFlagRecvCmdTimeCntEn = FALSE;
			}
			break;
			
		case EN_SPIS_RECV_START:
			if(gstSPIS_RX_Queue.u16RecCounter >= 5)
			{
				gstSPIS_RX_Queue.u8SpisRxBuf[gstSPIS_RX_Queue.u16PushIndex++] = para;
				gstSPIS_CMD_Header.u32Length = (u16)gstSPIS_RX_Queue.u8SpisRxBuf[3]<<8 |\
														gstSPIS_RX_Queue.u8SpisRxBuf[4];
				gstSPIS_RX_Queue.u16RecCounter++;
				if((gstSPIS_RX_Queue.u16RecCounter-5) ==  gstSPIS_CMD_Header.u32Length)
				{
					g_enSpisRecvStus = EN_SPIS_RECV_FINISH;
					SPIS_RX_DMA_MODE_SET(0);
				}
			}
			else
			{
				gstSPIS_RX_Queue.u8SpisRxBuf[gstSPIS_RX_Queue.u16PushIndex++] = para;
				gstSPIS_RX_Queue.u16RecCounter++;
			}
			break;
		default:
			break;
	}
	
}

void InterruptSpisDmaRecvCmd(void)
{
	uint32_t len,i;
	u8 u8TempRecv;

	g_sbFlagGetCmd = 1;
	len = DMA_CircularDataLenGet(PERIPHERAL_ID_SPIS_RX);
	
	for(i=0; i< len; i++)
	{
		DMA_CircularDataGet(PERIPHERAL_ID_SPIS_RX, &u8TempRecv, 1);
		sApp_SPIS_IntRecCMDProcess(u8TempRecv);
	}
	
	DMA_InterruptFlagClear(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT);
}

static void sApp_SPIS_IntRecDataProcess(u8 para)
{
	u8 u8TempRes;
	u8 len,i;
	/*it will be wrong if these are not static type,because
	the value will be clear to zero in next calling*/
	static u32 u32RecvByteCnt;
	
	switch(g_enSpisRecvStus)
	{
		case EN_SPIS_RECV_NULL:
			if(para == 0xEF) // pakage header frist byte
			{
				u32RecvByteCnt = 0;
				g_enSpisRecvStus = EN_SPIS_RECV_HEADER_INCOMP;
				TransferBuffer[u32RecvByteCnt++] = para;
			}
			break;
		
		case EN_SPIS_RECV_HEADER_INCOMP:
			if(para == 0x01)
			{
				g_enSpisRecvStus = EN_SPIS_RECV_START;
				TransferBuffer[u32RecvByteCnt++] = para;
			}
			else
			{
				g_vbFlagRecvDMAErr = TRUE;
				u32RecvByteCnt = 0;
				g_enSpisRecvStus = EN_SPIS_RECV_NULL;
			}
			break;
			
		case EN_SPIS_RECV_START:
			TransferBuffer[u32RecvByteCnt++] = para;
			if(u32RecvByteCnt >= (4096+7))
			{
				g_enSpisRecvStus = EN_SPIS_RECV_FINISH;
			}
			break;
			
		default:
			break;
	}
	
}

void InterruptSpisDmaRecvData(void)
{
	uint32_t len,i;
	u8 u8TempRecv;

	g_sbFlagGetCmd = 1;
	len = DMA_CircularDataLenGet(PERIPHERAL_ID_SPIS_RX);
	
	for(i=0; i< len; i++)
	{
		DMA_CircularDataGet(PERIPHERAL_ID_SPIS_RX, &u8TempRecv, 1);
		sApp_SPIS_IntRecDataProcess(u8TempRecv);
	}
	
	DMA_InterruptFlagClear(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT);
}

static void sApp_Spis_Clr_RecCmdPara(void)
{
	gstSPIS_RX_Queue.u16PopIndex = 0;
	gstSPIS_RX_Queue.u16PushIndex = 0;
	gstSPIS_RX_Queue.u16RecCounter = 0;
	g_enSpisRecvStus = EN_SPIS_RECV_NULL;
}

static void sApp_Spis_RespondCmd(u8 Result, u8 ParaLen, u8* ParaBuf)
{
	u8* pResBuf = NULL;
	u16 u16PackageLen;
	u16 u16Checksum = 0;
	u16 i;
	volatile u32 vu32LocalTick,vu32RmDMASpcace;

	/*	
		package header(2 bytes) + package ID(1 byte) + 
		package length(2 bytes) + result ID(1 byte) +
		respond parameters(ParaLen bytes) + checksum(2 bytes)
	*/
	pResBuf = malloc(2+1+2+1+ParaLen+2);
	if(pResBuf == NULL)
	{
	#if LOG_TOT_EN
		DBG("sApp_Uart_RespondCmd malloc pResBuf %d failed\n",(6+1+2+1+ParaLen+2));
		DBG("left free Ram size:%d\n",xPortGetFreeHeapSize());
	#endif
		while(1);
	}
	memcpy(pResBuf, cu8HeaderBuf, sizeof(cu8HeaderBuf));
	pResBuf[2] = 0x07;/*package respond CMD ID*/
	u16PackageLen = 1+ParaLen+2;/*u16PackageLen = result ID(1 byte) + ParaLen + Checksum(2 bytes)*/
	pResBuf[3] = (u8)(u16PackageLen>>8);/*package length*/
	pResBuf[4] = (u8)(u16PackageLen&0xFF);/*package length*/
	pResBuf[5] = Result;/*assume success*/ 
	if(ParaLen)/*paraLen > 0*/
	{
		memcpy(&pResBuf[6], ParaBuf, ParaLen);
	}
	u16Checksum = Checksum16(&pResBuf[2], (4+ParaLen));
	pResBuf[6+ParaLen] = (u8)(u16Checksum>>8);/*package length*/
	pResBuf[7+ParaLen] = (u8)(u16Checksum&0xFF);/*package length*/
	DMA_CircularDataPut(PERIPHERAL_ID_SPIS_TX, pResBuf, 8+ParaLen);
	
#if OPTIMAL_RESPONSE_EN
	vu32LocalTick = gSysTick;
	while(1)
	{
		vu32RmDMASpcace = DMA_CircularDataLenGet(PERIPHERAL_ID_SPIS_TX);
		if(vu32RmDMASpcace == 0)
		{
		#if LOG_TOT_EN
		 	DBG("send data finish\n");
		#endif
			break;
		}

		if(gSysTick - vu32LocalTick >RESPONSE_CMD_OVERTIME)
		{
		#if LOG_TOT_EN
			DBG("response CMD pack %d timeover\n",i);
		#endif
			break;
		}
	}
#endif

	free(pResBuf);
}

static u8 sApp_Spis_SendData(EN_SEND_PROT_TYPE enProtType, u32 RowLen, u8* SrcBuf, u32 PackSize)
{
	u32 u32SgnPackSize, u32PackNo,i;
	volatile u32 vu32LocalTick,vu32RmDMASpcace;
	volatile bool vbFlagTimeover = FALSE;
	
	if(enProtType == EN_PROT_NOML)
	{
		u32SgnPackSize = ((25*1024)/RowLen)*RowLen;
		u32PackNo = (PackSize + (u32SgnPackSize - 1))/u32SgnPackSize;
		for(i=0; i<u32PackNo; i++)
		{
			if(i != u32PackNo - 1)
			{
				DMA_CircularDataPut_EX(&frameBuffer[i*u32SgnPackSize], u32SgnPackSize);
			}
			else if((i == u32PackNo - 1))
			{
				DMA_CircularDataPut_EX(&frameBuffer[i*u32SgnPackSize], (PackSize-i*u32SgnPackSize));
			}
			
			vu32LocalTick = gSysTick;
			while(1)
			{
				vu32RmDMASpcace = DMA_CircularSpaceLenGet(PERIPHERAL_ID_SPIS_TX);
				if(vu32RmDMASpcace> u32SgnPackSize)
				{
					break;
				}

				if(gSysTick - vu32LocalTick >SEND_DATA_OVERTIME)
				{
				#if LOG_TOT_EN
					DBG("capture image send data 1 timeover\n");
				#endif
					vbFlagTimeover = TRUE;
					break;
				}
			}

			if(vbFlagTimeover == TRUE)
			{
				break;;
			}
		}

		if(vbFlagTimeover == TRUE)
		{
			return 0;
		}
	}
	else if(enProtType == EN_PROT_SIMPLY)
	{
		u32SgnPackSize = 25*1024;
		u32PackNo = (PackSize + (u32SgnPackSize -1))/u32SgnPackSize;
		memcpy(TransferBuffer, cu8HeaderBuf, 2);
		memcpy(&TransferBuffer[25*1024+2],cu8EndBuf, 2); 
		
		for(i=0; i<u32PackNo; i++)
		{
			memcpy(&TransferBuffer[2], &frameBuffer[i*u32SgnPackSize], u32SgnPackSize);
			DMA_CircularDataPut_EX(TransferBuffer, (u32SgnPackSize+4));
			
			vu32LocalTick = gSysTick;
			while(1)
			{
				vu32RmDMASpcace = DMA_CircularSpaceLenGet(PERIPHERAL_ID_SPIS_TX);
				if(vu32RmDMASpcace> u32SgnPackSize)
				{
					break;
				}

				if(gSysTick - vu32LocalTick >SEND_DATA_OVERTIME)
				{
				#if LOG_TOT_EN
					DBG("capture image send data 1 timeover\n");
				#endif
					vbFlagTimeover = TRUE;
					break;
				}
			}

			if(vbFlagTimeover == TRUE)
			{
				break;;
			}
		}

		if(vbFlagTimeover == TRUE)
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}

	return 1;
}


/*1st window get single ROIC image data*/
static u16 sApp_Spis_SingleICReadW1(DMA_SRC_BUF_TYPE enDmaBufType 
											,u16 BufOffset, u8 ROIC, u32 StaPoint, u32 EnPoint)
{
	u16 tmp1, tmp2, u16BufOffset, ByteCnt;
	u32 u32StrtPoin,u32EndPoin,i;

	u16BufOffset = BufOffset;
	u32StrtPoin = StaPoint;
	u32EndPoin = EnPoint;
	ByteCnt = 0;

	if(enDmaBufType == EN_SRC_BUF1) 
	{
		if((ROIC == 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x05)|((srcRevBuf1[i+1]&0x05)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x05)|((srcRevBuf1[i+3]&0x05)<<1);
				resData00[BufOffset + ByteCnt] =	tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{

				tmp1 =	((srcRevBuf1[i]&0x0A)>>1)|((srcRevBuf1[i+1]&0x0A));
				tmp2 =	((srcRevBuf1[i+2]&0x0A)>>1)|((srcRevBuf1[i+3]&0x0A));
				resData00[BufOffset+ ByteCnt] = tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x0050)|((srcRevBuf1[i+1]&0x0050)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x0050)|((srcRevBuf1[i+3]&0x0050)<<1);
				resData00[BufOffset+ ByteCnt] = (tmp1>>4)|(tmp2&0xf0);
				ByteCnt++;
			}
		}
		else if(ROIC == 3)
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf1[i]&0x120)>>1)|((srcRevBuf1[i+1]&0x120));
				tmp2 =	((srcRevBuf1[i+2]&0x120)>>1)|((srcRevBuf1[i+3]&0x120));
				resData00[BufOffset+ ByteCnt] = ((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				ByteCnt++;
			}
		}
	}
	else if(enDmaBufType == EN_SRC_BUF2)
	{
		if((ROIC == 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x05)|((srcRevBuf2[i+1]&0x05)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x05)|((srcRevBuf2[i+3]&0x05)<<1);
				resData00[BufOffset + ByteCnt] =	tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{

				tmp1 =	((srcRevBuf2[i]&0x0A)>>1)|((srcRevBuf2[i+1]&0x0A));
				tmp2 =	((srcRevBuf2[i+2]&0x0A)>>1)|((srcRevBuf2[i+3]&0x0A));
				resData00[BufOffset+ ByteCnt] = tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x0050)|((srcRevBuf2[i+1]&0x0050)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x0050)|((srcRevBuf2[i+3]&0x0050)<<1);
				resData00[BufOffset+ ByteCnt] = (tmp1>>4)|(tmp2&0xf0);
				ByteCnt++;
			}
		}
		else if(ROIC == 3)
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf2[i]&0x120)>>1)|((srcRevBuf2[i+1]&0x120));
				tmp2 =	((srcRevBuf2[i+2]&0x120)>>1)|((srcRevBuf2[i+3]&0x120));
				resData00[BufOffset+ ByteCnt] = ((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				ByteCnt++;
			}
		}
	}

	return ByteCnt;
}

/*2st window get single ROIC image data*/
static u16 sApp_Spis_SingleICReadW2(DMA_SRC_BUF_TYPE enDmaBufType 
												,u16 BufOffset , u8 ROIC, u32 StaPoint, u32 EnPoint)
{
	u16 tmp1, tmp2, u16BufOffset, ByteCnt;
	u32 u32StrtPoin,u32EndPoin,i;

	u16BufOffset = BufOffset;
	u32StrtPoin = StaPoint;
	u32EndPoin = EnPoint;
	ByteCnt = 0;

	if(enDmaBufType == EN_SRC_BUF1)	
	{
		if((ROIC == 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x05)|((srcRevBuf1[i+1]&0x05)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x05)|((srcRevBuf1[i+3]&0x05)<<1);
				resData11[BufOffset + ByteCnt] =	tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{

				tmp1 =	((srcRevBuf1[i]&0x0A)>>1)|((srcRevBuf1[i+1]&0x0A));
				tmp2 =	((srcRevBuf1[i+2]&0x0A)>>1)|((srcRevBuf1[i+3]&0x0A));
				resData11[BufOffset+ ByteCnt] =	tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x0050)|((srcRevBuf1[i+1]&0x0050)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x0050)|((srcRevBuf1[i+3]&0x0050)<<1);
				resData11[BufOffset+ ByteCnt] =	(tmp1>>4)|(tmp2&0xf0);
				ByteCnt++;
			}
		}
		else if(ROIC == 3)
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf1[i]&0x120)>>1)|((srcRevBuf1[i+1]&0x120));
				tmp2 =	((srcRevBuf1[i+2]&0x120)>>1)|((srcRevBuf1[i+3]&0x120));
				resData11[BufOffset+ ByteCnt] =	((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				ByteCnt++;
			}
		}
	}
	else if(enDmaBufType == EN_SRC_BUF2)
	{
		if((ROIC == 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x05)|((srcRevBuf2[i+1]&0x05)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x05)|((srcRevBuf2[i+3]&0x05)<<1);
				resData11[BufOffset + ByteCnt] =	tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{

				tmp1 =	((srcRevBuf2[i]&0x0A)>>1)|((srcRevBuf2[i+1]&0x0A));
				tmp2 =	((srcRevBuf2[i+2]&0x0A)>>1)|((srcRevBuf2[i+3]&0x0A));
				resData11[BufOffset+ ByteCnt] =	tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x0050)|((srcRevBuf2[i+1]&0x0050)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x0050)|((srcRevBuf2[i+3]&0x0050)<<1);
				resData11[BufOffset+ ByteCnt] =	(tmp1>>4)|(tmp2&0xf0);
				ByteCnt++;
			}
		}
		else if(ROIC == 3)
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf2[i]&0x120)>>1)|((srcRevBuf2[i+1]&0x120));
				tmp2 =	((srcRevBuf2[i+2]&0x120)>>1)|((srcRevBuf2[i+3]&0x120));
				resData11[BufOffset+ ByteCnt] =	((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				ByteCnt++;
			}
		}
	}

	return ByteCnt;
}

/*two window get image data with two ROIC*/
static u8 sApp_Spis_DoubleICRead(DMA_SRC_BUF_TYPE enDmaBufType,u16 W1BufOffset,  
											u16 W2BufOffset, u8 W1ROIC,u8 W2ROIC ,u32 StaPoint, u32 EnPoint)
{
	u16 tmp1, tmp2, u16W1BufOffset ,u16W2BufOffset, ByteCnt;
	u32 u32StrtPoin,u32EndPoin,i;

	u16W1BufOffset = W1BufOffset;
	u16W2BufOffset = W2BufOffset;
	u32StrtPoin = StaPoint;
	u32EndPoin = EnPoint;
	ByteCnt = 0;

	if(enDmaBufType == EN_SRC_BUF1)	
	{
		if((W1ROIC == 0)&&(W2ROIC== 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x05)|((srcRevBuf1[i+1]&0x05)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x05)|((srcRevBuf1[i+3]&0x05)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);
				resData11[u16W2BufOffset+ByteCnt] =  resData00[u16W1BufOffset+ByteCnt];
				ByteCnt++;
			}
		}
		else if((W1ROIC == 0)&&(W2ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x05)|((srcRevBuf1[i+1]&0x05)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x05)|((srcRevBuf1[i+3]&0x05)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);

				tmp1 =	((srcRevBuf1[i]&0x0A)>>1)|((srcRevBuf1[i+1]&0x0A));
				tmp2 =	((srcRevBuf1[i+2]&0x0A)>>1)|((srcRevBuf1[i+3]&0x0A));
				resData11[u16W2BufOffset+ByteCnt] =  tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 0)&&(W2ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x0055)|((srcRevBuf1[i+1]&0x0055)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x0055)|((srcRevBuf1[i+3]&0x0055)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	(tmp1&0xf)|(tmp2<<4);
				resData11[u16W2BufOffset+ByteCnt] =  (tmp1>>4)|(tmp2&0xf0);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 0)&&(W2ROIC == 3))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x125)|((srcRevBuf1[i+1]&0x125)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x125)|((srcRevBuf1[i+3]&0x125)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	(tmp1&0xf)|(tmp2<<4);
				resData11[u16W2BufOffset+ByteCnt] =	((tmp1>>5)&0x03)|
											((tmp1>>6)&0x0c)|
											((tmp2>>1)&0x30)|
											((tmp2>>2)&0xc0);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 1)&&(W2ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf1[i]&0x0A)>>1)|((srcRevBuf1[i+1]&0x0A));
				tmp2 =	((srcRevBuf1[i+2]&0x0A)>>1)|((srcRevBuf1[i+3]&0x0A));
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);
				resData11[u16W2BufOffset+ByteCnt] =  resData00[u16W1BufOffset+ByteCnt];
				ByteCnt++;
			}
		}
		else if((W1ROIC == 1)&&(W2ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf1[i]&0x0A)>>1)|((srcRevBuf1[i+1]&0x0A));
				tmp2 =	((srcRevBuf1[i+2]&0x0A)>>1)|((srcRevBuf1[i+3]&0x0A));
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);
				tmp1 =	(srcRevBuf1[i]&0x50)|((srcRevBuf1[i+1]&0x50)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x50)|((srcRevBuf1[i+3]&0x50)<<1);
				resData11[u16W2BufOffset+ByteCnt] =  (tmp1>>4)|tmp2;
				ByteCnt++;
			}
		}
		else if((W1ROIC == 1)&&(W2ROIC == 3))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf1[i]&0x12A)>>1)|((srcRevBuf1[i+1]&0x12A));
				tmp2 =	((srcRevBuf1[i+2]&0x12A)>>1)|((srcRevBuf1[i+3]&0x12A));
				resData00[u16W1BufOffset+ByteCnt] =	(tmp1&0xf)|(tmp2<<4);
				resData11[u16W2BufOffset+ByteCnt] =	((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 2)&&(W2ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x50)|((srcRevBuf1[i+1]&0x50)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x50)|((srcRevBuf1[i+3]&0x50)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	(tmp1>>4)|tmp2;
				resData11[u16W2BufOffset+ByteCnt] =  resData00[u16W1BufOffset+ByteCnt];
				ByteCnt++;
			}
		}
		else if((W1ROIC == 2)&&(W2ROIC == 3))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf1[i]&0x50))|((srcRevBuf1[i+1]&0x50)<<1);
				tmp2 =	((srcRevBuf1[i+2]&0x50))|((srcRevBuf1[i+3]&0x50)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	(tmp1>>4)|tmp2;
				tmp1 =	((srcRevBuf1[i]&0x120)>>1)|((srcRevBuf1[i+1]&0x120));
				tmp2 =	((srcRevBuf1[i+2]&0x120)>>1)|((srcRevBuf1[i+3]&0x120));
				resData11[u16W2BufOffset+ByteCnt] =  ((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 3)&&(W2ROIC == 3))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf1[i]&0x120)>>1)|((srcRevBuf1[i+1]&0x120));
				tmp2 =	((srcRevBuf1[i+2]&0x120)>>1)|((srcRevBuf1[i+3]&0x120));
				resData00[u16W1BufOffset+ByteCnt] =	((tmp1>>4)&0x03)|
								((tmp1>>5)&0x0c)|
								((tmp2)&0x30)	|
								((tmp2>>1)&0xc0);
				resData11[u16W2BufOffset+ByteCnt] =  resData00[u16W1BufOffset+ByteCnt];
				ByteCnt++;
			}

		}
		else if((W1ROIC == 1)&&(W2ROIC == 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf1[i]&0x0A)>>1)|((srcRevBuf1[i+1]&0x0A));
				tmp2 =	((srcRevBuf1[i+2]&0x0A)>>1)|((srcRevBuf1[i+3]&0x0A));
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);

				tmp1 =	(srcRevBuf1[i]&0x05)|((srcRevBuf1[i+1]&0x05)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x05)|((srcRevBuf1[i+3]&0x05)<<1);
				resData11[u16W2BufOffset+ByteCnt] =  tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 2)&&(W2ROIC == 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x0055)|((srcRevBuf1[i+1]&0x0055)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x0055)|((srcRevBuf1[i+3]&0x0055)<<1);
				resData00[u16W1BufOffset+ByteCnt] = (tmp1>>4)|(tmp2&0xf0);
				resData11[u16W2BufOffset+ByteCnt] = (tmp1&0xf)|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 2)&&(W2ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x50)|((srcRevBuf1[i+1]&0x50)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x50)|((srcRevBuf1[i+3]&0x50)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);
				tmp1 =	((srcRevBuf1[i]&0x0A)>>1)|((srcRevBuf1[i+1]&0x0A));
				tmp2 =	((srcRevBuf1[i+2]&0x0A)>>1)|((srcRevBuf1[i+3]&0x0A));
				resData11[u16W2BufOffset+ByteCnt] =  (tmp1>>4)|tmp2;
				ByteCnt++;
			}
		}
		else if((W1ROIC == 3)&&(W2ROIC == 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf1[i]&0x125)|((srcRevBuf1[i+1]&0x125)<<1);
				tmp2 =	(srcRevBuf1[i+2]&0x125)|((srcRevBuf1[i+3]&0x125)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	((tmp1>>5)&0x03)|
											((tmp1>>6)&0x0c)|
											((tmp2>>1)&0x30)|
											((tmp2>>2)&0xc0);
				resData11[u16W2BufOffset+ByteCnt] =	(tmp1&0xf)|(tmp2<<4);
		
				ByteCnt++;
			}
		}
		else if((W1ROIC == 3)&&(W2ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf1[i]&0x12A)>>1)|((srcRevBuf1[i+1]&0x12A));
				tmp2 =	((srcRevBuf1[i+2]&0x12A)>>1)|((srcRevBuf1[i+3]&0x12A));
				resData00[u16W1BufOffset+ByteCnt] =	((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				resData11[u16W2BufOffset+ByteCnt] =(tmp1&0xf)|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 3)&&(W2ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf1[i]&0x120)>>1)|((srcRevBuf1[i+1]&0x120));
				tmp2 =	((srcRevBuf1[i+2]&0x120)>>1)|((srcRevBuf1[i+3]&0x120));
				resData00[u16W1BufOffset+ByteCnt] =  ((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				tmp1 =	((srcRevBuf1[i]&0x50))|((srcRevBuf1[i+1]&0x50)<<1);
				tmp2 =	((srcRevBuf1[i+2]&0x50))|((srcRevBuf1[i+3]&0x50)<<1);
				resData11[u16W2BufOffset+ByteCnt] =	(tmp1>>4)|tmp2;
				ByteCnt++;
			}
		}
	}
	else if(enDmaBufType == EN_SRC_BUF2)
	{
		if((W1ROIC == 0)&&(W2ROIC== 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x05)|((srcRevBuf2[i+1]&0x05)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x05)|((srcRevBuf2[i+3]&0x05)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);
				resData11[u16W2BufOffset+ByteCnt] =  resData00[u16W1BufOffset+ByteCnt];
				ByteCnt++;
			}
		}
		else if((W1ROIC == 0)&&(W2ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x05)|((srcRevBuf2[i+1]&0x05)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x05)|((srcRevBuf2[i+3]&0x05)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);

				tmp1 =	((srcRevBuf2[i]&0x0A)>>1)|((srcRevBuf2[i+1]&0x0A));
				tmp2 =	((srcRevBuf2[i+2]&0x0A)>>1)|((srcRevBuf2[i+3]&0x0A));
				resData11[u16W2BufOffset+ByteCnt] =  tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 0)&&(W2ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x0055)|((srcRevBuf2[i+1]&0x0055)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x0055)|((srcRevBuf2[i+3]&0x0055)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	(tmp1&0xf)|(tmp2<<4);
				resData11[u16W2BufOffset+ByteCnt] =  (tmp1>>4)|(tmp2&0xf0);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 0)&&(W2ROIC == 3))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x125)|((srcRevBuf2[i+1]&0x125)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x125)|((srcRevBuf2[i+3]&0x125)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	(tmp1&0xf)|(tmp2<<4);
				resData11[u16W2BufOffset+ByteCnt] =	((tmp1>>5)&0x03)|
											((tmp1>>6)&0x0c)|
											((tmp2>>1)&0x30)|
											((tmp2>>2)&0xc0);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 1)&&(W2ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf2[i]&0x0A)>>1)|((srcRevBuf2[i+1]&0x0A));
				tmp2 =	((srcRevBuf2[i+2]&0x0A)>>1)|((srcRevBuf2[i+3]&0x0A));
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);
				resData11[u16W2BufOffset+ByteCnt] =  resData00[u16W1BufOffset+ByteCnt];
				ByteCnt++;
			}
		}
		else if((W1ROIC == 1)&&(W2ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf2[i]&0x0A)>>1)|((srcRevBuf2[i+1]&0x0A));
				tmp2 =	((srcRevBuf2[i+2]&0x0A)>>1)|((srcRevBuf2[i+3]&0x0A));
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);
				tmp1 =	(srcRevBuf2[i]&0x50)|((srcRevBuf2[i+1]&0x50)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x50)|((srcRevBuf2[i+3]&0x50)<<1);
				resData11[u16W2BufOffset+ByteCnt] =  (tmp1>>4)|tmp2;
				ByteCnt++;
			}
		}
		else if((W1ROIC == 1)&&(W2ROIC == 3))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf2[i]&0x12A)>>1)|((srcRevBuf2[i+1]&0x12A));
				tmp2 =	((srcRevBuf2[i+2]&0x12A)>>1)|((srcRevBuf2[i+3]&0x12A));
				resData00[u16W1BufOffset+ByteCnt] =	(tmp1&0xf)|(tmp2<<4);
				resData11[u16W2BufOffset+ByteCnt] =	((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 2)&&(W2ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x50)|((srcRevBuf2[i+1]&0x50)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x50)|((srcRevBuf2[i+3]&0x50)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	(tmp1>>4)|tmp2;
				resData11[u16W2BufOffset+ByteCnt] =  resData00[u16W1BufOffset+ByteCnt];
				ByteCnt++;
			}
		}
		else if((W1ROIC == 2)&&(W2ROIC == 3))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf2[i]&0x50))|((srcRevBuf2[i+1]&0x50)<<1);
				tmp2 =	((srcRevBuf2[i+2]&0x50))|((srcRevBuf2[i+3]&0x50)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	(tmp1>>4)|tmp2;
				tmp1 =	((srcRevBuf2[i]&0x120)>>1)|((srcRevBuf2[i+1]&0x120));
				tmp2 =	((srcRevBuf2[i+2]&0x120)>>1)|((srcRevBuf2[i+3]&0x120));
				resData11[u16W2BufOffset+ByteCnt] =  ((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 3)&&(W2ROIC == 3))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf2[i]&0x120)>>1)|((srcRevBuf2[i+1]&0x120));
				tmp2 =	((srcRevBuf2[i+2]&0x120)>>1)|((srcRevBuf2[i+3]&0x120));
				resData00[u16W1BufOffset+ByteCnt] =	((tmp1>>4)&0x03)|
								((tmp1>>5)&0x0c)|
								((tmp2)&0x30)	|
								((tmp2>>1)&0xc0);
				resData11[u16W2BufOffset+ByteCnt] =  resData00[u16W1BufOffset+ByteCnt];
				ByteCnt++;
			}

		}
		else if((W1ROIC == 1)&&(W2ROIC == 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf2[i]&0x0A)>>1)|((srcRevBuf2[i+1]&0x0A));
				tmp2 =	((srcRevBuf2[i+2]&0x0A)>>1)|((srcRevBuf2[i+3]&0x0A));
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);

				tmp1 =	(srcRevBuf2[i]&0x05)|((srcRevBuf2[i+1]&0x05)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x05)|((srcRevBuf2[i+3]&0x05)<<1);
				resData11[u16W2BufOffset+ByteCnt] =  tmp1|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 2)&&(W2ROIC == 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x0055)|((srcRevBuf2[i+1]&0x0055)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x0055)|((srcRevBuf2[i+3]&0x0055)<<1);
				resData00[u16W1BufOffset+ByteCnt] = (tmp1>>4)|(tmp2&0xf0);
				resData11[u16W2BufOffset+ByteCnt] = (tmp1&0xf)|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 2)&&(W2ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x50)|((srcRevBuf2[i+1]&0x50)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x50)|((srcRevBuf2[i+3]&0x50)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	tmp1|(tmp2<<4);
				tmp1 =	((srcRevBuf2[i]&0x0A)>>1)|((srcRevBuf2[i+1]&0x0A));
				tmp2 =	((srcRevBuf2[i+2]&0x0A)>>1)|((srcRevBuf2[i+3]&0x0A));
				resData11[u16W2BufOffset+ByteCnt] =  (tmp1>>4)|tmp2;
				ByteCnt++;
			}
		}
		else if((W1ROIC == 3)&&(W2ROIC == 0))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	(srcRevBuf2[i]&0x125)|((srcRevBuf2[i+1]&0x125)<<1);
				tmp2 =	(srcRevBuf2[i+2]&0x125)|((srcRevBuf2[i+3]&0x125)<<1);
				resData00[u16W1BufOffset+ByteCnt] =	((tmp1>>5)&0x03)|
											((tmp1>>6)&0x0c)|
											((tmp2>>1)&0x30)|
											((tmp2>>2)&0xc0);
				resData11[u16W2BufOffset+ByteCnt] =	(tmp1&0xf)|(tmp2<<4);
		
				ByteCnt++;
			}
		}
		else if((W1ROIC == 3)&&(W2ROIC == 1))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf2[i]&0x12A)>>1)|((srcRevBuf2[i+1]&0x12A));
				tmp2 =	((srcRevBuf2[i+2]&0x12A)>>1)|((srcRevBuf2[i+3]&0x12A));
				resData00[u16W1BufOffset+ByteCnt] =	((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				resData11[u16W2BufOffset+ByteCnt] =(tmp1&0xf)|(tmp2<<4);
				ByteCnt++;
			}
		}
		else if((W1ROIC == 3)&&(W2ROIC == 2))
		{
			for(i=u32StrtPoin; i<u32EndPoin; i+=4)
			{
				tmp1 =	((srcRevBuf2[i]&0x120)>>1)|((srcRevBuf2[i+1]&0x120));
				tmp2 =	((srcRevBuf2[i+2]&0x120)>>1)|((srcRevBuf2[i+3]&0x120));
				resData00[u16W1BufOffset+ByteCnt] =  ((tmp1>>4)&0x03)|
										   ((tmp1>>5)&0x0c)|
										   ((tmp2)&0x30)   |
										   ((tmp2>>1)&0xc0);
				tmp1 =	((srcRevBuf2[i]&0x50))|((srcRevBuf2[i+1]&0x50)<<1);
				tmp2 =	((srcRevBuf2[i+2]&0x50))|((srcRevBuf2[i+3]&0x50)<<1);
				resData11[u16W2BufOffset+ByteCnt] =	(tmp1>>4)|tmp2;
				ByteCnt++;
			}
		}
	}

	return ByteCnt;
}

static u8 sApp_Spis_CaptureWin(bool IsTwoWin, DMA_SRC_BUF_TYPE enDmaBufType, 
										ST_WINDOW_INFO* pstWin1Info, ST_WINDOW_INFO* pstWin2Info)
{
	u8 u8ICNo1,u8ICNo2;
	u16 tmp1,tmp2,u16W1Start,u16W2Start,u16BufOffset, u16TempOffset;
	u32 i,j,k,u32StrtPoin,u32EndPoin;
	
	/*parameter check*/
	if((enDmaBufType != EN_SRC_BUF1)&&(enDmaBufType != EN_SRC_BUF2))
	{
#if LOG_TOT_EN
		DBG("buf type para err\n");
#endif
		return 0;
	}

	if((pstWin1Info == NULL)  \
		||((IsTwoWin == TRUE)&&(pstWin2Info == NULL)))
	{
#if LOG_TOT_EN
		DBG("win info para err\n");
#endif
		return 0;
	}

	if(enDmaBufType == EN_SRC_BUF1)
	{
		for(k=0; k<MEMORY_BYTE_LEN; k++)
		{
			if(srcRevBuf1[k]&0x400)/*DE is high level means is captureing data now*/
				break;
		}
	}
	else if(enDmaBufType == EN_SRC_BUF2)
	{
		for(k=0; k<MEMORY_BYTE_LEN; k++)
		{
			if(srcRevBuf2[k]&0x400)/*DE is high level means is captureing data now*/
				break;
		}
	}

	if(IsTwoWin == FALSE)
	{
		/*merge part 1:left dammy line*/
		u32StrtPoin = (L_DARKLINE_START%128)*8+k;
		u32EndPoin = ((L_DARKLINE_END%128)+1)*8+k;
		u8ICNo1 = 0;
		u16TempOffset = sApp_Spis_SingleICReadW1(enDmaBufType,u16W1Start, u8ICNo1, u32StrtPoin,u32EndPoin);
		u16W1Start +=u16TempOffset;
		
		/*merge part 2: frist segment data*/
		u32StrtPoin = pstWin1Info->u8FrstICStrt*8+k;
		u32EndPoin = (pstWin1Info->u8FrstICStrt+pstWin1Info->u8FrstICLen)*8+k;
		u8ICNo1 = pstWin1Info->u8FrstICNo;
		u16TempOffset = sApp_Spis_SingleICReadW1(enDmaBufType,u16W1Start, u8ICNo1, u32StrtPoin,u32EndPoin);
		u16W1Start +=u16TempOffset;		
				
		/*merge part 3: second segment data*/
		u32StrtPoin = 1*8+k;
		u32EndPoin = (1+pstWin1Info->u8ScndICLen)*8+k;
		u8ICNo1 = pstWin1Info->u8ScndICNo;
		u16TempOffset = sApp_Spis_SingleICReadW1(enDmaBufType,u16W1Start, u8ICNo1, u32StrtPoin,u32EndPoin);
		u16W1Start +=u16TempOffset;
		
		/*merge part 4: third segment data(maybe none)*/
		if(pstWin1Info->u8ThirdICLen != 0)
		{
			u32StrtPoin = 1*8+k;
			u32EndPoin = (1+pstWin1Info->u8ThirdICLen)*8+k;
			u8ICNo1 = pstWin1Info->u8ThirdICNo;
			u16TempOffset = sApp_Spis_SingleICReadW1(enDmaBufType,u16W1Start, u8ICNo1, u32StrtPoin,u32EndPoin);
			u16W1Start +=u16TempOffset;			
		}
		
		/*merge part 5:right dammy line*/
		u32StrtPoin = (R_DARKLINE_START%128)*8+k;
		u32EndPoin = (R_DARKLINE_END%128+1)*8+k;
		u8ICNo1 = 3;
		u16TempOffset = sApp_Spis_SingleICReadW1(enDmaBufType,u16W1Start, u8ICNo1, u32StrtPoin,u32EndPoin);
		u16W1Start +=u16TempOffset;
	}
	else if(IsTwoWin == TRUE)
	{
		u32StrtPoin = L_DARKLINE_START*8+k;
		u32EndPoin = (L_DARKLINE_END+1)*8+k;
		u8ICNo1 = 0;
		u8ICNo2 = 0;
		u16W1Start = 0;
		u16W2Start = 0;
		sApp_Spis_DoubleICRead(enDmaBufType, u16W1Start, u16W2Start, u8ICNo1, u8ICNo2,u32StrtPoin, u32EndPoin);
		
		u16BufOffset = ((L_DARKLINE_END+1)-L_DARKLINE_START)*2;/*input index offset in data buffer*/
		
		if(pstWin1Info->u8FrstICStrt <= pstWin2Info->u8FrstICStrt)
		{
			/*merge part 2:S1 -> S2(not include S2 point itself)*/
			u32StrtPoin = pstWin1Info->u8FrstICStrt*8+k;
			u32EndPoin = pstWin2Info->u8FrstICStrt*8+k;
			u8ICNo1 = pstWin1Info->u8FrstICNo;
			u8ICNo2 = pstWin2Info->u8ScndICNo;
			u16W1Start = u16BufOffset + 0;
			u16W2Start = u16BufOffset + ((pstWin1Info->u8FrstICStrt - 1) \
						+ (ROIC_WIDTH - 2 - pstWin2Info->u8FrstICStrt+1))*2;
			
			sApp_Spis_DoubleICRead(enDmaBufType, u16W1Start, u16W2Start, u8ICNo1, u8ICNo2,u32StrtPoin, u32EndPoin);
			
			/*merge part 3:S2 -> end(include s2 and end point itself)*/
			u32StrtPoin = pstWin2Info->u8FrstICStrt*8+k;
			u32EndPoin=1024-8 +k;/*last point is not the real data*/
			u8ICNo1 = pstWin1Info->u8FrstICNo;
			u8ICNo2 = pstWin2Info->u8FrstICNo;
			u16W1Start = u16BufOffset + (pstWin2Info->u8FrstICStrt - pstWin1Info->u8FrstICStrt)*2;
			u16W2Start = u16BufOffset + 0;
			sApp_Spis_DoubleICRead(enDmaBufType, u16W1Start, u16W2Start, u8ICNo1, u8ICNo2,u32StrtPoin, u32EndPoin);
			
			/*merge part 4: start -> S1(not include S1 point itself)*/
			u32StrtPoin = 8+ k;
			u32EndPoin = pstWin1Info->u8FrstICStrt*8 + k;
			u8ICNo1 = pstWin1Info->u8ScndICNo;
			u8ICNo2 = pstWin2Info->u8ScndICNo;
			u16W1Start = u16BufOffset + (ROIC_WIDTH-2 - pstWin1Info->u8FrstICStrt+1)*2;
			u16W2Start = u16BufOffset + (ROIC_WIDTH-2 - pstWin2Info->u8FrstICStrt+1)*2;				
			sApp_Spis_DoubleICRead(enDmaBufType, u16W1Start, u16W2Start, u8ICNo1, u8ICNo2,u32StrtPoin, u32EndPoin);

		}
		else if(pstWin1Info->u8FrstICStrt > pstWin2Info->u8FrstICStrt)
		{
			/*merge part 2:S2 -> S1(not include S1 point itself)*/
			u32StrtPoin = pstWin2Info->u8FrstICStrt*8+k;
			u32EndPoin = pstWin1Info->u8FrstICStrt*8+k;
			u8ICNo1 = pstWin1Info->u8ScndICNo;
			u8ICNo2 = pstWin2Info->u8FrstICNo;
			u16W1Start = u16BufOffset + ((pstWin2Info->u8FrstICStrt - 1) \
						+ (ROIC_WIDTH - 2 - pstWin1Info->u8FrstICStrt+1))*2;
			u16W2Start = u16BufOffset + 0;
			sApp_Spis_DoubleICRead(enDmaBufType, u16W1Start, u16W2Start, u8ICNo1, u8ICNo2,u32StrtPoin, u32EndPoin);
			
			/*merge part 3:S1 -> end(include s2 and end point itself)*/
			u32StrtPoin = pstWin1Info->u8FrstICStrt*8+k;
			u32EndPoin=1024-8 +k;/*last point is not the real data*/
			u8ICNo1 = pstWin1Info->u8FrstICNo;
			u8ICNo2 = pstWin2Info->u8FrstICNo;
			u16W1Start = u16BufOffset + 0;
			u16W2Start = u16BufOffset + (pstWin1Info->u8FrstICStrt - pstWin2Info->u8FrstICStrt)*2;
			
			sApp_Spis_DoubleICRead(enDmaBufType, u16W1Start, u16W2Start, u8ICNo1, u8ICNo2,u32StrtPoin, u32EndPoin);
			
			/*merge part 4: start -> S2(not include S2 point itself)*/
			u32StrtPoin = 8+ k;
			u32EndPoin = pstWin2Info->u8FrstICStrt*8 + k;
			u8ICNo1 = pstWin1Info->u8ScndICNo;
			u8ICNo2 = pstWin2Info->u8ScndICNo;
			u16W1Start = u16BufOffset + (ROIC_WIDTH-2 - pstWin1Info->u8FrstICStrt+1)*2;
			u16W2Start = u16BufOffset + (ROIC_WIDTH-2 - pstWin2Info->u8FrstICStrt+1)*2;
				
			sApp_Spis_DoubleICRead(enDmaBufType, u16W1Start, u16W2Start, u8ICNo1, u8ICNo2,u32StrtPoin, u32EndPoin);
		
			
		}

		/*merge part 5: last 2 point*/
		/*win1 last 2 point*/
	
		u32StrtPoin = pstWin1Info->u8FrstICStrt*8 + k;
		u32EndPoin = (pstWin1Info->u8FrstICStrt+2)*8 + k;
		u8ICNo1 = pstWin1Info->u8ScndICNo;
		u16W1Start = u16BufOffset + (ROIC_WIDTH-2)*2;
		sApp_Spis_SingleICReadW1(enDmaBufType,u16W1Start, u8ICNo1, u32StrtPoin,u32EndPoin);

		/*win2 last 2 point*/
		u32StrtPoin = pstWin2Info->u8FrstICStrt*8 + k;
		u32EndPoin = (pstWin2Info->u8FrstICStrt+2)*8 + k;
		u8ICNo2 = pstWin2Info->u8ScndICNo;
		u16W2Start = u16BufOffset + (ROIC_WIDTH-2)*2;
		sApp_Spis_SingleICReadW2(enDmaBufType,u16W2Start, u8ICNo2, u32StrtPoin,u32EndPoin);				
								
		/*merge part 6:right dammy line*/
		u32StrtPoin = (R_DARKLINE_START%128)*8+k;
		u32EndPoin = (R_DARKLINE_END%128+1)*8+k;			
		sApp_Spis_DoubleICRead(enDmaBufType, 138*2, 138*2, 3, 3,u32StrtPoin, u32EndPoin);
	}
}

static u8 sApp_Spis_ClrFrame(u16 FrameCnt)
{
	g_bFlagClrFramEn = TRUE;

	XAO_H();
/***capture image and send image data******************************/
	sampleCount = 0;
	dummyLines = 10;
	frameCount = FrameCnt;
	frameLines = 0;
	
	SET_PFIRST_0_L();
	PwcDmaConfig();
	PwmDmaConfig(EN_WAVE_CLR_FRAM);

	sampleFinishFlag = 0;
	samplebufIndex = 1;
	sampleCount = 0;
	SET_PFIRST_0_H();//step2 pirst0 变高  //lk
	PWM_Enable(PWM_TIMER);
	PWC_Enable(PWC_TIMER);
	HCLK_L();
	
	while(1)
	{
		if(sampleFinishFlag != 0)
		{
			if(sampleFinishFlag == 1)
			{
				sampleFinishFlag = 0;
				
			}
			else if(sampleFinishFlag == 2)
			{
				sampleFinishFlag = 0;
			}
			
			if( frameCount ==  0)
			{
				break;
			}
		}
	}	
	
	g_bFlagClrFramEn = FALSE;
}

void sApp_Spis_ClrFastScan(u16 FrameCnt)
{
	u16 i,j;
	u16 StvCovLTCnt = 52;
	
	XAO_H();
	for(i=0; i<FrameCnt; i++)
	{
		SCLK_PIRST_HI();
		HCLK_H();
		DelayUs(30);
		HSTV_H();
		DelayUs(30);
		HCLK_L();
		DelayUs(30);
	
		for(j=0; j<FRAME_LINE_TOT-1+ StvCovLTCnt; j++)
		{
			HCLK_H();
			DelayUs(30);
			if(j == StvCovLTCnt)/*STV high level width = x+1*/
			{
				HSTV_L();
			}
			
			HCLK_L();
			DelayUs(30);
		}
	
	///	SCLK_PIRST_LO();
		DelayUs(30);
		HCLK_H();
	}
}

static u8 sApp_Spis_ClrLag_BefCpt(void)
{
#if DEBUG_CLR_LAG_FAST_SCAN_EN
	sApp_Spis_ClrFastScan(1);
#else
	sApp_Spis_ClrFrame(1);
#endif

	Dev_GPIO_Vcom2Flt();
	DelayMs(2);


	/*Vcom*/
	Dev_GPIO_Vcom2Vcom();
	DelayMs(10);
	
#if DEBUG_CLR_LAG_FAST_SCAN_EN
	sApp_Spis_ClrFastScan(2);
#else
	sApp_Spis_ClrFrame(5);
#endif

}


static u8 sApp_Spis_ClrLag_AftCpt(void)
{
#if FUNC_MOS_CTRL_FPC_EN
	Dev_GPIO_Vcom2Vdd();
#else
	XAO_L();
	SET_PFIRST_0_H();
#endif
}

static u8 _app_Spis_CaptureImage(ST_SPIS_REC_CMD_HEADER* pstCmd) 
{
/*
	total 9 bytes:
	package header(2 bytes) + package identify(1 byte)
	+ package length(2 bytes) + row number(2 bytes) 
	+ row length(2 bytes)
*/
	u8 u8ResDataHeader[9];
///	u16 u16RecordBuf[360];
	volatile u16 vu16PacLen, vu16RowLen; 
	volatile u32 vu32RmDMASpcace;
	volatile u32 vu32LocalTick;
	uint32_t i,j,k,RowCnt;
	uint16_t tmp1,tmp2,tmp3,tmp4;
	uint32_t data;
	u8 u8ICNo1,u8ICNo2,u8FlagCntTime;
	u16 u16W1Start,u16W2Start;
	u8 ROIC1XLen,  ROIC2XLen,ROIC3XLen,ROIC4XLen; 
	u32 n,h,size,s,bb,u32StrtPoin,u32EndPoin, u32ByteCnt, u32TmpByteCnt,u32RandNum,Star1,Star2,Temp;
	volatile u32 x1=0;
	u32 CycleRecBuf[130],CycCnt;
	u16 u16P1StarX,u16P1StarY,u16P2StarX, u16P2StarY;
	u16 u16W1BufferOffset,u16W2BufferOffset,u16W1PackNo,u16W2PackNo;
	volatile bool vbFlagTimeover = FALSE;
	u16 u16BufOffset;
	DMA_SRC_BUF_TYPE enSrcBufType;

	u16P1StarX = (((u16)pstCmd->u8ParaBuf[1])<<8)|(pstCmd->u8ParaBuf[2]);
	u16P1StarY = (((u16)pstCmd->u8ParaBuf[3])<<8)|(pstCmd->u8ParaBuf[4]);
	u16P2StarX = (((u16)pstCmd->u8ParaBuf[5])<<8)|(pstCmd->u8ParaBuf[6]);
	u16P2StarY = (((u16)pstCmd->u8ParaBuf[7])<<8)|(pstCmd->u8ParaBuf[8]);


	memcpy(u8ResDataHeader, cu8HeaderBuf, 2);
	/*row number(2 bytes) + row length(2 bytes) + row data(512bytes) + end chars(2 bytes)*/
	vu16PacLen = 2 + 2+ 296 + 2;
	u8ResDataHeader[3] = vu16PacLen>>8;
	u8ResDataHeader[4] = (u8)vu16PacLen;
	vu16RowLen = 296;
	u8ResDataHeader[7] = vu16RowLen>>8;
	u8ResDataHeader[8] = (u8)vu16RowLen;
	DMA_CircularFIFOClear(PERIPHERAL_ID_SPIS_TX);

	XAO_H();

/***capture image and send image data******************************/
	sampleCount = 0;
	dummyLines = 10;//10;///5;//5;//10;;
	frameCount = NORMAL_CAPTURE_NUMBER;//2;//2;
	frameLines = 0;
	
	SET_PFIRST_0_L();//step0 pirst0=0
	PwcDmaConfig();
	PwmDmaConfig(EN_WAVE_CAPTR_IMAGE);
	sampleFinishFlag = 0;
	samplebufIndex = 1;
	sampleCount = 0;
	SET_PFIRST_0_H();//step2 pirst0 变高  //lk
	PWC_Enable(PWC_TIMER);
	PWM_Enable(PWM_TIMER);
	HCLK_L();
	RowCnt = 0;
	g_vu16LineCnt = 0;


	if(pstCmd->u8ParaBuf[0] == 1)/*one window*/
	{
		/*transfer the position to whole image*/
		jigsaw_process(u16P1StarX,u16P1StarY,0,0,0,&g_stWindowInfo_P1,NULL);
		u8FlagCntTime = 0;
		u16W1PackNo = 0;
		u16W1BufferOffset = 0;
	
		while(1)
		{
			if(sampleFinishFlag != 0)
			{
				 j = 0;
				if(sampleFinishFlag == 1)
				{
					enSrcBufType = EN_SRC_BUF1;
				}
				else if(sampleFinishFlag == 2)
				{
					enSrcBufType = EN_SRC_BUF2;
				}
				sampleFinishFlag = 0;

				__nds32__mtsr(0, NDS32_SR_PFMC0);
				__nds32__mtsr(1, NDS32_SR_PFM_CTL);

				
				sApp_Spis_CaptureWin(FALSE, enSrcBufType, &g_stWindowInfo_P1, NULL);
					
				if(dummyLines == 0 && frameLines <= (270+1+SHIFT_ROW)&& frameLines > 0 && frameCount == 1)
				{
					RowCnt++;
					if(((RowCnt>= TOP_DARKLINE_START+1+SHIFT_ROW) && (RowCnt<=(TOP_DARKLINE_END+1+SHIFT_ROW))) \
						|| ((RowCnt>= (g_stWindowInfo_P1.u16StartRow+1+SHIFT_ROW)) && (RowCnt<=(g_stWindowInfo_P1.u16EndRow+1+SHIFT_ROW))) \
						|| ((RowCnt>= (BOTTOM_DAKLINE_SATAR+1+SHIFT_ROW)) && (RowCnt<=(BOTTOM_DAKLINE_END+1+SHIFT_ROW))))
					{						
						u16W1PackNo++;
						u8ResDataHeader[5] = u16W1PackNo>>8;
						u8ResDataHeader[6] = (u8)u16W1PackNo;
					
						memcpy(&frameBuffer[u16W1BufferOffset*307], u8ResDataHeader, 9);
						memcpy(&frameBuffer[u16W1BufferOffset*307+9], resData00, 296);
						memcpy(&frameBuffer[u16W1BufferOffset*307+9+296], cu8EndBuf, 2);
						u16W1BufferOffset++;
					}
				}
				
				__nds32__mtsr(0, NDS32_SR_PFM_CTL);
			///	CycleRecBuf[CycCnt++] = __nds32__mfsr(NDS32_SR_PFMC0);
				
				if( frameCount ==  0)
				{
					break;
				}
			}

		}
		
		sApp_Spis_ClrLag_AftCpt();
		sApp_Spis_RespondCmd(0x00, 0, NULL);
		sApp_Spis_SendData(EN_PROT_NOML, 307,  frameBuffer, 307*148);
	}
	else if(pstCmd->u8ParaBuf[0] == 2)/*two window, not debug now*/
	{
		jigsaw_process(u16P1StarX,u16P1StarY,u16P2StarX,u16P2StarY,1,&g_stWindowInfo_P1,&g_stWindowInfo_P2);
	#if LOG_TOT_EN
		DBG("W1 1st IC:%d, W2 1st IC:%d\n",g_stWindowInfo_P1.u8FrstICNo, g_stWindowInfo_P2.u8FrstICNo);
		DBG("W1 2st IC:%d, W2 2st IC:%d\n",g_stWindowInfo_P1.u8ScndICNo, g_stWindowInfo_P2.u8ScndICNo);
		DBG("W1 1st Start:%d, W2 1st Start:%d\n",g_stWindowInfo_P1.u8FrstICStrt, g_stWindowInfo_P2.u8FrstICStrt);
		DBG("W1 Stat row:%d,end row:%d, W2 stat row:%d,end row:%d\n",g_stWindowInfo_P1.u16StartRow,g_stWindowInfo_P1.u16EndRow \
																	, g_stWindowInfo_P2.u16StartRow,g_stWindowInfo_P2.u16EndRow);
	#endif
		
		u16W1PackNo = 0;
		u16W2PackNo = 0;
		u16W1BufferOffset = 0;
		u16W2BufferOffset = 128;

		while(1)
		{
			if(sampleFinishFlag != 0)
			{
				if(sampleFinishFlag == 1)
				{
					enSrcBufType = EN_SRC_BUF1;
				}
				else if(sampleFinishFlag == 2)	
				{
					enSrcBufType = EN_SRC_BUF2;
				}
				sampleFinishFlag = 0;

				__nds32__mtsr(0, NDS32_SR_PFMC0);
				__nds32__mtsr(1, NDS32_SR_PFM_CTL);

				sApp_Spis_CaptureWin(TRUE, enSrcBufType, &g_stWindowInfo_P1, &g_stWindowInfo_P2);
				
				if(dummyLines == 0 && frameLines <= (270+SHIFT_ROW+1)&& frameLines > 0 && frameCount == 1)
				{
					RowCnt++;
					if((RowCnt>= (g_stWindowInfo_P1.u16StartRow+1+SHIFT_ROW)) && (RowCnt<=(g_stWindowInfo_P1.u16EndRow+1+SHIFT_ROW))) 
					{
						if(RowCnt == (g_stWindowInfo_P1.u16EndRow+1+SHIFT_ROW))
						{
							u8ResDataHeader[2] = 0x0F;
						}
						else
						{
							u8ResDataHeader[2] = 0x08;
						}
						
						u16W1PackNo++;
						u8ResDataHeader[5] = u16W1PackNo>>8;
						u8ResDataHeader[6] = (u8)u16W1PackNo;
						memcpy(&frameBuffer[u16W1BufferOffset*307], u8ResDataHeader, 9);
						memcpy(&frameBuffer[u16W1BufferOffset*307+9], resData00, 296);
						memcpy(&frameBuffer[u16W1BufferOffset*307+9+296], cu8EndBuf, 2);
						u16W1BufferOffset++;
					}

					if((RowCnt>= (g_stWindowInfo_P2.u16StartRow+1+SHIFT_ROW)) && (RowCnt<=(g_stWindowInfo_P2.u16EndRow+1+SHIFT_ROW))) 
					{
						if(RowCnt == (g_stWindowInfo_P2.u16EndRow+1+SHIFT_ROW))
						{
							u8ResDataHeader[2] = 0x0F;
						}
						else
						{
							u8ResDataHeader[2] = 0x08;
						}
						
						u16W2PackNo++;
						u8ResDataHeader[5] = u16W2PackNo>>8;
						u8ResDataHeader[6] = (u8)u16W2PackNo;
						memcpy(&frameBuffer[u16W2BufferOffset*307], u8ResDataHeader, 9);
						memcpy(&frameBuffer[u16W2BufferOffset*307+9], resData11, 296);
						memcpy(&frameBuffer[u16W2BufferOffset*307+9+296], cu8EndBuf, 2);
						u16W2BufferOffset++;
					}
				}
				
				__nds32__mtsr(0, NDS32_SR_PFM_CTL);
			//	CycleRecBuf[CycCnt+++] = __nds32__mfsr(NDS32_SR_PFMC0);
				
				if( frameCount ==  0)
				{
					break;
				}
			}
		}
		
		sApp_Spis_ClrLag_AftCpt();

		sApp_Spis_RespondCmd(0x00, 0, NULL);
		sApp_Spis_SendData(EN_PROT_NOML, 307,  frameBuffer, 256*307);
	}
	else/*window count parameter error*/
	{
	#if LOG_TOT_EN
		DBG("window count parameter error\n");
	#endif
		sApp_Spis_RespondCmd(0x01, 0, NULL);
	}
}

u8 _app_Spis_CaptureGainImage(ST_SPIS_REC_CMD_HEADER* pstCmd) 
{
/*
	total 9 bytes:
	package header(2 bytes) + package identify(1 byte)
	+ package length(2 bytes) + row number(2 bytes) 
	+ row length(2 bytes)
*/
	u8 u8ResDataHeader[9];
	u16 u16RecordBuf[360];
	volatile u16 vu16PacLen, vu16RowLen; 
	volatile u32 vu32RmDMASpcace;
	volatile u32 vu32LocalTick;
	uint32_t i,j,k,RowCnt;
	uint16_t tmp1,tmp2,tmp3,tmp4;
	uint32_t data;
	u8 u8ICNo1,u8ICNo2,u8FlagCntTime;
	u16 u16W1Start,u16W2Start;
	u8 ROIC1XLen,  ROIC2XLen,ROIC3XLen,ROIC4XLen; 
	u32 n,h,size,s,bb,u32StrtPoin,u32EndPoin, u32ByteCnt, u32TmpByteCnt,u32RandNum,Star1,Star2,Temp;
	volatile u32 x1=0;
	u8 u8PacIdenty = 0x08;
	volatile u16 vu16StartRowNo,vu16EndRowNo;
	volatile bool vbFlagTimeover = FALSE;
	
	if(pstCmd->u8ParaBuf[1] == 1)
	{
		vu16StartRowNo = 1;
		vu16EndRowNo = 125;
	}
	else if(pstCmd->u8ParaBuf[1] == 2)
	{
		vu16StartRowNo = 126;
		vu16EndRowNo = 250;
	}
	else
	{
		sApp_Spis_RespondCmd(0x01, 0, NULL);
		return 0;
	}
	memset(u16RecordBuf, 0xFF, sizeof(u16RecordBuf));
	memcpy(u8ResDataHeader, cu8HeaderBuf, 2);
	/*row number(2 bytes) + row length(2 bytes) + row data(512bytes) + end chars(2 bytes)*/
	vu16PacLen = 2 + 2+ 1024 + 2;
	u8ResDataHeader[3] = vu16PacLen>>8;
	u8ResDataHeader[4] = (u8)vu16PacLen;
	vu16RowLen = 1024;
	u8ResDataHeader[7] = vu16RowLen>>8;
	u8ResDataHeader[8] = (u8)vu16RowLen;
	DMA_CircularFIFOClear(PERIPHERAL_ID_SPIS_TX);
	 
	XAO_H();
/***capture image and send image data******************************/
	sampleCount = 0;
	dummyLines = 10;///10;/*如果将此值改为5，矣缅錚C工具一采图就会中断*/
	frameCount = NORMAL_CAPTURE_NUMBER;//2;//2;
	frameLines = 0;
	
	SET_PFIRST_0_L();//step0 pirst0=0
	PwcDmaConfig();
	PwmDmaConfig(EN_WAVE_CAPTR_IMAGE);

	sampleFinishFlag = 0;
	samplebufIndex = 1;
	sampleCount = 0;
	SET_PFIRST_0_H();//step2 pirst0 变高  //lk
	PWM_Enable(PWM_TIMER);
	PWC_Enable(PWC_TIMER);
///	HCLK_H();
	HCLK_L();
	RowCnt = 0;
	
	u8FlagCntTime = 0;
///	memset(CycleRecBuf, 0x00, sizeof(CycleRecBuf));
///	CycCnt = 0;

	while(1)
	{
		if(sampleFinishFlag != 0)
		{
			 j = 0;
			if(sampleFinishFlag == 1)
			{
				sampleFinishFlag = 0;
			///	TEST_PIN_H();

				__nds32__mtsr(0, NDS32_SR_PFMC0);
				__nds32__mtsr(1, NDS32_SR_PFM_CTL);
				
				for(k=0; k<MEMORY_BYTE_LEN; k++)/*cost 1000 cycles*/
				{
					if(srcRevBuf1[k]&0x400)/*DE is high level means is captureing data now*/
						break;
				}	
				
				for(i=k; i<1024+k; i += 4)
				{
					resData00[j] =	(srcRevBuf1[i]&0x0F)|((srcRevBuf1[i+1]&0x0F)<<4);
					resData11[j] =  (srcRevBuf1[i+2]&0x0F)|((srcRevBuf1[i+3]&0x0F)<<4);

					resData22[j] =  ((srcRevBuf1[i]&0x70)>>4)|((srcRevBuf1[i]&0x100)>>5) \
									|(srcRevBuf1[i+1]&0x70)|((srcRevBuf1[i+1]&0x100)>>1);

					resData33[j] =  ((srcRevBuf1[i+2]&0x70)>>4)|((srcRevBuf1[i+2]&0x100)>>5) \
									|(srcRevBuf1[i+3]&0x70)|((srcRevBuf1[i+3]&0x100)>>1);


					j++;
				}
				
			///	__nds32__mtsr(0, NDS32_SR_PFM_CTL);
			///	CycleRecBuf[CycCnt++] = __nds32__mfsr(NDS32_SR_PFMC0);
				
				if(dummyLines == 0 && frameLines <= FRAME_LINE_TOT && frameLines > 0 && frameCount == 1)
				{
					RowCnt++;
					if((RowCnt <= (vu16EndRowNo+ DAMMY_LINE_CNT+SHIFT_ROW))&&(RowCnt >= (vu16StartRowNo+DAMMY_LINE_CNT+SHIFT_ROW)))//180)
					{
						u16RecordBuf[RowCnt-SHIFT_ROW-1] = resData00[50];
						if(RowCnt-SHIFT_ROW == vu16EndRowNo)//180)
						{
							u8ResDataHeader[2] = 0x0F;
							u8ResDataHeader[5] = 0;
							u8ResDataHeader[6] = 125;
						}
						else
						{
							u8ResDataHeader[2] = 0x08;
							u8ResDataHeader[5] = ((RowCnt-SHIFT_ROW)%125)>>8;
							u8ResDataHeader[6] = (u8)((RowCnt-SHIFT_ROW)%125);
						}

						memcpy(&frameBuffer[((RowCnt-DAMMY_LINE_CNT-1-SHIFT_ROW)%125)*1024], resData00, 256);
						memcpy(&frameBuffer[((RowCnt-DAMMY_LINE_CNT-1-SHIFT_ROW)%125)*1024+256], resData11, 256);
						memcpy(&frameBuffer[((RowCnt-DAMMY_LINE_CNT-1-SHIFT_ROW)%125)*1024+512], resData22, 256);
						memcpy(&frameBuffer[((RowCnt-DAMMY_LINE_CNT-1-SHIFT_ROW)%125)*1024+768], resData33, 256);
					}
				}
				
				__nds32__mtsr(0, NDS32_SR_PFM_CTL);
			///	CycleRecBuf[CycCnt++] = __nds32__mfsr(NDS32_SR_PFMC0);
			}
			else if(sampleFinishFlag == 2)
			{
				sampleFinishFlag = 0;
				
				for(k=0; k<MEMORY_BYTE_LEN; k++)
				{
					if(srcRevBuf2[k]&0x400)/*DE is high level means is captureing data now*/
						break;
				}	
				
				for(i=k; i<k+1024; i+=4)
				{
					resData00[j] =	(srcRevBuf2[i]&0x0F)|(srcRevBuf2[i+1]&0x0F)<<4;
					resData11[j] =	(srcRevBuf2[i+2]&0x0F)|(srcRevBuf2[i+3]&0x0F)<<4;
					
					resData22[j] =	(srcRevBuf2[i]&0x70)>>4|(srcRevBuf2[i]&0x100)>>5 \
									|(srcRevBuf2[i+1]&0x70)|(srcRevBuf2[i+1]&0x100)>>1;
					
					resData33[j] =	(srcRevBuf2[i+2]&0x70)>>4|(srcRevBuf2[i+2]&0x100)>>5 \
									|(srcRevBuf2[i+3]&0x70)|(srcRevBuf2[i+3]&0x100)>>1;
					j++;
				}

				if(dummyLines == 0 && frameLines <= FRAME_LINE_TOT && frameLines > 0 && frameCount == 1)
				{
					RowCnt++;
					if((RowCnt <= (vu16EndRowNo+DAMMY_LINE_CNT+SHIFT_ROW))&&(RowCnt >= (vu16StartRowNo+DAMMY_LINE_CNT+SHIFT_ROW)))//180)
					{
						u16RecordBuf[RowCnt-SHIFT_ROW-1] = resData22[50];
						if(RowCnt-SHIFT_ROW == vu16EndRowNo)//180)
						{
							u8ResDataHeader[2] = 0x0F;
							u8ResDataHeader[5] = 0;
							u8ResDataHeader[6] = 125;
						}
						else
						{
							u8ResDataHeader[2] = 0x08;
							u8ResDataHeader[5] = (RowCnt-SHIFT_ROW%125)>>8;
							u8ResDataHeader[6] = (u8)(RowCnt-SHIFT_ROW%125);
						}
						
						memcpy(&frameBuffer[((RowCnt-DAMMY_LINE_CNT-1-SHIFT_ROW)%125)*1024], resData00, 256);
						memcpy(&frameBuffer[((RowCnt-DAMMY_LINE_CNT-1-SHIFT_ROW)%125)*1024+256], resData11, 256);
						memcpy(&frameBuffer[((RowCnt-DAMMY_LINE_CNT-1-SHIFT_ROW)%125)*1024+512], resData22, 256);
						memcpy(&frameBuffer[((RowCnt-DAMMY_LINE_CNT-1-SHIFT_ROW)%125)*1024+768], resData33, 256);
					}
				}
			}
			
			if( frameCount ==  0)
			{
				break;
			}
		}
	}
	
	sApp_Spis_ClrLag_AftCpt();
	sApp_Spis_RespondCmd(0x00, 0, NULL);	
	sApp_Spis_SendData(EN_PROT_SIMPLY, 0,  frameBuffer, 125*1024);
}

u8 _app_Spis_ClrFrame(ST_SPIS_REC_CMD_HEADER* pstCmd) 
{
	sApp_Spis_ClrFrame(3);
	sApp_Spis_RespondCmd(0x00, 0, NULL);/*respond*/
}

static u8 _app_Spis_WriteGain(ST_SPIS_REC_CMD_HEADER* pstCmd) 
{
/*
	total 9 bytes:
	package header(2 bytes) + package identify(1 byte)
	+ package length(2 bytes) + row number(2 bytes) 
	+ row length(2 bytes)
*/
	u8 u8ResDataHeader[9];
	u32 u32GainStartAddr;
	volatile u32 vu32RecvPackCnt,vu32LocalTick;
	u16 u16RecvPackNo;
	bool bFlagRecvSucss = FALSE;

#if LOG_TOT_EN
	DBG("write gain, para=%d\n",pstCmd->u8ParaBuf[1]);
#endif

	/*get info witch gain data to be written*/
	if(pstCmd->u8ParaBuf[1] == 1)
	{
		u32GainStartAddr = CALIDATA1_ONCHIP_ADDRESS;
	}
	else if(pstCmd->u8ParaBuf[1] == 2)
	{
		u32GainStartAddr = CALIDATA2_ONCHIP_ADDRESS;
	}
	else if(pstCmd->u8ParaBuf[1] == 3)
	{
		u32GainStartAddr = CALIDATA3_ONCHIP_ADDRESS;
	}
	else if(pstCmd->u8ParaBuf[1] == 4)
	{
		u32GainStartAddr = CALIDATA4_ONCHIP_ADDRESS;
	}
	else
	{
	#if LOG_TOT_EN
		DBG("para err\n");
	#endif
		sApp_Spis_RespondCmd(0x84, 0, NULL);/*respond para err*/
		return 0;
	}

	/*erase the address will be written*/
	OnchipFlashBlockErase(u32GainStartAddr,256*1024);
	g_enSpisRecvStus = EN_SPIS_RECV_NULL;

	/*reset the mapping function of DMA receive interrupt*/
	DMA_ChannelEnable(PERIPHERAL_ID_SPIS_RX, 0);
	SPIS_RX_DMA_MODE_SET(1);
	DMA_CircularConfig(PERIPHERAL_ID_SPIS_RX, 1, DmaSpisRecvFifo, SPIS_DMA_RECV_FIFO_LEN);
	DMA_InterruptFunSet(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT, InterruptSpisDmaRecvData);
	DMA_InterruptEnable(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT, 1);
	DMA_ChannelEnable(PERIPHERAL_ID_SPIS_RX, 1);
	DMA_CircularFIFOClear(PERIPHERAL_ID_SPIS_RX);
	vu32RecvPackCnt = 0;

	/*response host that module is ready for receiving data*/
	sApp_Spis_RespondCmd(0x00, 0, NULL);
	
	vu32LocalTick = gSysTick;
	while(1)
	{
		/*receive wrong data in package */
		if(g_vbFlagRecvDMAErr == TRUE)
		{
			g_vbFlagRecvDMAErr = FALSE;
		#if LOG_TOT_EN
			DBG("receive data len err,recv size:%d\n",g_u32RecvDataLen);
		#endif
			break;
		}

		/*one package data receive finish and success*/
		if(g_enSpisRecvStus == EN_SPIS_RECV_FINISH)
		{
			g_enSpisRecvStus = EN_SPIS_RECV_NULL;
			if(memcmp(TransferBuffer, cu8HeaderBuf,2 ) != 0)
			{
			#if LOG_TOT_EN
				DBG("recv header err; recv: %02X %02X\n",TransferBuffer[0],TransferBuffer[1]);
			#endif
				sApp_Spis_RespondCmd(0x84, 0, NULL);
				break;
			}
			
			/*pack header 2bytes + identify 1 byte + pack No 2 bytes*/
			if(memcmp(&TransferBuffer[5+SPIS_DMA_RECV_DATA_LEN], cu8EndBuf,2 ) != 0)
			{
			#if LOG_TOT_EN
				DBG("recv ender err; recv: %02X %02X\n",TransferBuffer[5+SPIS_DMA_RECV_DATA_LEN] \
														,TransferBuffer[5+SPIS_DMA_RECV_DATA_LEN+1]);
			#endif
				sApp_Spis_RespondCmd(0x84, 0, NULL);
				break;
			}

			/*analysis receive pacakage No*/
			u16RecvPackNo = (((u16)TransferBuffer[3])<<8)|(TransferBuffer[4]);
			vu32RecvPackCnt++;/*local record receive package No.*/
			
			if(TransferBuffer[2] == 0x0F)/*pacakge identify is "0x0F" means the last package*/
			{
				/*if it is last package,the package No. must be the specific figure*/
				if((vu32RecvPackCnt != u16RecvPackNo)||(vu32RecvPackCnt != GAIN_PACK_MAX) \
					||(u16RecvPackNo != GAIN_PACK_MAX))
				{
				#if LOG_TOT_EN
					DBG("recv last pack cnt err,recv cnt=%d,recv No=%d\n",vu32RecvPackCnt,u16RecvPackNo);
				#endif
					sApp_Spis_RespondCmd(0x84, 0, NULL);/*last No. error,respose to host the result*/
					break;
				}
				/*save the received data to flash*/
				OnchipFlashWrite(u32GainStartAddr + (vu32RecvPackCnt-1)*SPIS_DMA_RECV_DATA_LEN, 
									&TransferBuffer[5], SPIS_DMA_RECV_DATA_LEN);
			#if LOG_TOT_EN
				DBG("receive gain data success\n");
			#endif
				DMA_CircularFIFOClear(PERIPHERAL_ID_SPIS_RX);/*clear the DMA receive FIFO*/
				bFlagRecvSucss = TRUE;
				break;
			}
			else if(TransferBuffer[2] == 0x08)
			{
				if(vu32RecvPackCnt != u16RecvPackNo)
				{
				#if LOG_TOT_EN
					DBG("recv pack cnt err\n");
				#endif
					sApp_Spis_RespondCmd(0x84, 0, NULL);
					break;
				}
				OnchipFlashWrite(u32GainStartAddr + (vu32RecvPackCnt-1)*SPIS_DMA_RECV_DATA_LEN, 
									&TransferBuffer[5], SPIS_DMA_RECV_DATA_LEN);
			#if LOG_TOT_EN
				DBG("recv pac:st%d\n",vu32RecvPackCnt);
			#endif
				DMA_CircularFIFOClear(PERIPHERAL_ID_SPIS_RX);
				g_enSpisRecvStus = EN_SPIS_RECV_NULL;
				sApp_Spis_RespondCmd(0x00, 0, NULL);
				continue;
			}
			else
			{
			#if LOG_TOT_EN
				DBG("recv pack identify err\n");
			#endif
				sApp_Spis_RespondCmd(0x84, 0, NULL);
				break;
			}
		}

		/*check whether receive data timeover*/
		if(gSysTick - vu32LocalTick >= RECV_GAIN_OUTTIME)
		{
		#if LOG_TOT_EN
			DBG("receive gain data timeover\n");
		#endif
			sApp_Spis_RespondCmd(0x0F, 0, NULL);
			break;
		}
	}
	
	g_enSpisRecvStus = EN_SPIS_RECV_NULL;

	/*recover the DMA receive interrupt mapping function*/
	DMA_ChannelEnable(PERIPHERAL_ID_SPIS_RX, 0);
	SPIS_RX_DMA_MODE_SET(1);
	DMA_CircularConfig(PERIPHERAL_ID_SPIS_RX, SPIS_INT_LEN, DmaSpisRecvFifo, SPIS_DMA_RECV_FIFO_LEN);
	DMA_InterruptFunSet(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT, InterruptSpisDmaRecvCmd);
	DMA_InterruptEnable(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT, 1);
	DMA_ChannelEnable(PERIPHERAL_ID_SPIS_RX, 1);

	/*final,response to host receive gain data finish and success*/
	if(bFlagRecvSucss == TRUE)
	{
		sApp_Spis_RespondCmd(0x00, 0, NULL);
	}
}

static u8 _app_Spis_GetGain(ST_SPIS_REC_CMD_HEADER* pstCmd) 
{
/*
	total 9 bytes:
	package header(2 bytes) + package identify(1 byte)
	+ package length(2 bytes) + row number(2 bytes) 
	+ row length(2 bytes)
*/
	u8 u8ResDataHeader[9];
	u32 u32GainStartAddr;
	volatile u32 vu32RmDMASpcace;
	u16 u16RecvPackNo,i;
#if LOG_TOT_EN
	DBG("get gain,para=%d\n",pstCmd->u8ParaBuf[1]);
#endif

	if(pstCmd->u8ParaBuf[1] == 1)
	{
		u32GainStartAddr = CALIDATA1_ONCHIP_ADDRESS;
	}
	else if(pstCmd->u8ParaBuf[1] == 2)
	{
		u32GainStartAddr = CALIDATA2_ONCHIP_ADDRESS;
	}
	else if(pstCmd->u8ParaBuf[1] == 3)
	{
		u32GainStartAddr = CALIDATA3_ONCHIP_ADDRESS;
	}
	else if(pstCmd->u8ParaBuf[1] == 4)
	{
		u32GainStartAddr = CALIDATA4_ONCHIP_ADDRESS;
	}
	else
	{
	#if LOG_TOT_EN
		DBG("para err\n");
	#endif
		sApp_Spis_RespondCmd(0x84, 0, NULL);/*respond para err*/
		return 0;
	}
	
	OnchipFlashRead(u32GainStartAddr, frameBuffer, 25*1024);
	sApp_Spis_RespondCmd(0x00, 0, NULL);
	memcpy(TransferBuffer, cu8HeaderBuf, 2);
	memcpy(&TransferBuffer[25*1024+2],cu8EndBuf, 2);
	for(i=0; i<9; i++)
	{
		memcpy(&TransferBuffer[2], frameBuffer, 25*1024);
		DMA_CircularDataPut_EX(TransferBuffer,25*1024+4);
		if(i<8)
		{
			OnchipFlashRead(u32GainStartAddr+(i+1)*25*1024, frameBuffer, 25*1024);
		}
		while(1)
		{
			vu32RmDMASpcace = DMA_CircularSpaceLenGet(PERIPHERAL_ID_SPIS_TX);
			if(vu32RmDMASpcace> (25*1024+10))
			{
				break;
			}
		}
	}	

	while(1)
	{
		vu32RmDMASpcace = DMA_CircularDataLenGet(PERIPHERAL_ID_SPIS_TX);
		if(vu32RmDMASpcace == 0)
		{
		#if LOG_TOT_EN
			DBG("send gain finish\n");
		#endif
			break;
			
		}
	}
}

static u8 _app_Spis_UpdateCode(ST_SPIS_REC_CMD_HEADER* pstCmd) 
{
	u32 u32CodeSize, u32EraseSize;
	u16 u16RecvCodeCrc, u16CalCodeCrc;
	volatile u32 vu32RecvPackCnt,vu32LocalTick;
	u16 u16RecvPackNo;

	u8 Result = 0;

	u32CodeSize = (((u32)pstCmd->u8ParaBuf[0])<<24) |(((u32)pstCmd->u8ParaBuf[1])<<16) \
					|(((u32)pstCmd->u8ParaBuf[2])<<8) | pstCmd->u8ParaBuf[3];
	u16RecvCodeCrc = (((u16)pstCmd->u8ParaBuf[4])<<8) | pstCmd->u8ParaBuf[5];
	u32EraseSize = ((u32CodeSize+4095)/4096)*4096;
	OnchipFlashErase(UPDATE_FIRMWARE_ADDR, u32EraseSize);
	
	g_enSpisRecvStus = EN_SPIS_RECV_NULL;
	DMA_ChannelEnable(PERIPHERAL_ID_SPIS_RX, 0);
	SPIS_RX_DMA_MODE_SET(1);
	DMA_CircularConfig(PERIPHERAL_ID_SPIS_RX, 1, DmaSpisRecvFifo, SPIS_DMA_RECV_FIFO_LEN);
	DMA_InterruptFunSet(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT, InterruptSpisDmaRecvData);
	DMA_InterruptEnable(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT, 1);
	DMA_ChannelEnable(PERIPHERAL_ID_SPIS_RX, 1);
	DMA_CircularFIFOClear(PERIPHERAL_ID_SPIS_RX);
	vu32RecvPackCnt = 0;
	
	sApp_Spis_RespondCmd(0x00, 0, NULL);
	vu32LocalTick = gSysTick;
	while(1)
	{
		if(g_vbFlagRecvDMAErr == TRUE)
		{
			g_vbFlagRecvDMAErr = FALSE;
		#if LOG_TOT_EN
			DBG("receive data len err,recv size:%d\n",g_u32RecvDataLen);
		#endif
			break;
		}

		if(g_enSpisRecvStus == EN_SPIS_RECV_FINISH)
		{
			g_enSpisRecvStus = EN_SPIS_RECV_NULL;
			if(memcmp(TransferBuffer, cu8HeaderBuf,2 ) != 0)
			{
			#if LOG_TOT_EN
				DBG("recv header err; recv: %02X %02X\n",TransferBuffer[0],TransferBuffer[1]);
			#endif
				sApp_Spis_RespondCmd(0x84, 0, NULL);
				break;
			}
			
			/*pack header 2bytes + identify 1 byte + pack No 2 bytes*/
			if(memcmp(&TransferBuffer[5+SPIS_DMA_RECV_DATA_LEN], cu8EndBuf,2 ) != 0)
			{
			#if LOG_TOT_EN
				DBG("recv ender err; recv: %02X %02X\n",TransferBuffer[5+SPIS_DMA_RECV_DATA_LEN] \
														,TransferBuffer[5+SPIS_DMA_RECV_DATA_LEN+1]);
			#endif
				sApp_Spis_RespondCmd(0x84, 0, NULL);
				break;
			}

			u16RecvPackNo = (((u16)TransferBuffer[3])<<8)|(TransferBuffer[4]);
			vu32RecvPackCnt++;
			if(TransferBuffer[2] == 0x0F)
			{
				if(vu32RecvPackCnt != u16RecvPackNo)
				{
				#if LOG_TOT_EN
					DBG("recv last pack cnt err,recv cnt=%d,recv No=%d\n",vu32RecvPackCnt,u16RecvPackNo);
				#endif
					sApp_Spis_RespondCmd(0x84, 0, NULL);
					break;
				}
			///	memcpy(&frameBuffer[(vu32RecvPackCnt-1)*SPIS_DMA_RECV_DATA_LEN], &TransferBuffer[5],SPIS_DMA_RECV_DATA_LEN);?
				OnchipFlashWrite(UPDATE_FIRMWARE_ADDR + (vu32RecvPackCnt-1)*SPIS_DMA_RECV_DATA_LEN, 
									&TransferBuffer[5], SPIS_DMA_RECV_DATA_LEN);
			#if LOG_TOT_EN
				DBG("receive code data success,last package No:%d\n",vu32RecvPackCnt);
			#endif
				DMA_CircularFIFOClear(PERIPHERAL_ID_SPIS_RX);
				sApp_Spis_RespondCmd(0x00, 0, NULL);
				Result = 1;
				break;
			}
			else if(TransferBuffer[2] == 0x08)
			{
				if(vu32RecvPackCnt != u16RecvPackNo)
				{
				#if LOG_TOT_EN
					DBG("recv pack cnt err\n");
				#endif
					sApp_Spis_RespondCmd(0x84, 0, NULL);
					break;
				}
				
				OnchipFlashWrite(UPDATE_FIRMWARE_ADDR + (vu32RecvPackCnt-1)*SPIS_DMA_RECV_DATA_LEN, 
									&TransferBuffer[5], SPIS_DMA_RECV_DATA_LEN);
			#if LOG_TOT_EN
				DBG("recv pac:st%d\n",vu32RecvPackCnt);
			#endif
				DMA_CircularFIFOClear(PERIPHERAL_ID_SPIS_RX);
				g_enSpisRecvStus = EN_SPIS_RECV_NULL;
				sApp_Spis_RespondCmd(0x00, 0, NULL);
				continue;
			}
			else
			{
			#if LOG_TOT_EN
				DBG("recv pack identify err\n");
			#endif
				sApp_Spis_RespondCmd(0x84, 0, NULL);
				break;
			}
		}

		if(gSysTick - vu32LocalTick >= RECV_GAIN_OUTTIME)
		{
		#if LOG_TOT_EN
			DBG("receive gain data timeover\n");
		#endif
			sApp_Spis_RespondCmd(0x0F, 0, NULL);
			break;
		}
	}

	DMA_ChannelEnable(PERIPHERAL_ID_SPIS_RX, 0);
	SPIS_RX_DMA_MODE_SET(1);
	DMA_CircularConfig(PERIPHERAL_ID_SPIS_RX, SPIS_INT_LEN, DmaSpisRecvFifo, SPIS_DMA_RECV_FIFO_LEN);
	DMA_InterruptFunSet(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT, InterruptSpisDmaRecvCmd);
	DMA_InterruptEnable(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT, 1);
	DMA_ChannelEnable(PERIPHERAL_ID_SPIS_RX, 1);

	if(Result == 1)
	{
		u16CalCodeCrc = CRC16((unsigned char*)(UPDATE_FIRMWARE_ADDR),u32CodeSize,0);

	#if LOG_TOT_EN
		DBG("%08X  %08X  %08X\n",u32CodeSize,u16CalCodeCrc,u16RecvCodeCrc);
	#endif
		if(u16CalCodeCrc == u16RecvCodeCrc)
		{
		#if LOG_TOT_EN
			DBG("WRITE MVA OK\n");
		#endif
		}
		else
		{
		#if LOG_TOT_EN
			DBG("WRITE MVA Error\n");
		#endif
			sApp_Spis_RespondCmd(0x01, 0, NULL);
			return 0;
		}
		
		sApp_Spis_RespondCmd(0x00, 0, NULL);

		DelayMs(1000);
		
		if(u16CalCodeCrc == u16RecvCodeCrc)
		{
			*(volatile uint32_t *)0x4002E800 = 0xFF;
			*(volatile uint32_t *)0x4002E824 = UPDATE_FIRMWARE_ADDR;
			*(volatile uint32_t *)0x4002E800 = 0;
			*(volatile uint32_t *)0x40022008 = 1;
			*(volatile uint32_t *)0x40022008 = 0;
		}
		
	}
	else
	{
	#if LOG_TOT_EN
		DBG("recv code data err\n");
	#endif
		sApp_Spis_RespondCmd(0x01, 0, NULL);
	}
}

static u8 _app_Spis_GetFwVersion(ST_SPIS_REC_CMD_HEADER* pstCmd)
{
	sApp_Spis_RespondCmd(0x00, 4, FirmwareVersion);
}

static u8 _app_Spis_WriteSN(ST_SPIS_REC_CMD_HEADER* pstCmd)
{
	OnchipFlashRead(SN_ADDR,frameBuffer,4096);
	OnchipFlashErase(SN_ADDR,4096);
	if((pstCmd->u8ParaBuf[0]>0)&&(pstCmd->u8ParaBuf[0]<=4))
	{
		memcpy(frameBuffer+64*(pstCmd->u8ParaBuf[0]-1),&pstCmd->u8ParaBuf[1],64);
		OnchipFlashWrite(SN_ADDR,frameBuffer,4096);
		sApp_Spis_RespondCmd(0x00,0,NULL);
	}
	else
	{
		sApp_Spis_RespondCmd(0x01,0,NULL);
	}
}

static u8 _app_Spis_ReadSN(ST_SPIS_REC_CMD_HEADER* pstCmd)
{
	u8 u8ParaBuf[64];
	if((pstCmd->u8ParaBuf[0]>=1)&&(pstCmd->u8ParaBuf[0]<=4))
	{
		memset(u8ParaBuf,0,sizeof(u8ParaBuf));
		OnchipFlashRead(SN_ADDR+64*(pstCmd->u8ParaBuf[0]-1),u8ParaBuf,64);
		sApp_Spis_RespondCmd(0x00,64,u8ParaBuf);
	}
	else
	{
		sApp_Spis_RespondCmd(0x01,0,NULL);
	}

}


static u8 _app_Spis_CaptureImageSimplify(ST_SPIS_REC_CMD_HEADER* pstCmd) 
{
/*
	total 9 bytes:
	package header(2 bytes) + package identify(1 byte)
	+ package length(2 bytes) + row number(2 bytes) 
	+ row length(2 bytes)
*/
	u16 u16RecordBuf[360];
	volatile u16 vu16PacLen, vu16RowLen; 
	volatile u32 vu32RmDMASpcace;
	volatile u32 vu32LocalTick;
	uint32_t i,j,k,RowCnt;
	uint16_t tmp1,tmp2,tmp3,tmp4;
	uint32_t data;
	u8 u8ICNo1,u8ICNo2,u8FlagCntTime;
	u16 u16W1Start,u16W2Start;
	u8 ROIC1XLen,  ROIC2XLen,ROIC3XLen,ROIC4XLen; 
	u32 n,h,size,s,bb,u32StrtPoin,u32EndPoin, u32ByteCnt, u32TmpByteCnt,u32RandNum,Star1,Star2,Temp;
	volatile u32 x1=0;
//	u32 CycleRecBuf[500],CycCnt;
	u16 u16P1StarX,u16P1StarY,u16P2StarX, u16P2StarY;
	u16 u16W1BufferOffset,u16W2BufferOffset,u16PackNo;
	volatile bool vbFlagTimeover = FALSE;	
	u16 u16BufOffset;
	DMA_SRC_BUF_TYPE enSrcBufType;

	u16P1StarX = (((u16)pstCmd->u8ParaBuf[1])<<8)|(pstCmd->u8ParaBuf[2]);
	u16P1StarY = (((u16)pstCmd->u8ParaBuf[3])<<8)|(pstCmd->u8ParaBuf[4]);
	u16P2StarX = (((u16)pstCmd->u8ParaBuf[5])<<8)|(pstCmd->u8ParaBuf[6]);
	u16P2StarY = (((u16)pstCmd->u8ParaBuf[7])<<8)|(pstCmd->u8ParaBuf[8]);

	memset(u16RecordBuf, 0xFF, sizeof(u16RecordBuf));
	DMA_CircularFIFOClear(PERIPHERAL_ID_SPIS_TX);

	XAO_H();

/***capture image and send image data******************************/
	sampleCount = 0;
	dummyLines = 10;//5;//10;;
	frameCount = NORMAL_CAPTURE_NUMBER;//2;
	frameLines = 0;
	
	SET_PFIRST_0_L();//step0 pirst0=0
	PwcDmaConfig();
	PwmDmaConfig(EN_WAVE_CAPTR_IMAGE);
	sampleFinishFlag = 0;
	samplebufIndex = 1;
	sampleCount = 0;
	SET_PFIRST_0_H();//step2 pirst0 变高  //lk
	PWC_Enable(PWC_TIMER);
	PWM_Enable(PWM_TIMER);
	HCLK_L();
	RowCnt = 0;
	u16W1BufferOffset = 0;
	g_vu16LineCnt = 0;

	pstCmd->u8ParaBuf[0] = 1;

	if(pstCmd->u8ParaBuf[0] == 1)/*one window*/
	{
		jigsaw_process(u16P1StarX,u16P1StarY,0,0,0,&g_stWindowInfo_P1,NULL);
	
		while(1)
		{
			if(sampleFinishFlag != 0)
			{
				if(sampleFinishFlag == 1)
				{
					enSrcBufType = EN_SRC_BUF1;
				}
				else if(sampleFinishFlag == 2)	
				{
					enSrcBufType = EN_SRC_BUF2;
				}
				sampleFinishFlag = 0;
				__nds32__mtsr(0, NDS32_SR_PFMC0);
				__nds32__mtsr(1, NDS32_SR_PFM_CTL);
					
				sApp_Spis_CaptureWin(FALSE, enSrcBufType, &g_stWindowInfo_P1, NULL);
					
				if(dummyLines == 0 && frameLines <= (270+1+SHIFT_ROW)&& frameLines > 0 && frameCount == 1)
				{
					RowCnt++;
					if(((RowCnt>= TOP_DARKLINE_START+1+SHIFT_ROW) && (RowCnt<=(TOP_DARKLINE_END+1+SHIFT_ROW))) \
						|| ((RowCnt>= (g_stWindowInfo_P1.u16StartRow+1+SHIFT_ROW)) && (RowCnt<=(g_stWindowInfo_P1.u16EndRow+1+SHIFT_ROW))) \
						|| ((RowCnt>= (BOTTOM_DAKLINE_SATAR+1+SHIFT_ROW)) && (RowCnt<=(BOTTOM_DAKLINE_END+1+SHIFT_ROW))))
					{
						memcpy(&frameBuffer[u16W1BufferOffset*296], resData00, 296);
						u16W1BufferOffset++;
					}
				}
				
				__nds32__mtsr(0, NDS32_SR_PFM_CTL);
				///	CycleRecBuf[CycCnt++] = __nds32__mfsr(NDS32_SR_PFMC0);
				
				if( frameCount ==  0)
				{
					break;
				}
			}
		}
		
		sApp_Spis_ClrLag_AftCpt();
		sApp_Spis_RespondCmd(0x00, 0, NULL);
		sApp_Spis_SendData(EN_PROT_SIMPLY, 0,  frameBuffer, 148*148*2);
	}
	else if(pstCmd->u8ParaBuf[0] == 2)
	{
		jigsaw_process(u16P1StarX,u16P1StarY,u16P2StarX,u16P2StarY,1,&g_stWindowInfo_P1,&g_stWindowInfo_P2);
		u16W1BufferOffset = 0;
		u16W2BufferOffset = 128;

		while(1)
		{
			if(sampleFinishFlag != 0)
			{
				 j = 0;
				if(sampleFinishFlag == 1)
				{
					enSrcBufType = EN_SRC_BUF1;
				}
				else if(sampleFinishFlag == 2)	
				{
					enSrcBufType = EN_SRC_BUF2;
				}
				sampleFinishFlag = 0;
				__nds32__mtsr(0, NDS32_SR_PFMC0);
				__nds32__mtsr(1, NDS32_SR_PFM_CTL);
				sApp_Spis_CaptureWin(TRUE, enSrcBufType, &g_stWindowInfo_P1, &g_stWindowInfo_P2);
				
				if(dummyLines == 0 && frameLines <= (270+SHIFT_ROW+1)&& frameLines > 0 && frameCount == 1)
				{
					RowCnt++;
					if((RowCnt>= (g_stWindowInfo_P1.u16StartRow+1+SHIFT_ROW)) && (RowCnt<=(g_stWindowInfo_P1.u16EndRow+1+SHIFT_ROW))) 
					{
						memcpy(&frameBuffer[u16W1BufferOffset*296], resData00, 296);
						u16W1BufferOffset++;
					}

					if((RowCnt>= (g_stWindowInfo_P2.u16StartRow+1+SHIFT_ROW)) && (RowCnt<=(g_stWindowInfo_P2.u16EndRow+1+SHIFT_ROW))) 
					{						
						memcpy(&frameBuffer[u16W2BufferOffset*296], resData11, 296);
						u16W2BufferOffset++;
					}
				}
				
				__nds32__mtsr(0, NDS32_SR_PFM_CTL);
				//	CycleRecBuf[CycCnt++] = __nds32__mfsr(NDS32_SR_PFMC0);
				
				if( frameCount ==  0)
				{
					break;
				}
			}
		}

		sApp_Spis_ClrLag_AftCpt();
	
		sApp_Spis_RespondCmd(0x00, 0, NULL);
		sApp_Spis_SendData(EN_PROT_SIMPLY, 0,  frameBuffer, 148*256*2);
	}
}

static u8 _app_Spis_SetIntegralLineCnt(ST_SPIS_REC_CMD_HEADER* pstCmd)
{
	g_stCaptrAfterClkWave.IntegralFramLineCnt = (((u16)(pstCmd->u8ParaBuf[0]))<<8)|pstCmd->u8ParaBuf[1];
#if LOG_TOT_EN
	DBG("inter line:%d\n",g_stCaptrAfterClkWave.IntegralFramLineCnt);
#endif
	sApp_Spis_RespondCmd(0x00, 0, NULL);	
}

static u8 _app_Spis_Handshake(ST_SPIS_REC_CMD_HEADER* pstCmd)
{
	u8 buf[1];
	
	DelayMs(2);
	sApp_Spis_RespondCmd(0x00, 0, buf);
}

static u8 _app_Spis_LowPowerMode(ST_SPIS_REC_CMD_HEADER* pstCmd)
{
	if(pstCmd->u8ParaBuf[0] == 1)
	{
		SleepROIC();
		DelayMs(20);
		sApp_Spis_RespondCmd(0x00, 0, NULL);
	}
	else if(pstCmd->u8ParaBuf[0] == 0)
	{
		WakeUpROIC();
		DelayMs(20);
		sApp_Spis_RespondCmd(0x00, 0, NULL);
	}
	else
	{
		DelayMs(10);
		sApp_Spis_RespondCmd(0x84, 0, NULL);
	}
}


static u8 sApp_SPIS_ProcessRecPara(ST_SPIS_REC_CMD_HEADER* pstCmd)
{	
	switch(pstCmd->u8Cmd)
	{
		case EN_SPI_CMD_CAPTURE_WIN_IMAGE:
		#if LOG_TOT_EN
			DBG("CI\n");
		#endif
		
		#if FUNC_MOS_CTRL_FPC_EN
			sApp_Spis_ClrLag_BefCpt();
		#endif
			_app_Spis_CaptureImage(pstCmd);
		
			break;
			
		case EN_SPI_CMD_CAPTURE_GAIN_IMAGE:
		#if LOG_TOT_EN
			DBG("CG 3\n");
		#endif
		
		#if FUNC_MOS_CTRL_FPC_EN
			sApp_Spis_ClrLag_BefCpt();
		#endif
			_app_Spis_CaptureGainImage(pstCmd);
			break;

		case EN_SPI_CMD_CLEAR_FRAM:
			_app_Spis_ClrFrame(pstCmd);
			break;

		case EN_SPI_CMD_WRITE_GAIN:
			_app_Spis_WriteGain(pstCmd);
			break;

		case EN_SPI_CMD_READ_GAIN:
		#if LOG_TOT_EN
			DBG("R G\n");
		#endif	
			_app_Spis_GetGain(pstCmd);
			break;

		case EN_SPI_CMD_UPDATA_CODE:
			_app_Spis_UpdateCode(pstCmd);
			break;

		case EN_SPI_CMD_GET_FW_VERSION:
			_app_Spis_GetFwVersion(pstCmd);
			break;

		case EN_SPI_CMD_WRITE_SN:
			_app_Spis_WriteSN(pstCmd);
			break;

		case EN_SPI_CMD_READ_SN:
			_app_Spis_ReadSN(pstCmd);
			break;
			
		case EN_SPI_CMD_CAPTURE_WIN_IMAGE_SIMPLIFY:
		#if FUNC_MOS_CTRL_FPC_EN
			sApp_Spis_ClrLag_BefCpt();
		#endif
			_app_Spis_CaptureImageSimplify(pstCmd);
			break;

		case EN_SPI_CMD_SET_INTEGRAL_LINE:
			#if LOG_TOT_EN
			DBG("S INT\n");
			#endif
			_app_Spis_SetIntegralLineCnt(pstCmd);
			break;
			
		case EN_SPI_CMD_HANDSHAKE:
			#if LOG_TOT_EN
			DBG("HS\n");
			#endif
			_app_Spis_Handshake(pstCmd);
			break;

		case EN_SPI_CMD_SWITCH_LOW_POWER_MODE:
			#if LOG_TOT_EN
			DBG("Low power mode\n");
			#endif
			_app_Spis_LowPowerMode(pstCmd);
			break;
			
		default:
			break;
	}
	
	g_enSpisRecvStus = EN_SPIS_RECV_NULL;
	DMA_ChannelEnable(PERIPHERAL_ID_SPIS_RX, 1);
	SPIS_RX_DMA_MODE_SET(1);
}

static u8 sApp_Spis_DataParser(u8 *DataBuf)
{
	u16 u16CRC;
	
	gstSPIS_CMD_Header.u16Header = ((u16)DataBuf[0])<<8 | DataBuf[1];
	gstSPIS_CMD_Header.u8Identity = DataBuf[2];
	gstSPIS_CMD_Header.u32Length = (u16)DataBuf[3]<<8 | DataBuf[4];

	if(gstSPIS_CMD_Header.u8Identity == SPIS_CMD)
	{
		gstSPIS_CMD_Header.u8Cmd = DataBuf[5];
		gstSPIS_CMD_Header.u32RecParaLen = gstSPIS_CMD_Header.u32Length -3;//3: 2byte CRC + 1byte CMD 
		memcpy(gstSPIS_CMD_Header.u8ParaBuf, &(DataBuf[6]), gstSPIS_CMD_Header.u32RecParaLen);
		gstSPIS_CMD_Header.u16RecCRC = (u16)(DataBuf[6+gstSPIS_CMD_Header.u32RecParaLen])<<8 | \
											DataBuf[6+gstSPIS_CMD_Header.u32RecParaLen + 1];
	}
	else
	{
		sApp_Spis_Clr_RecCmdPara();
		return 0;
	}
	
	u16CRC = Checksum16(&(DataBuf[2]),(gstSPIS_CMD_Header.u32Length+1));

	if(u16CRC != gstSPIS_CMD_Header.u16RecCRC)
	{
	#if LOG_TOT_EN
		DBG("CRC error:calculate CRC=%04x,receive CRC=%04x,Rec[%d]=%x,Rec[%d]=%x\n",\
						u16CRC,gstSPIS_CMD_Header.u16RecCRC,(5+gstSPIS_CMD_Header.u32RecParaLen),\
						(DataBuf[5+gstSPIS_CMD_Header.u32RecParaLen]),(5+gstSPIS_CMD_Header.u32RecParaLen + 1),\
						DataBuf[5+gstSPIS_CMD_Header.u32RecParaLen + 1]);
	#endif
		sApp_Spis_Clr_RecCmdPara();
		return 0;
	}

	return 1;	
}


u8 App_SPIS_ProcessDataPackage(void)
{
	if(gstSPIS_RX_Queue.u16PushIndex == 0)
	{
		return 0;
	}

	/*UART receive CMD complete,then process CMD*/
	if(g_enSpisRecvStus == EN_SPIS_RECV_FINISH)
	{
		g_sbFlagRecvCmdTimeCntEn = FALSE;
		
		if(sApp_Spis_DataParser(gstSPIS_RX_Queue.u8SpisRxBuf))
		{	
			sApp_SPIS_ProcessRecPara(&gstSPIS_CMD_Header);
			
			sApp_Spis_Clr_RecCmdPara();
			return 1;
		}
	}

	/*check receive CMD timeout*/
	if(g_sbFlagRecvCmdTimeCntEn == TRUE)
	{
		if(gSysTick - g_vu32WaitCmdTick >= RECV_CMD_TIMEOUT)
		{
		#if LOG_TOT_EN
			DBG("receive CMD imcomplete timeout\n");
		#endif
			g_sbFlagRecvCmdTimeCntEn = FALSE;
			sApp_Spis_Clr_RecCmdPara();
		}
	}	
}

void App_Spi_InitSpis(void)
{
	/*SPIS INIT*/
	DMA_ChannelAllocTableSet(DmaChannelMap);
	GPIO_SpisIoConfig(2);
	SPIS_Init(0);	
	SPIS_TX_DMA_MODE_SET(1);
	DMA_CircularConfig(PERIPHERAL_ID_SPIS_TX, SPIS_DMA_SEND_FIFO_LEN/2, DmaSpisSendFifo, SPIS_DMA_SEND_FIFO_LEN);
	DMA_ChannelEnable(PERIPHERAL_ID_SPIS_TX, 1);

	SPIS_RX_DMA_MODE_SET(1);
	DMA_CircularConfig(PERIPHERAL_ID_SPIS_RX, SPIS_INT_LEN, DmaSpisRecvFifo, SPIS_DMA_RECV_FIFO_LEN);
	DMA_InterruptFunSet(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT, InterruptSpisDmaRecvCmd);
	DMA_InterruptEnable(PERIPHERAL_ID_SPIS_RX, DMA_THRESHOLD_INT, 1);
	DMA_ChannelEnable(PERIPHERAL_ID_SPIS_RX, 1);
}

