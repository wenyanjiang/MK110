#ifndef __APP_SPI_H_
#define __APP_SPI_H_
#include "type.h"
#include "oxifp_type_define.h"
#include "drv_config.h"

#define SPIS_LENGTH_MAX			256//1024
#define SPIS_CMD				0x01
#define RECV_CMD_TIMEOUT		200/*unit:ms*/

#define MAX_ROW_CNT			250//128
#define ROIC_WIDTH			128
#define MAX_X_POSITION		511
#define MAX_Y_POSITION		249
#define WINDOW_X			128///150
#define WINDOW_Y			128//128

typedef enum _EN_SPIS_RECV_STATUS
{
	EN_SPIS_RECV_NULL = 0,
	EN_SPIS_RECV_HEADER_INCOMP,//receive header incomplete
	EN_SPIS_RECV_START ,
	EN_SPIS_RECV_FINISH,   //receive complete
	
}EN_SPIS_RECV_STATUS;

typedef enum _EN_SPI_CMD
{
	EN_SPI_CMD_CAPTURE_WIN_IMAGE						= 0x01,
	EN_SPI_CMD_CAPTURE_GAIN_IMAGE						= 0x02, 
	EN_SPI_CMD_CLEAR_FRAM								= 0x03,
	EN_SPI_CMD_WRITE_GAIN								= 0x04,
	EN_SPI_CMD_READ_GAIN								= 0x05,
	EN_SPI_CMD_UPDATA_CODE								= 0x06,
	EN_SPI_CMD_GET_FW_VERSION							= 0x07,
	EN_SPI_CMD_WRITE_SN									= 0x08,
	EN_SPI_CMD_READ_SN									= 0x09,

	EN_SPI_CMD_CAPTURE_WIN_IMAGE_SIMPLIFY				= 0x10,

	EN_SPI_CMD_SET_INTEGRAL_LINE						= 0x80,
	
	EN_SPI_CMD_HANDSHAKE								= 0x17,   
	
	EN_SPI_CMD_SWITCH_LOW_POWER_MODE					= 0xF1,
	
}EN_UART_CMD;

typedef enum _EN_DMA_SRC_BUF_TYPE
{
	EN_SRC_BUF1 = 0x01,  
	EN_SRC_BUF2 = 0x02,  
	
}DMA_SRC_BUF_TYPE;

typedef enum _EN_SEND_PROT_TYPE
{
	EN_PROT_NOML 	= 0x01,  
	EN_PROT_SIMPLY 	= 0x02,  
	
}EN_SEND_PROT_TYPE;

typedef struct
{
	u8 u8SpisRxBuf[SPIS_LENGTH_MAX];
	u16 u16PushIndex;
	u16 u16PopIndex;
	u16 u16RecCounter;	
	
}ST_SPIS_ITEM;

typedef struct
{
	u16 u16Header;
	u8 u8Identity;
	u32 u32Length;
	u8 u8Cmd;
	u8 u8ParaBuf[SPIS_LENGTH_MAX];
	u32 u32RecParaLen;
	u16 u16RecCRC;
	
}ST_SPIS_REC_CMD_HEADER;

typedef struct
{
	u8 u8FrstICNo;
	u8 u8FrstICStrt;
	u8 u8FrstICLen;

	u8 u8ScndICNo;
	u8 u8ScndICLen;

	u8 u8ThirdICNo;
	u8 u8ThirdICLen;

	u16 u16StartRow;
	u16 u16EndRow;


	u32 u32ByteCnt;
	
}ST_WINDOW_INFO;

extern ST_WINDOW_INFO g_stWindowInfo_P1;
extern ST_WINDOW_INFO g_stWindowInfo_P2;

extern void App_Spi_InitSpis(void);
#endif	

