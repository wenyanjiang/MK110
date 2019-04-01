#ifndef __APP_SPI_H_
#define __APP_SPI_H_
#include "type.h"
#include "oxifp_type_define.h"

extern int OnchipFlashBlockErase(uint32_t Page_Address,int size);
extern int OnchipFlashErase(uint32_t Page_Address,int size);
extern int OnchipFlashRead(uint32_t Page_Address,u8* ptr,int size);
extern int OnchipFlashWrite(uint32_t Page_Address,u8* ptr,int size);
extern void App_Roic_ParaInit(void);

#endif	

