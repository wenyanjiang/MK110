#ifndef __APP_OXI_H_
#define __APP_OXI_H_
#include "type.h"
#include "oxifp_type_define.h"

#define PAK_COUNT_16BIT 45
#define PAK_COUNT_GAIN 23
#define PAK_COUNT_GAIN_8BIT	23
#define	PAK_COUNT_8BIT 23

extern void App_Usb_ProcessDataPackage(void);
extern void App_Usb_ResEssentialInfo(void);

#endif	

