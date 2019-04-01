
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
#include "dev_dma.h"
#include "dev_gpio.h"
#include "dev_roic.h"
#include "app_spi.h"
#include "drv_config.h"
#include "drv_sys.h"
#include "app_usb.h"

static void Oscillator_Set(void)
{
#if(OSC_TYPE == OSC_INTERNAL)
	Clock_PllFreeRunEfuse();
#else		
	/*init OSC GPIO*/
	GPIO_RegOneBitClear(GPIO_B_PU, GPIOB30);
	GPIO_RegOneBitSet(GPIO_B_PD, GPIOB30);
	GPIO_RegOneBitClear(GPIO_B_PU, GPIOB31);
	GPIO_RegOneBitSet(GPIO_B_PD, GPIOB31);
	
	Clock_Config(1, 12000000);
	Clock_PllLock(480000);

#endif
}

void System_Init(void)
{
	
	WDG_Disable();
//	delayUS(2*1000);
//	WDG_Disable();
	CacheTcmInitFast(PHYMEM_16KBPM0 /*cache*/, 
					PHYMEM_NONE	/*tcm r0*/,
					 0				/*tcm r0 offset*/, 
					 4				/*tcm r0 size*/, 
					 PHYMEM_NONE	/*tcm r1*/, 
					 0				/*tcm r1 offset*/, 
					 0				/*tcm r1 size*/
					 );
	Oscillator_Set();
	Clock_SysClkSelect(PLL_CLK_MODE);
	Clock_UARTClkSelect(1);
	SpiFlashInit(80000000, MODE_4BIT, 0, 1);

}

void Uart_Init(void)
{
#if(LOG_TOT_EN == 0)	
	GPIO_UartTxIoConfig(2, 0xff);
	GPIO_UartRxIoConfig(2, 0xff);

#else
	#if(INTERFACE_TYPE == INTERFACE_SPI)
		GPIO_UartTxIoConfig(2, 1);//Uart2 TX:A30 send data
		UARTS_Init(2,  115200, 8,  0,  1);
	#endif
#endif

	GPIO_UartTxIoConfig(1, 1);//Uart1 TX:A28 Send image
	UARTS_Init(1,  115200, 8,  0,  1);
}
/*
1. clkx4 跑8M
2. pirst0 变高
3. 等DE变高，开始数据128*8 个
4. DE会自己变低，但CLKX4 8M不要停
5. 停掉CLKX4 8M，拉低,
6. pirst0 拉低， 产生CLKX4上的几个低频时钟（6个上升沿）
7. HCLKB 反向，HCLK 反向
8. clkx4 驱动最大间隔的时钟，
9.  clkx4 再驱动3个低频上升沿时钟，
 * */

void Peripheral_Init(void)
{
	SpiFlashInit(80000000, MODE_4BIT, 0, 1);	
	Dev_Gpio_Init();
	Uart_Init();
	GIE_ENABLE();
	ADC_Disable();
	Clock_ModuleEnable(ALL_MODULE_CLK_SWITCH);
	SysTickInit();
	mem_init();
///	Dev_Gpio_Init();

#if(INTERFACE_TYPE == INTERFACE_USB)
	UsbInit();
#endif

	memset(srcRevBuf1, 0, MEMORY_BYTE_LEN);
	memset(srcRevBuf2, 0, MEMORY_BYTE_LEN);
	App_Spi_InitSpis();
	/*SPIM1 INIT*/
	SPIM1_DISABLE_SPIS();
	SPIM1_CS_INIT();
	Dev_Roic_InitRoic();//This example show the DMA and MCU recv test
///	SleepROIC();
	GPIO_Spim1IoConfig(0xFF);
	SCLK_PIRST_LO();
	SCLK_PIRST_OUTPUT();

	NVIC_EnableIRQ(DMA0_IRQn);
	NVIC_EnableIRQ(DMA1_IRQn);
	
	REG_GPIO_B_REG_OE &= ~(0x77F);//d2 and d3:B0~B6,B8,B9,SDO_DE:B14
	REG_GPIO_B_REG_IE |= 0x77F;
	GPIO_RegOneBitSet(GPIO_C_IE,GPIOC0);
	GPIO_RegOneBitClear(GPIO_C_OE,GPIOC0);
}


/*sandy is my most lover*/
int main()
{
	uint8_t	recvBuf;
	u32 i;

	//System_Init();
	Peripheral_Init();
	App_Roic_ParaInit();

	while(1)
	{
	#if(INTERFACE_TYPE == INTERFACE_SPI)
		App_SPIS_ProcessDataPackage();
	#elif(INTERFACE_TYPE == INTERFACE_USB)
		App_Usb_ProcessDataPackage();
	#endif
	}
}

