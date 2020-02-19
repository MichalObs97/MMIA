#include "stm32f0xx.h"
#include "sct.h"

#define sct_nla(x) do { if (x) GPIOB->BSRR = (1 << 5); else GPIOB->BRR = (1 << 5); } while (0)
#define sct_sdi(x) do { if (x) GPIOB->BSRR = (1 << 4); else GPIOB->BRR = (1 << 4); } while (0)
#define sct_clk(x) do { if (x) GPIOB->BSRR = (1 << 3); else GPIOB->BRR = (1 << 3); } while (0)
#define sct_noe(x) do { if (x) GPIOB->BSRR = (1 << 10); else GPIOB->BRR = (1 << 10); } while (0)

void sct_init(void)
{
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER10_0 ;

	sct_led(0);
	sct_noe(0);
}

void sct_led(uint32_t value)
{
	uint32_t i = 0;
	for (i = 0; i<32; i++)
	{
		if (value & 1)
		{
			sct_sdi(1);
		}
		else
		{
			sct_sdi(0);
		}
		value >>= 1;
		sct_clk(1);
		//for (volatile uint32_t j = 0; j < 100000; j++) {}
		sct_clk(0);
	}
	sct_nla(1);
	sct_nla(0);
}


