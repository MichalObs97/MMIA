#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "stm32f0xx.h"
#include "sct.h"


int main(void)
{
	sct_init();
	sct_led(0x7A5C36DE);
	for (volatile uint32_t k = 0; k< 100000; k++) {}

	for (;;) {}
}
