//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is tutorial code for Part 1 of Introductory Lab.
//
// See "system/include/cmsis/stm32f051x8.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f051x8.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "display.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/* Definitions of registers and their bits are
 given in system/include/cmsis/stm32f051x8.h */

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Clock prescaler for TIM3 timer: set to trigger every 1ms */
#define myTIM3_PRESCALER ((uint16_t)48000 - 1)
/* Delay count for TIM2 timer: max size */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myDAC_Init(void);
void myADC_Init(void);
void myEXTI_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void mySPI1_Init(void);
void update_Freq(void);
void update_POT(void);
void sleep_ms(uint16_t);

/*** Call this function to boost the STM32F0xx clock to 48 MHz ***/

void SystemClock48MHz(void) {
//
// Disable the PLL
//
	RCC->CR &= ~(RCC_CR_PLLON);
//
// Wait for the PLL to unlock
//
	while (( RCC->CR & RCC_CR_PLLRDY) != 0)
		;
//
// Configure the PLL for 48-MHz system clock
//
	RCC->CFGR = 0x00280000;
//
// Enable the PLL
//
	RCC->CR |= RCC_CR_PLLON;
//
// Wait for the PLL to lock
//
	while (( RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY)
		;
//
// Switch the processor to the PLL clock source
//
	RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;
//
// Update the system with the new clock frequency
//
	SystemCoreClockUpdate();

}

/*****************************************************************/

char firstEdge = 1;
char usingFunctionGenerator = 1;

unsigned int freq = 0;
unsigned int res = 0;
unsigned int pot = 0;

int main(int argc, char *argv[]) {

	SystemClock48MHz();

	// By customizing __initialize_args() it is possible to pass arguments,
	// for example when running tests with semihosting you can pass various
	// options to the test.
	// trace_dump_args(argc, argv);

	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init(); /* Initialize I/O port PA */
	myGPIOB_Init();
	myDAC_Init();
	myADC_Init();
	myEXTI_Init(); /* Initialize EXTI */
	myTIM2_Init(); /* Initialize timer TIM2 */
	myTIM3_Init();
	mySPI1_Init();
	oled_config();

	while (1) {
		update_POT();
		refresh_OLED(freq, res);
	}

	return 0;

}

void myGPIOA_Init() {
	/* Enable clock for GPIOA peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA0 as input */
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);

	/* Configure PA1 as input */
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);


	/* Configure PA2 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER2);

	/* Ensure no pull-up/pull-down for PA0 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

	/* Ensure no pull-up/pull-down for PA1 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	/* Ensure no pull-up/pull-down for PA2 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);

	/* Configure PA4 and PA5 as analog mode */
	GPIOA->MODER |= GPIO_MODER_MODER4;
	GPIOA->MODER |= GPIO_MODER_MODER5;
}

void myGPIOB_Init() {
	/* Enable clock for GPIOB peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Configure PB3 and PB5 as AF0 */
	GPIOB->MODER &= ~(GPIO_MODER_MODER3);
	GPIOB->MODER |= GPIO_MODER_MODER3_1;
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3);

	GPIOB->MODER &= ~(GPIO_MODER_MODER5);
	GPIOB->MODER |= GPIO_MODER_MODER5_1;
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL5);

	/* Configure PB4, PB6, and PB7 as output */
	GPIOB->MODER &= ~(GPIO_MODER_MODER4);
	GPIOB->MODER |= GPIO_MODER_MODER4_0;
	GPIOB->MODER &= ~(GPIO_MODER_MODER6);
	GPIOB->MODER |= GPIO_MODER_MODER6_0;
	GPIOB->MODER &= ~(GPIO_MODER_MODER7);
	GPIOB->MODER |= GPIO_MODER_MODER7_0;
}

void myDAC_Init() {
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	/* Enable DAC_OUT1 channel */
	DAC->CR |= DAC_CR_EN1;

	/* Disable channel1 tri-state buffer */
	DAC->CR &= ~DAC_CR_BOFF1;

	/* Disable channel1 trigger enable */
	DAC->CR &= ~DAC_CR_TEN1;
}

void myADC_Init() {
	// turn on clock
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	// calibrate ADC
	ADC1->CR &= ~ADC_CR_ADEN;
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL)
		;

	// enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	ADC1->CR &= ~ADC_CR_ADDIS;

	// wait for ready
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
		;

	ADC1->CHSELR |= ADC_CHSELR_CHSEL5;

	// set to continuous and right-align
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;

	// start ADC
	ADC1->CR |= ADC_CR_ADSTART;
}

void myTIM2_Init() {
	/* Enable clock for TIM2 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM2->CR1 = ((uint16_t) 0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	TIM2->EGR = ((uint16_t) 0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 0);
	// Same as: NVIC->IP[3] = ((uint32_t)0x00FFFFFF);

	/* Enable TIM2 interrupts in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);
	// Same as: NVIC->ISER[0] = ((uint32_t)0x00008000) */

	/* Enable update interrupt generation */
	TIM2->DIER |= TIM_DIER_UIE;
}

void myTIM3_Init() {
	/* Enable clock for TIM3 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	/* Configure TIM3: buffer auto-reload, count down, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM3->CR1 = ((uint16_t) 0x001C);

	/* Set clock prescaler value */
	TIM3->PSC = myTIM3_PRESCALER;

	/* Update timer registers */
	TIM3->EGR = ((uint16_t) 0x0001);
}

void myEXTI_Init() {
	/* Map EXTI0 line to PA0 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;

	/* Map EXTI0 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[1]
	SYSCFG->EXTICR[1] = SYSCFG_EXTICR1_EXTI1_PA;

	/* Map EXTI0 line to PA2 */
	// Relevant register: SYSCFG->EXTICR[2]
	SYSCFG->EXTICR[2] = SYSCFG_EXTICR1_EXTI2_PA;

	/* EXTI0 line interrupts: set falling-edge trigger */
	// Relevant register: EXTI->FTSR
	EXTI->FTSR |= 0x1;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= 0x1 << 1;

	/* EXTI2 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= 0x1 << 2;

	/* Unmask interrupts from EXTI0 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= 0x1;

	/* Unmask interrupts from EXTI2 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= 0x1 << 2;

	/* Assign EXTI0 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Assign EXTI2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI2_3_IRQn, 1);

	/* Enable EXTI0 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	/* Enable EXTI2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI2_3_IRQn);
}

void mySPI1_Init() {
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
}

/* This handler is declared in system/src/cmsis/vectors_stm32f051x8.c */
void TIM2_IRQHandler() {
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0) {
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

void EXTI0_1_IRQHandler() {
	if ((EXTI->PR & EXTI_PR_PR0) != 0) {
		firstEdge = 1;
		EXTI->IMR ^= 0b110; // toggle interrupts

		EXTI->PR |= EXTI_PR_PR0;
	}

	if ((EXTI->PR & EXTI_PR_PR1) != 0) {
		update_Freq();

		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		// NOTE: A pending register (PR) bit is cleared
		// by writing 1 to it.

		EXTI->PR |= EXTI_PR_PR1;
	}
}

/* This handler is declared in system/src/cmsis/vectors_stm32f051x8.c */
void EXTI2_3_IRQHandler() {
	// Declare/initialize your local variables here...

	/* Check if EXTI2 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR2) != 0) {
		update_Freq();

		// 2. Clear EXTI2 interrupt pending flag (EXTI->PR).
		// NOTE: A pending register (PR) bit is cleared
		// by writing 1 to it.

		EXTI->PR |= EXTI_PR_PR2;
	}
}

//1. If this is the first edge:
//	- Clear count register (TIM2->CNT).
//	- Start timer (TIM2->CR1).
//    Else (this is the second edge):
//	- Stop timer (TIM2->CR1).
//	- Read out count register (TIM2->CNT).
//	- Calculate signal period and frequency.
//	- Print calculated values to the console.
//	  NOTE: Function trace_printf does not work
//	  with floating-point numbers: you must use
//	  "unsigned int" type to print your signal
//	  period and frequency.
void update_Freq() {
	if (firstEdge) {
		firstEdge = 0;

		TIM2->CNT = 0;
		TIM2->CR1 |= TIM_CR1_CEN;
	} else {
		firstEdge = 1;

		TIM2->CR1 &= ~(TIM_CR1_CEN);
		uint32_t count = TIM2->CNT;

		float period = count * 1.0 / SystemCoreClock;
		freq = 1 / period;
	}
}

void update_POT() {
	pot = ADC1->DR;
	res = pot * 5000 / 4095;
	DAC->DHR12R1 = pot;
}

void sleep_ms(uint16_t sleep_ms) {
	TIM3->SR &= ~(TIM_SR_UIF); // clear UIF

	// set timer to run for sleep_ms ms
	TIM3->CNT = sleep_ms;

	TIM3->CR1 |= TIM_CR1_CEN; // start timer

	while (!(TIM3->SR & TIM_SR_UIF))
		;

	TIM3->SR &= ~(TIM_SR_UIF); // clear UIF

	TIM3->CR1 &= ~(TIM_CR1_CEN); // stop timer
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
