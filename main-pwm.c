/* Charlieplexing PWM demo on the CH32V003 RISC-V mcu
 * Hardware PWM version
 *
 * For the 8-pin package, the default pin assignment avoids using SWIO, and
 * leaves one additional GPIO/ADC pin free for other uses.
 *
 * Pin Map: (See schematic PNG for circuit diagram of the LEDs)
 * pin 1: cathode group 1	pin 8: SWIO/debug
 * pin 2: GND		pin 7: cathode group 2
 * pin 3: Potentiometer	pin 6: cathode group 4
 * pin 5: 3V3		pin 5: cathode group 3
 *
 * pin1=T1CH2,PA1, pin7=T1CH1,PC4, pin6=T2CH2_1,PC2,remap, pin5=T2CH4_1,PC1,remap
 *
 * in a charlieplexed matrix, the maximum number of LEDs is equalt to n*(n-1)
 * where n is the number of GPIOs used. Only LEDs sharing a common anode 
 * (or a common anode) can be lit simultaneously. This should be equal to 
 * (n-1). So for an n=4 array, there can be 12 LEDs in total, and 3 can be 
 * lit simultaneously. This implies that max brightness can be 25% in a 
 * PWM configuration.
 *
 * Hardware PWM version. All LED GPIOs to be timer outputs, switching 
 * the common anode groups every millisecond or so in the systick ISR.
 * On the smallest CH32v003 package (SOP8), there are exactly 4 timer pins
 * available (reserving pin 8 for SWIO programming/debugging):
 * pin1=T1CH2,PA1, pin7=T1CH1,PC4, pin6=T2CH2,PC2,remap, pin5=T2CH4,PC1,remap
 * this leaves pin3 free for other uses, including analog input.
 *
 * Requires PWM output to work in open-drain mode, which this mcu apparently supports
 *
 */

// Could be defined here, or in the processor defines.
#define SYSTEM_CORE_CLOCK 48000000
#define APB_CLOCK SYSTEM_CORE_CLOCK
//#define SYSTICK_USE_HCLK - this will run systick at 48MHz, instead of 48/8=6MHz

#include "ch32v003fun.h"
#include <stdio.h>

/* some bit definitions for systick regs */
#define SYSTICK_SR_CNTIF (1<<0)
#define SYSTICK_CTLR_STE (1<<0)
#define SYSTICK_CTLR_STIE (1<<1)
#define SYSTICK_CTLR_STCLK (1<<2)
#define SYSTICK_CTLR_STRE (1<<3)
#define SYSTICK_CTLR_SWIE (1<<31)

#define SYSTICKHZ 1000

/* this is the truth table for the LEDs, defining how they're connected
 * and how to set the GPIO pins to select a specific LED. (unused GPIOs
 * should be set as floating inputs)
 */

typedef struct {	//Defines the Cathode pins for each LED
	GPIO_TypeDef* port;	//e.g. GPIOA
	uint8_t pin;		//e.g. GPIO_PinSource1
	TIM_TypeDef* timer;	//Timer e.g. TIM1
	uint16_t channel;	//Timer channel Enable e.g. TIM_CC1E for ch1
	volatile uint8_t brightness;
} Connection;

/* pin1=T1CH2,PA1, pin7=T1CH1,PC4, pin6=T2CH2_1,PC2,remap, pin5=T2CH4_1,PC1,remap */
/* LED Cathode connection map */
volatile Connection LEDs[12] = { 
//	 port,	pin,		timer,	channel,	brightness
	{GPIOA, GPIO_PinSource1, TIM1,	TIM_CC2E,	8	},	// LED 0
	{GPIOC, GPIO_PinSource4, TIM1,	TIM_CC1E,	63	},	// LED 1
	{GPIOC, GPIO_PinSource4, TIM1,	TIM_CC1E,	32	},	// LED 2
	{GPIOC, GPIO_PinSource1, TIM2,	TIM_CC4E,	16	},	// LED 3
	{GPIOA, GPIO_PinSource1, TIM2,	TIM_CC2E,	16	},	// LED 4
	{GPIOC, GPIO_PinSource1, TIM2,	TIM_CC4E,	16	},	// LED 5
	{GPIOC, GPIO_PinSource2, TIM2,	TIM_CC2E,	8	},	// LED 6
	{GPIOC, GPIO_PinSource1, TIM2,	TIM_CC4E,	4	},	// LED 7
	{GPIOC, GPIO_PinSource4, TIM1,	TIM_CC1E,	2	},	// LED 8
	{GPIOC, GPIO_PinSource2, TIM2,	TIM_CC2E,	0	},	// LED 9
	{GPIOA, GPIO_PinSource1, TIM1,	TIM_CC2E,	2	},	// LED 10
	{GPIOC, GPIO_PinSource2, TIM1,	TIM_CC2E,	4	}	// LED 11
};

/* to be used to iterate through each group of LEDs. 
 * TODO: calculate this at runtime from the data in the
 * Connection struct
 * something like if(LEDs[i].port==GPIOA & LEDs[i].pin==GPIO_PinSource1) {group[?][?]=[i]}
 */
static const uint8_t group[4][3] = {	//4 cathode groups of 3 LEDs each
	{ 1, 2, 8  },	// LEDs sharing pin_C4
	{ 0, 4, 9  },	// LEDs sharing pin_A1
	{ 3, 5, 7  },	// LEDs sharing pin_C1
	{ 6, 9, 11 }	// LEDs sharing pin_C2
};

/**************************************************************
 *
 * (void) t1t2_pwm_init ( void )
 *
 * initialize timer1 and timer2 peripherals for PWM output
 *
 **************************************************************/
void t1t2_pwm_init( void )
{
	// Enable GPIOA, GPIOC, TIM1, TIM2, IO Aux function module
	RCC->APB2PCENR |=	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC |
				RCC_APB2Periph_TIM1  | RCC_AFIOEN;
	RCC->APB1PCENR |=	RCC_APB1Periph_TIM2;
    
	// Reset TIM1 & 2 to init all regs
	RCC->APB2PRSTR |=  RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;
	RCC->APB1PRSTR |=  RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

/* pin1=T1CH2,PA1, pin7=T1CH1,PC4, pin6=T2CH2_1,PC2,remap, pin5=T2CH4_1,PC1,remap */
	
	// PC4 is T1CH1, 10MHz Output alt func, open-drain
	GPIOC->CFGLR &= ~(0xf<<(4*4));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*GPIO_PinSource4);
	// PA1 is T1CH2
	GPIOA->CFGLR &= ~(0xf<<(4*1));
	GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*GPIO_PinSource1);
	// PC2 is T2CH2
	GPIOC->CFGLR &= ~(0xf<<(4*2));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*GPIO_PinSource2);
	// PC1 is T2CH4
	GPIOC->CFGLR &= ~(0xf<<(4*1));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*GPIO_PinSource1);

	// Remap mode 01: Partial mapping (CH1/ETR/PC5, CH2/PC2, CH3/PD2, CH4/PC1).
	AFIO->PCFR1 |= AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP1;

	// SMCFGR: default clk input is CK_INT

	// Prescaler - this, along with the period, will determine the PWM frequency
	TIM1->PSC = 0x0000;
	TIM2->PSC = 0x0000;
	// Auto Reload - sets period
	TIM1->ATRLR = 255;
	TIM2->ATRLR = 255;

	// Reload immediately
	// initialize counter
	TIM1->SWEVGR |= TIM_UG;
	TIM2->SWEVGR |= TIM_UG;
	
	// CTLR1: default is up, events generated, edge align
	// enable auto-reload of preload
	TIM2->CTLR1 |= TIM_ARPE;
	TIM1->CTLR1 |= TIM_ARPE; // unnecessary?

	// Set the Capture Compare Register value to 50% initially
	TIM1->CH1CVR = 128; //T1CH1/PC4/pin7
	TIM1->CH2CVR = 128; //T1CH2/PA1/pin1
	TIM2->CH2CVR = 128; //T2CH2/PC2/pin6*
	TIM2->CH4CVR = 128; //T2CH4/PC1/pin5*
	                    // * - alternate function

/* pin1=T1CH2,PA1, pin7=T1CH1,PC4, pin6=T2CH2,PC2,remap, pin5=T2CH4,PC1,remap */
	TIM1->CCER |= TIM_CC1E | TIM_CC1P;	// Enable CH1 output, positive pol
	TIM1->CCER |= TIM_CC2E | TIM_CC2P;	// Enable CH4 output, positive pol
	TIM2->CCER |= TIM_CC2E | TIM_CC2P;	// Enable CH1 output, positive pol
	TIM2->CCER |= TIM_CC4E | TIM_CC4P;	// Enable CH2 output, positive pol

	TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;	// CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1;	// CH2 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	TIM2->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2PE;
	TIM2->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1 | TIM_OC4PE;
	// for channel 1 and 2, let CCxS stay 00 (output), set OCxM to 110 (PWM I)
	// enabling preload causes the new pulse width in compare capture register only to come into effect when UG bit in SWEVGR is set (= initiate update) (auto-clears)

	TIM1->BDTR  |= TIM_MOE;	// Brake and deadband registers
	TIM1->CTLR1 |= TIM_CEN;	// Enable TIM1
	TIM2->CTLR1 |= TIM_CEN;	// Enable TIM2
}

/* 
 * Detaches pin from timer output, sets to OD output mode, sets output low
 *
 * Example: GPIO_Low(GPIOA, GPIO_PinSource1, TIM1, TIM_CC1E); // detach PA1 from T1C1, sets OD mode, Low
 * 
 */
void GPIO_Low(GPIO_TypeDef* port, uint8_t pin, TIM_TypeDef* timer, uint16_t channel) {
        timer->CCER &= ~channel;        //disable this channel output
        port->CFGLR &= ~(0xf<<(4*pin)); //clear GPIO config - this appears to be necessary. 
        port->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD)<<(4*pin); //Sets output mode to GPIO, OD
        port->BCR = (1<<(pin));         //set GPIO 'low'
};

/* 
 * Detaches pin from timer output, sets to PP output mode, sets output high
 *
 * Example: GPIO_High(GPIOA, GPIO_PinSource1, TIM1, TIM_CC1E); // detach PA1 from T1C1, sets PP mode, High
 * 
 */
void GPIO_High(GPIO_TypeDef* port, uint8_t pin, TIM_TypeDef* timer, uint16_t channel) {
        timer->CCER &= ~channel;        //disable this channel output
        port->CFGLR &= ~(0xf<<(4*pin)); //clear GPIO config - this appears to be necessary. 
        port->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*pin); //Sets output mode to GPIO, PP
        port->BSHR = (1<<(pin));                //set GPIO 'high'
};

/* 
 * Detaches pin from timer output, sets to input mode
 *
 * Example: GPIO_Float(GPIOA, GPIO_PinSource1, TIM1, TIM_CC1E); // detach PA1 from T1C1, sets PP mode, High
 * 
 */
void GPIO_Float(GPIO_TypeDef* port, uint8_t pin, TIM_TypeDef* timer, uint16_t channel) {
        timer->CCER &= ~channel;        //disable this channel output
        port->CFGLR &= ~(0xf<<(4*pin)); //clear GPIO config - default config is as an input, Hi-Z
        //port->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*pin); //Sets output mode to GPIO, PP
        //port->BSHR = (1<<(pin));                //set GPIO 'high'
};

/* 
 * Attaches pin to timer output, sets to OD_AF output mode
 * Requires the appropriate timer to be configured already.
 *
 * Example: GPIO_PWM(GPIOA, 1, TIM1, TIM_CC1E); // attach PA1 to T1C1, sets OD_AF mode
 * 
 */
void GPIO_PWM(GPIO_TypeDef* port, uint8_t pin, TIM_TypeDef* timer, uint16_t channel) {
        port->CFGLR &= ~(0xf<<(4*pin)); //clear GPIO config - this appears to be necessary. 
        port->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*pin);//Sets mode to OD_AF
        timer->CCER |= channel;         //enable this channel output
};

/*
 * set timer channel PW
 *
 * Example: pwm_setpw(TIM1, 1, 128); // Set TIM1CH1 to pulsewidth of 128
 */
void pwm_setpw(TIM_TypeDef* timer, uint16_t chl, uint16_t width)
{
    	switch( chl )
	    {
	    	case TIM_CC1E: timer->CH1CVR = width; break;
	    	case TIM_CC2E: timer->CH2CVR = width; break;
	    	case TIM_CC3E: timer->CH3CVR = width; break;
	    	case TIM_CC4E: timer->CH4CVR = width; break;
	    }
}

/*
 * Setup systick to fire an interrput at SYSTICKHZ Hz
 *
 * TODO: is this necessary, or does ch32v003fun init systick to 1kHz already?
 */
void systick_init(void) {
	/* disable default SysTick behavior */
	SysTick->CTLR = 0;
	
	/* enable the SysTick IRQ */
	NVIC_EnableIRQ(SysTicK_IRQn);

	/* Set the tick interval to SYSTICKHZ */
	SysTick->CMP = (SYSTEM_CORE_CLOCK/8/SYSTICKHZ)-1;
	
	/* Start at zero */
	SysTick->CNT = 0;
	
	/* Enable SysTick counter, IRQ, HCLK/1 */
	SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE |
					SYSTICK_CTLR_STCLK;
}

/*
 * SysTick ISR does all the work.
 *
 */

/*
 * note - the __attribute__((interrupt)) syntax is crucial!
 */
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
	/* move the compare further ahead in time.
	 * as a warning, if more than this length of time
	 * passes before triggering, you may miss your
	 * interrupt.
	 */
	SysTick->CMP += (SYSTEM_CORE_CLOCK/8/SYSTICKHZ); //by default, Systick runs at 48/8=6MHz

	/* clear IRQ */
	SysTick->SR = 0;

/********************************************************
 * Each time we enter this ISR (every 1ms) we need to:
 *
 * 1. Set all pins HIGH (PWM off). (technically only really need to set the 3 PWM pins high)
 * 2. Switch to next group of LEDs
 * 3. Update PWM brightness settings (at this point, nothing is being actually output)
 * 4. Select next group's cathodes (3 of them), and set to PWM
 *
 * At any given time (except when switching groups) there's one pin HIGH, and the other 3 pins on OD PWM
 *
 ********************************************************/
	static uint8_t currentgroup = 0; //the common anode group of LEDs are being worked on
	  
  	/* 1. Set all pins to HIGH (PWM off) */
	GPIO_High(GPIOA, GPIO_PinSource1, TIM1, TIM_CC2E); //PA1, T1CH2
	GPIO_High(GPIOC, GPIO_PinSource1, TIM2, TIM_CC4E); //PC1, T2CH4
	GPIO_High(GPIOC, GPIO_PinSource2, TIM2, TIM_CC2E); //PC2, T2CH2
	GPIO_High(GPIOC, GPIO_PinSource4, TIM1, TIM_CC1E); //PC4, T1CH1		
	
	/* 2. Switch to next group of LEDs */
	if(++currentgroup > 4) currentgroup=0;
		
	/* 3. Update PWM brightness settings */
	for(int i=0;i<3;i++){
		pwm_setpw(	LEDs[group[currentgroup][i]].timer,
				LEDs[group[currentgroup][i]].channel,
				LEDs[group[currentgroup][i]].brightness
			 );
	}
	
	/* 4. Select next group's cathodes (3 of them), and set to PWM */
	for(int i=0;i<3;i++){
		GPIO_PWM(	LEDs[group[currentgroup][i]].port,
				LEDs[group[currentgroup][i]].pin,
				LEDs[group[currentgroup][i]].timer,
				LEDs[group[currentgroup][i]].channel
			 );
	}

	/* pin1=T1CH2,PA1, pin7=T1CH1,PC4, pin6=T2CH2_1,PC2,remap, pin5=T2CH4_1,PC1,remap */
}

/**********************************************************************************
*
* Configure ADC for polled input. 
* Pin 3 = PA2 / ADC Ch0
*
***********************************************************************************/
void adc_init( void )
{
	// ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
	RCC->CFGR0 &= ~(0x1F<<11);
	
	// Enable GPIOC and ADC
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1;
	
	// PA2 is analog input chl 0
	GPIOA->CFGLR &= ~(0xf<<(4*GPIO_PinSource2));	// CNF = 00: Analog, MODE = 00: Input
	
	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;
	
	// Set up single conversion on chl 0 
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = ADC_Channel_0;	// 0-9 for 8 ext inputs and two internals
	
	// set sampling time for chl 0
	ADC1->SAMPTR2 &= ~ADC_SMP2;
	ADC1->SAMPTR2 |= ( ADC_SampleTime_3Cycles << (3 * ADC_Channel_0 ) );	// 0:7 => 3/9/15/30/43/57/73/241 cycles
		
	// turn on ADC and set rule group to sw trig
	ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL;
	
	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);
	
	// Calibrate
	ADC1->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);
	
	// should be ready for SW conversion now
}

/*
 * start conversion, wait and return result
 */
uint16_t adc_get( void )
{
	// start sw conversion (auto clears)
	ADC1->CTLR2 |= ADC_SWSTART;
	
	// wait for conversion complete
	while(!(ADC1->STATR & ADC_EOC));
	
	// get result
	return ADC1->RDATAR;
}

/*
 * main() - does the stuff
 */
int main() {
	SystemInit48HSI();
	t1t2_pwm_init();	//initialize timers and outputs for PWM
	systick_init();		//start systick
	adc_init();		//start ADC for sw polling

	while(1){
		int Delay = adc_get();
		Delay_Ms(( Delay >> 3 ) + 5 );
		int temp=LEDs[0].brightness;
		for(int i=1; i<12; i++){
			LEDs[i-1].brightness=LEDs[i].brightness;
		}
		LEDs[11].brightness=temp;
	}
}
