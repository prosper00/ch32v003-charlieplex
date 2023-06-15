/* Charlieplexing PWM demo on the CH32V003 RISC-V mcu
 * Hardware PWM version
 *
 * For the 8-pin package, the default pin assignment avoids using SWIO, and
 * leaves one additional GPIO/ADC pin free for other uses.
 *
 * Pin Map: (See schematic PNG for circuit diagram of the LEDs)
 * pin 1: array pin1	pin 8: SWIO/debug
 * pin 2: GND		pin 7: array pin2
 * pin 3: Potentiometer	pin 6: array pin3 
 * pin 5: 3V3		pin 5: array pin4
 *
 * pin1=T1CH2,PA1, pin7=T1CH4,PC4, pin6=T2CH2_1,PC2,remap, pin5=T2CH4_1,PC1,remap
 *
 * in a charlieplexed matrix, the maximum number of LEDs is equal to n*(n-1)
 * where n is the number of GPIOs used. Only LEDs sharing a common anode 
 * can be lit simultaneously. This should be equal to (n-1). For an n=4 array, 
 * there can be 12 LEDs in total, and 3 can be lit simultaneously
 *
 * Hardware PWM version. All LED GPIOs are timer outputs, switching 
 * the common pin groups every millisecond or so in the systick ISR.
 *
 * Requires PWM output to work in open-drain mode, which this mcu apparently supports
 *
 */

#define SYSTEM_CORE_CLOCK 48000000
#define APB_CLOCK SYSTEM_CORE_CLOCK
#define SYSTICK_USE_HCLK

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

/*maps the GPIO pins to their associated timer and output channel
 * NOTE: this definition is using alternate function remapping
 * and not the default channel mappings. Search for 'AFIO' to see
 * where this is activated
 */
typedef struct {
	GPIO_TypeDef* port;
	uint8_t pin;
	TIM_TypeDef* timer;
	uint16_t channel;
} timermap_typedef;

/* pin1=T1CH2,PA1, pin7=T1CH4,PC4, pin6=T2CH2_1,PC2,remap, pin5=T2CH4_1,PC1,remap */
timermap_typedef PA1 = {GPIOA, 1, TIM1, TIM_CC2E};
timermap_typedef PC4 = {GPIOC, 4, TIM1, TIM_CC4E};
timermap_typedef PC2 = {GPIOC, 2, TIM2, TIM_CC2E};
timermap_typedef PC1 = {GPIOC, 1, TIM2, TIM_CC4E};

/* this is the connection table for the LEDs, defining how they're connected.
 * Adjust the 'brightness' value in software to individually set a LED's brigness
 */
typedef struct {
	timermap_typedef* anode;	//e.g. PA1
	timermap_typedef* cathode;
	volatile uint8_t brightness;
} Connection;

/* LED pin connection map - see included PNG schematic.*/
volatile Connection LEDs[12] = { 
//	anode port,pin cathode port,pin,timer,channel,	brightness
	{&PC4,	&PA1,	16	},	// LED 0
	{&PA1,	&PC4,	255	},	// LED 1
	{&PC1,	&PC4,	128	},	// LED 2
	{&PC4,	&PC1,	64	},	// LED 3
	{&PC1,	&PA1,	32	},	// LED 4
	{&PA1,	&PC1,	16	},	// LED 5
	{&PC1,	&PC2,	8	},	// LED 6
	{&PC2,	&PC1,	4	},	// LED 7
	{&PC2,	&PC4,	2	},	// LED 8
	{&PC4,	&PC2,	1	},	// LED 9
	{&PC2,	&PA1,	0	},	// LED 10
	{&PA1,	&PC2,	0	}	// LED 11
};

/* to be used to iterate through each group of LEDs. 
 * A 'group' is defined as all the LED's that are sharing a common anode point
 * TODO: calculate this at runtime from the data in the 'Connection' struct
 * something like if(LEDs[i].anode==&PA1) {group[?][?]=[i]}
 */
static const uint8_t group[4][3] = {	//4 common-anode groups of 3 LEDs each
	{ 0, 3, 9  },	// LEDs sharing pin_C4
	{ 1, 5, 11 },	// LEDs sharing pin_A1
	{ 2, 4, 6  },	// LEDs sharing pin_C1
	{ 7, 8, 10 }	// LEDs sharing pin_C2
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

	// PC4 is T1CH4, 10MHz Output alt func, open-drain
	GPIOC->CFGLR &= ~(0xf<<(4*GPIO_PinSource4));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*GPIO_PinSource4);
	// PA1 is T1CH2
	GPIOA->CFGLR &= ~(0xf<<(4*GPIO_PinSource1));
	GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*GPIO_PinSource1);
	// PC2 is T2CH2
	GPIOC->CFGLR &= ~(0xf<<(4*GPIO_PinSource2));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*GPIO_PinSource2);
	// PC1 is T2CH4
	GPIOC->CFGLR &= ~(0xf<<(4*GPIO_PinSource1));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*GPIO_PinSource1);

	// Remap mode 01: TIM2 Partial mapping (CH1/ETR/PC5, CH2/PC2, CH3/PD2, CH4/PC1).
	AFIO->PCFR1 |= AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP1;

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
	TIM1->CTLR1 |= TIM_ARPE;
	TIM2->CTLR1 |= TIM_ARPE;

	// Set the Capture Compare Register value to 0 initially
	TIM1->CH2CVR = 0; //T1CH2/PA1/pin1
	TIM1->CH4CVR = 0; //T1CH4/PC4/pin7
	TIM2->CH2CVR = 0; //T2CH2/PC2/pin6*
	TIM2->CH4CVR = 0; //T2CH4/PC1/pin5*
	                    // * - alternate function

	TIM1->CCER |= TIM_CC2E | TIM_CC2P;	// Enable T1CH2 output, positive pol
	TIM1->CCER |= TIM_CC4E | TIM_CC4P;	// Enable T1CH4 output, positive pol
	TIM2->CCER |= TIM_CC2E | TIM_CC2P;	// Enable T2CH2 output, positive pol
	TIM2->CCER |= TIM_CC4E | TIM_CC4P;	// Enable T2CH4 output, positive pol

	TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1;	// CH2 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	TIM1->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1;	// CH4 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	TIM2->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2PE;
	TIM2->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1 | TIM_OC4PE;

	TIM1->BDTR  |= TIM_MOE;	// Brake and deadband registers
	TIM1->CTLR1 |= TIM_CEN;	// Enable TIM1
	TIM2->CTLR1 |= TIM_CEN;	// Enable TIM2
}

/* 
 * void GPIO_Low(GPIO_TypeDef* port, uint8_t pin, TIM_TypeDef* timer, uint16_t channel)
 *
 * Detaches pin from timer output, sets to OD output mode, sets output low
 *
 * Example: GPIO_Low(GPIOA, 1, TIM1, TIM_CC1E); // detach PA1 from T1C1, sets OD mode, Low
 * 
 */
void GPIO_Low(GPIO_TypeDef* port, uint8_t pin, TIM_TypeDef* timer, uint16_t channel) {
        timer->CCER &= ~channel;        //disable this channel output
        port->CFGLR &= ~(0xf<<(4*pin)); //clear GPIO config - this appears to be necessary. 
        port->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD)<<(4*pin); //Sets output mode to GPIO, OD
        port->BCR = (1<<(pin));         //set GPIO 'low'
};

/*
 * void GPIO_High(GPIO_TypeDef* port, uint8_t pin, TIM_TypeDef* timer, uint16_t channel)
 *
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
 * void GPIO_Float(GPIO_TypeDef* port, uint8_t pin, TIM_TypeDef* timer, uint16_t channel)
 *
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
 * void GPIO_PWM(GPIO_TypeDef* port, uint8_t pin, TIM_TypeDef* timer, uint16_t channel) {
 *
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
 * void pwm_setpw(TIM_TypeDef* timer, uint16_t chl, uint16_t width)
 *
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
 */
void systick_init(void) {
	/* disable default SysTick behavior */
	SysTick->CTLR = 0;
	
	/* enable the SysTick IRQ */
	NVIC_EnableIRQ(SysTicK_IRQn);

	/* Set the tick interval to SYSTICKHZ */
	SysTick->CMP = (SYSTEM_CORE_CLOCK/SYSTICKHZ)-1;
	
	/* Start at zero */
	SysTick->CNT = 0;
	
	/* Enable SysTick counter, IRQ, HCLK/1 */
	SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE |
					SYSTICK_CTLR_STCLK;
}


/****************************************************************************
 * Where the magic happens. Every time this function is called, it switches to the next
 * group of LED's and sets their brigtness. If this function is called frequently and 
 * consistently enough, persistence-of-vision makes it appear that each individual LED 
 * is able to be lit to any desired brigtness simultaneously.
 *
 * Best to call this from an ISR at a regular rate (millisecond/tens of milliseconds)
 *
 * Takes no arguments. Returns nothing.
 * *************************************************************************/
void updateLEDs(void)
{
	static uint8_t currentgroup = 0; //the common anode group of LEDs are being worked on

  	/* 1. Set current PWM pins to Float (PWM off) (probably not strictly necessary */
	for(int i=0;i<3;i++){
		GPIO_Float(	LEDs[group[currentgroup][i]].cathode->port,
				LEDs[group[currentgroup][i]].cathode->pin,
				LEDs[group[currentgroup][i]].cathode->timer,
				LEDs[group[currentgroup][i]].cathode->channel
			  );
	}
		
	/* 2. Select the next group of LEDs */
	if(++currentgroup == 4) currentgroup=0;
		
	/* 3. Update PWM brightness settings for the currently selected group */
	for(int i=0;i<3;i++){
		pwm_setpw(	LEDs[group[currentgroup][i]].cathode->timer,
				LEDs[group[currentgroup][i]].cathode->channel,
				LEDs[group[currentgroup][i]].brightness
			 );
	}
	
	/* 4. Set the currently selected group's cathodes (3 of them) to PWM */
	for(int i=0;i<3;i++){
		GPIO_PWM(	LEDs[group[currentgroup][i]].cathode->port,
				LEDs[group[currentgroup][i]].cathode->pin,
				LEDs[group[currentgroup][i]].cathode->timer,
				LEDs[group[currentgroup][i]].cathode->channel
			 );
	}
	
 	/* 5. Set currently selected group's anode pin HIGH */
	GPIO_High(	LEDs[group[currentgroup][0]].anode->port,
			LEDs[group[currentgroup][0]].anode->pin,
			LEDs[group[currentgroup][0]].anode->timer,
			LEDs[group[currentgroup][0]].anode->channel
		 );
}

/*
 * SysTick ISR called every millisaecond.
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
	SysTick->CMP += (SYSTEM_CORE_CLOCK/SYSTICKHZ); 

	/* clear IRQ */
	SysTick->SR = 0;
	updateLEDs();
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
