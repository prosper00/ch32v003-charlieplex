/* Charlieplexing PWM demo on the CH32V003 RISC-V mcu
 *
 * For the 8-pin package, the default pin assignment avoids using SWIO, and
 * leaves one additional GPIO/ADC pin free for other uses.
 *
 * Pin Map: (See schematic PNG for circuit diagram of the LEDs)
 * pin 1: cathode group 1	pin 8: SWIO/debug
 * pin 2: GND			pin 7: free
 * pin 3: cathode group 2	pin 6: cathode group 4
 * pin 5: 3V3			pin 5: cathode group 3
 *
 * in a charlieplexed matrix, the maximum number of LEDs is equalt to n*(n-1)
 * where n is the number of GPIOs used. Only LEDs sharing a common cathode 
 * (or a common anode) can be lit simultaneously. This should be equal to 
 * (n-1). So for an n=4 array, there can be 12 LEDs in total, and 3 can be 
 * lit simultaneously. This implies that max brightness can be 25% in a 
 * PWM configuration.
 *
 * This is not the most memory-efficient way to do this. However, a relatively 
 * high degree of abstraction is offered, allowing a reasonable flexibility in
 * the assignment of GPIOs and the order of the LEDs available in software.
 *
 * (maybe) TODO 1 - Hardware PWM. This would require all LED GPIOs to be timer 
 * outputs, and then switch the common cathode groups every millisecond or so. 
 * Ought to be much less CPU intensive and higher resolution than the current
 * software PWM approach. 
 * On the smallest CH32v003 package (SOP8), there are exactly 4 timer pins
 * available (reserving pin 8 for SWIO programming/debugging):
 * pin1=T1CH2, pin7=T1CH1, pin6=T2CH2, pin5=T2CH4
 * this leaves pin3 free for other uses, including analog input.
 *
 * (maybe) TODO 2: allow for matrices other than 4-wire
 *
 * BUG: brightness values of '0' cause... weirdness. Occasionally, '0' seems to
 * generate much higher than expected brigtness to be output. Dunno why.
 *
 */


// Could be defined here, or in the processor defines.
#define SYSTEM_CORE_CLOCK 48000000
#define APB_CLOCK SYSTEM_CORE_CLOCK
#define SYSTICK_USE_HCLK

#include "ch32v003fun.h"
#include "wiring.h"
#include <stdio.h>

/* some bit definitions for systick regs */
#define SYSTICK_SR_CNTIF (1<<0)
#define SYSTICK_CTLR_STE (1<<0)
#define SYSTICK_CTLR_STIE (1<<1)
#define SYSTICK_CTLR_STCLK (1<<2)
#define SYSTICK_CTLR_STRE (1<<3)
#define SYSTICK_CTLR_SWIE (1<<31)

#define SYSTICKHZ 32000

uint32_t count;
volatile uint32_t systick_cnt;


/* this is the truth table for the LEDs, defining how they're connected
 * and how to set the GPIO pins to select a specific LED. (unused GPIOs
 * should be set as floating inputs)
 *
 * could be a const, if I find a better place for 'brightness' to live.
 */
typedef struct {
	const uint8_t pinCathode; //pin to set LOW
	const uint8_t pinAnode;	  //pin to set HIGH
	uint8_t brightness;
} Connection;

volatile Connection LEDs[12] = { 
//	 GPIO LO  GPIO HI
	{ pin_A1, pin_A2, 16 },	// LED 0
	{ pin_A2, pin_A1, 63 },	// LED 1
	{ pin_A2, pin_C1, 32 },	// LED 2
	{ pin_C1, pin_A2, 16 },	// LED 3
	{ pin_A1, pin_C1, 7  },	// LED 4
	{ pin_C1, pin_A1, 6  },	// LED 5
	{ pin_C2, pin_C1, 5  },	// LED 6
	{ pin_C1, pin_C2, 4  },	// LED 7
	{ pin_A2, pin_C2, 3  },	// LED 8
	{ pin_C2, pin_A2, 2  },	// LED 9
	{ pin_A1, pin_C2, 1  },	// LED 10
	{ pin_C2, pin_A1, 1  }	// LED 11
};

/* to be used to iterate through each group of LEDs. 
 * TODO: calculate this at runtime from the data in the
 * Connection struct
 */
static const uint8_t group[4][3] = {	//4 cathode groups of 3 LEDs each
	{ 0, 4, 10 },	// LEDs sharing pin_A1
	{ 1, 2, 8  },	// LEDs sharing pin_A2
	{ 3, 5, 7  },	// LEDs sharing pin_C1
	{ 6, 9, 11 }	// LEDs sharing pin_C2
};

/*
 * Setup systick to fire an interrput at SYSTICKHZ Hz
 */
void systick_init(void)
{
	/* disable default SysTick behavior */
	SysTick->CTLR = 0;
	
	/* enable the SysTick IRQ */
	NVIC_EnableIRQ(SysTicK_IRQn);
	
	/* Set the tick interval to 1ms for normal op */
	/*SysTick->CMP = (SYSTEM_CORE_CLOCK/1000)-1; */

	/* Set the tick interval to 16kHz */
	SysTick->CMP = (SYSTEM_CORE_CLOCK/SYSTICKHZ)-1;
	
	/* Start at zero */
	SysTick->CNT = 0;
	systick_cnt = 0;
	
	/* Enable SysTick counter, IRQ, HCLK/1 */
	SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE |
					SYSTICK_CTLR_STCLK;
}

/*
 * SysTick ISR does all the work.
 *
 * pseudo-code:
 *
 * while(1){
 * 	for(int ramp=0; ramp <64; ramp++){
 * 		1. check each LED in the group to see if it's been on for long 
 *	 	enough, and then turn it off;
 * 		2. delay();
 *	 }
 * 	1. turn off the common pin of the group;
 * 	2. advance to next group;
 * 	3. turn on the common pin of the new group;
 * 	4. turn on all all leds in the new group;
 * }
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

	/* update counter */
	systick_cnt++;

	static uint8_t ramp,  
		       currentgroup; //the common cathode group of LEDs are being worked on

	ramp=systick_cnt & 0x3F;		// 0 to 63
	
	if (ramp == 0) {
		pinMode(LEDs[group[currentgroup][0]].pinCathode, pinMode_I_floating); // switch off this group
										      
		currentgroup = (currentgroup + 1) & 0x3;	// move to next group
	
		pinMode(LEDs[group[currentgroup][0]].pinCathode, pinMode_O_pushPull); // switch on the group
		digitalWrite(LEDs[group[currentgroup][0]].pinCathode, low);
		
		pinMode(LEDs[group[currentgroup][0]].pinAnode, pinMode_O_pushPull); // switch on this LED
		digitalWrite(LEDs[group[currentgroup][0]].pinAnode, high);

		pinMode(LEDs[group[currentgroup][1]].pinAnode, pinMode_O_pushPull); // switch on this LED
		digitalWrite(LEDs[group[currentgroup][1]].pinAnode, high);
		
		pinMode(LEDs[group[currentgroup][2]].pinAnode, pinMode_O_pushPull); // switch on this LED
		digitalWrite(LEDs[group[currentgroup][2]].pinAnode, high);
	}
	if (LEDs[group[currentgroup][0]].brightness == ramp) { // if the target pulse width has been achieved
		pinMode(LEDs[group[currentgroup][0]].pinAnode, pinMode_I_floating); // switch off this LED
	}
	if (LEDs[group[currentgroup][1]].brightness == ramp) {
		pinMode(LEDs[group[currentgroup][1]].pinAnode, pinMode_I_floating); // switch off this LED
	}
	if (LEDs[group[currentgroup][2]].brightness == ramp) {
		pinMode(LEDs[group[currentgroup][2]].pinAnode, pinMode_I_floating); // switch off this LED
	}
}

void Delay_Ticks(uint32_t n) {/* 1ms = SYSTICKHZ/1000 ticks */
	uint32_t end = systick_cnt + n; 
	while( ((int32_t)(systick_cnt - end )) < 0 );
}

void Delay_ms_ticks(uint32_t n) {
	Delay_Ticks(SYSTICKHZ/1000*n);
}

/*
 * main() - does the stuff
 */
int main() {
	SystemInit48HSI();
	systick_init();

	// Enable GPIO ports
	portEnable(port_A);
	portEnable(port_C);

	// all to hi-Z // LED 'off'
	pinMode(pin_A1, pinMode_I_floating);
	pinMode(pin_A2, pinMode_I_floating);
	pinMode(pin_C1, pinMode_I_floating);
	pinMode(pin_C2, pinMode_I_floating);

	while(1){
				
		Delay_ms_ticks(30);
		int temp=LEDs[0].brightness;
		for(int i=1; i<12; i++){
			LEDs[i-1].brightness=LEDs[i].brightness;
		}
		LEDs[11].brightness=temp;
	}

}
