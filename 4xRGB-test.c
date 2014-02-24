// this tunes the accuracy of _delay_ms
// I've screwed with the normal timing to the point where I have to play with this from standard values to get it right.
#define F_CPU 270270

// common modules that we use in this program
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>



/**********************************************************************/
// formatted R/G/B LED 0,1,2,3
#define RED 0
#define GREEN 1
#define BLUE 2
volatile char displaybuffer[4][3];


// button tracking - this counts up every 1/xxx of a second that the button is down.
// when the count reaches defined values, things happen.
// or for some functions, when the button is released, different things happen based on how long it was down
int button_down_count = 0;


/***********************************************************************************/
// update this to the max mode when you add more modes
// should be one more than actual modes - the biggest one runs all patterns
#define max_mode 12
/***********************************************************************************/


// These indicate which mode we should be running
// the ISR changes button_mode immediately when the button is clicked
// the main program notices that the two no longer match and switches modes
volatile int button_mode = 1; // this is changed by the button within the ISR
volatile int current_mode = 1; // this follows button_mode when the main code reacts
volatile int shutdown_now = 0;


void SetupDisplayInterrupt()
{
   // Setup Timer 0
   TCCR0A = 0b00000000;   // Normal Mode
   TCCR0B = 0b00000001;   // No Prescaler
   TCNT0 = 0;        	// Initial value

// enable interrupts
   TIMSK = (1<<TOIE0);  	// Timer Mask: Enable interrupt on Timer 0 overflow
}

/**********************************************************************/
/**********************************************************************/
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
void Wheel(int WheelPos, int LEDnum) 
{
	if (WheelPos < 85) 
	{
   		displaybuffer[LEDnum][RED] = (WheelPos * 3)/4;
   		displaybuffer[LEDnum][GREEN] = (255 - WheelPos * 3)/4;
   		displaybuffer[LEDnum][BLUE]  = 0;
  	} 
  	else if(WheelPos < 170) 
  	{
   		WheelPos -= 85;
   		displaybuffer[LEDnum][RED] = (255 - WheelPos * 3)/4; 
   		displaybuffer[LEDnum][GREEN] = 0;
   		displaybuffer[LEDnum][BLUE] = (WheelPos * 3)/4;
  	}
  	else 
  	{
   		WheelPos -= 170;
   		displaybuffer[LEDnum][RED] = 0;
   		displaybuffer[LEDnum][GREEN] = (WheelPos * 3)/4;
   		displaybuffer[LEDnum][BLUE] = (255 - WheelPos * 3)/4;
  	}
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

	while (1)
	{
		for(j=0; j<256; j++) 
		{
			for(i=0; i<4; i++) {
				Wheel((i*16+j) & 255, i);
		}
	    _delay_ms(wait);
    }
  }
}

/**********************************************************************/
/**********************************************************************/
// this is the main function

void LEDTEST()
{
//	while (1)
	{
		for (int color=0; color<3; color++)
		for (int LED = 0; LED < 4; LED++)
		{
			displaybuffer[LED][color] = 15;
			_delay_ms(250);
			displaybuffer[LED][color] = 0;
		}
	}
}

int main()
{
    SetupDisplayInterrupt();
    sei();           	// Global enable Interrupts

//	ClearDisplay();
	LEDTEST(); // need a way to call this except on initial power-up
rainbow(10);

#if 0
	while (1)
	{
		if (shutdown_now)
		{
			shutdown_now = 0;
			shutdownPattern();
			button_down_count = 101;
			power_off();
		}
		if (button_mode != current_mode)
		{
			current_mode = button_mode;
			ModeSwitchPattern(); // this displays the current mode in binary for 1 second
		}
		else
		{
			int runmode;
			if (current_mode == max_mode)
			{
				runmode = rotator;
				rotator++;
			}
			else
			{
				runmode = current_mode;
			}
			switch (runmode)
			{
			}
		}
	}
#endif
	return 0;
}





// **********************************************************
// *** BUTTON PRESS HANDLING
// **********************************************************
//Setup pin change interrupt used to wake from sleep
void init_pcint(void)
{
  GIMSK |= 1<<PCIE;   // General Interrupt Mask: Enable Pin Change Interrupt
  PCMSK |= 1<<PCINT0; // Pin Change Mask: Watch for Pin Change on Pin5 (PB0)
}

//Pin Change Interrupt
ISR(PCINT0_vect)
{
  sleep_disable();
  GIMSK &= ~(1<<PCIE); //Disable the interrupt so it doesn't keep flagging
  PCMSK &= ~(1<<PCINT0);
}




// turn off and wait for button press
void power_off(void)
{
  cli();
  TIMSK &= ~(1<<TOIE0);  //Timer Interrupt Mask: Turn off the Timer Overflow Interrupt bit

  // don't proceed until the button is released
  DDRB = 0x0;   // PB0 to input mode
  PORTB = 0x01;   // pull-up active
  while ((PINB & 0x01) == 0);
  _delay_ms(50); // Wait a moment to make sure the button isn't bouncing

  // go into power down mode until the button gets pressed again
gotosleep:
  init_pcint();     	// set up pin change interrupt
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sei(); // enable interrupts (so we can come back alive again when the button is pressed)

  // this statement puts the CPU into power down mode until the pin change interrupt awakens it.
  sleep_mode();

  // when we hit here, we've awakened from sleep
  // interrupts will be disabled when we get back from here but RETI at the end of this function will enable interrupts
 
  // detect double click
  _delay_ms(50); // debounce button press
 
  while ((PINB & 0x01) == 0); // wait for button release
  _delay_ms(50); // debounce button release


  // button must be double clicked in 1/2 second, otherwise it was just something
  // randomly hitting the button and we go back to sleep.
  for (int x=0; x<500; x++) // 1/2 second to press button again
  {
      if ((PINB & 0x01) == 0) // button was pressed again
   	 goto wakeup;
    _delay_ms(1);
  }
  // no subsequent button press, false alarm
  goto gotosleep;
 
wakeup:
    button_down_count = 101; // don't switch modes upon release.
    SetupDisplayInterrupt(); // put interrupts back to run mode
}





// **********************************************************
// *** DISPLAY HANDLER
// **********************************************************

// even though the max value for luminance is 15, it helps to count a bit past that
// so even at full brightness there's some off time.  Otherwise the last 4 or 5 values
// are equally as bright.
#define cyclecount 35

// This gets called on timer interrupt
// this also detects button presses
ISR(TIM0_OVF_vect)
{
	static short loopcount=0;

	static unsigned char PWMCount = 0; // number of times we enter this function per PWM cycle (full brightness)
	static short LEDNumber = 0; // points to the set of 3 in displaybuffer that we're on right now
	static unsigned char compare[3];
	static unsigned char disableRed, disableGreen, disableBlue;

	// these are the values we calculate each time for the next entrance
	static unsigned char pinlevelB = 1<<0 | 1<<1 | 1<<4; // all 3 pins on
	static unsigned char DDRval = 0;

	if (compare[RED]   == PWMCount) { DDRval &= disableRed;   pinlevelB &= disableRed; }
	if (compare[GREEN] == PWMCount) { DDRval &= disableGreen; pinlevelB &= disableGreen; }
	if (compare[BLUE]  == PWMCount) { DDRval &= disableBlue;  pinlevelB &= disableBlue; }

	DDRB = 0;
	PORTB = pinlevelB;
   	DDRB = DDRval;
	
	++PWMCount;

	TCNT0 = 192;        	// Initial value = closer to 255 = we come in here more times per second

	if (PWMCount == cyclecount) // giving a little off time even at full on gives a little more dynamic range control
		PWMCount = 0; // and drop down below to go to the next LED in the rotation
	else
		return;

	// switch banks every full PWM cycle
	// every 16, we drop down here.  PWMcount will be zero and values will get reloaded next round.

	if (loopcount == 4)
		loopcount = 0; // start cycle over after last LED

	if (loopcount == 0)
	{
		LEDNumber = 0;

		// first LED = PB0 low, 1,2,3 high as needed for R/G/B
		DDRval = 1<<0 | 1<<1 | 1<<2 | 1<<3;
		pinlevelB = 1<<1 | 1<<2 | 1<<3; // red, green, blue on

		// these are applied to both DDRval and pinlevelB to shut off the color
		// so that it tristates and the pullup is disabled.
		disableRed = ~(1<<3);
		disableGreen = ~(1<<2);
		disableBlue = ~(1<<1);
	}
	if (loopcount == 1)
	{
		LEDNumber = 1;

		// first LED = PB0 low, 1,2,3 high as needed for R/G/B
		DDRval = 1<<1 | 1<<2 | 1<<3 | 1<<4;
		pinlevelB = 1<<2 | 1<<3 | 1<<4; // red, green, blue on

		// these are applied to both DDRval and pinlevelB to shut off the color
		// so that it tristates and the pullup is disabled.
		disableRed = ~(1<<2);
		disableGreen = ~(1<<3);
		disableBlue = ~(1<<4);
	}

	if (loopcount == 2)
	{
		LEDNumber = 2;

		// first LED = PB0 low, 1,2,3 high as needed for R/G/B
		DDRval = 1<<1 | 1<<2 | 1<<3 | 1<<4;
		pinlevelB = 1<<1 | 1<<2 | 1<<3; // red, green, blue on

		// these are applied to both DDRval and pinlevelB to shut off the color
		// so that it tristates and the pullup is disabled.
		disableRed = ~(1<<1);
		disableGreen = ~(1<<2);
		disableBlue = ~(1<<3);
	}

	if (loopcount == 3)
	{
		LEDNumber = 3;

		// first LED = PB0 low, 1,2,3 high as needed for R/G/B
		DDRval = 1<<0 | 1<<1 | 1<<2 | 1<<3;
		pinlevelB = 1<<0 | 1<<1 | 1<<2; // red, green, blue on

		// these are applied to both DDRval and pinlevelB to shut off the color
		// so that it tristates and the pullup is disabled.
		disableRed = ~(1<<0);
		disableGreen = ~(1<<1);
		disableBlue = ~(1<<2);
	}

	// Load the values for each LED at the beginning of the cycle for it.
	compare[RED]   = displaybuffer[LEDNumber][RED];
	compare[GREEN] = displaybuffer[LEDNumber][GREEN];
	compare[BLUE]  = displaybuffer[LEDNumber][BLUE];

	loopcount++;
}
