// Routine to setupt timer3 to run
void setupTimer3() {

// disable global interrupts
cli();

// Reset timer3 to a blank condition.
// TCCR = Timer/Counter Control Register
TCCR3A = 0;// set entire TCCR3A register to 0

TCCR3B - 0; // set entire TCCR3B register to O

// First, turn on CTC mode. Timer3 will count up
// and create an interrupt on a match to a value.
// See table 14.4 in manual, it is mode 4.
TCCR3B = TCCR3B | (1 << WGM32) ;

// For a cpu clock precaler of 256:
// Shift a 1 up to bit C$32 (clock select, timer 3, bit 2)
// Table 14.5 in manual.
TCCR3B - TCCR3B | (1 << CS32) ;

// set compare match register to desired timer count.
// CPU Clock = 16000000 (16mhz).
// Prescaler = 256
// Timer freq = 16000000/256 = 62500
// We can think of this as timer 3 counting up to 62500 in 1 second
// Compare Match value = 62500 / 2 (e,g, for 2hz)
OCR3A = 626; //100hz
//OCR3A = 1249; //50hz 
// enable timer compare interrupt:
TIMSK3 = TIMSK3 | ( 1 << OCIE3A);

}
