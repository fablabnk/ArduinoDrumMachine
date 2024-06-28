// dsp-D8 Drum Chip (c) DSP Synthesizers 2015
// Free for non commercial use

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "drum_samples.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Standard Arduino Pins
#define digitalPinToPortReg(P) \
	(((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define digitalPinToDDRReg(P) \
	(((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define digitalPinToPINReg(P) \
	(((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define digitalPinToBit(P) \
	(((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (P) - 14))

#define digitalReadFast(P) bitRead(*digitalPinToPINReg(P), digitalPinToBit(P))

#define digitalWriteFast(P, V) bitWrite(*digitalPinToPortReg(P), digitalPinToBit(P), (V))

const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

//--------- Ringbuf parameters ----------
uint8_t Ringbuffer[256];
uint8_t RingWrite = 0;
uint8_t RingRead = 0;
volatile uint8_t RingCount = 0;

const int LFilterPin = A4;
float alpha = 0.1;
int16_t filteredOutput = 0;

//-----------------------------------------
ISR(TIMER1_COMPA_vect)
{

	//-------------------  Ringbuffer handler -------------------------

	if (RingCount)
	{ // If entry in FIFO..
		int16_t rawOutput = Ringbuffer[RingRead++];
		filteredOutput = (alpha * rawOutput) + ((1 - alpha) * filteredOutput);
		OCR2A = filteredOutput; // Output LSB of 16-bit DAC
		RingCount--;
	}

	//-----------------------------------------------------------------
}

// Define the array of arrays
const uint8_t *const drumSamples[8] PROGMEM = {BD, SN, CH, OH, RS, CL, RD, CR};
int buttonPins[8] = {2, 3, 4, 5, 6, 7, 8, 9}; // or byte?
bool currentButtonStates[8] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};
bool lastButtonStates[8] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
const int pitchPins[3] = {A0, A1, A2};
uint8_t phaccs[8];
uint8_t pitches[8] = {128, 64, 64, 64, 64, 64, 16, 16};
uint16_t samplelens[8] = {2154, 3482, 482, 2572, 1160, 2384, 5066, 5414};
uint16_t samplecnts[8];
uint16_t samplepnts[8];

uint8_t oldPORTB;
uint8_t oldPORTD;

int16_t total;
uint8_t divider;
uint8_t MUX = 0;
char ser = ' ';

void setup()
{
	OSCCAL = 0xFF;

	// Drumtrigger inputs
	for (int i = 0; i < 8; i++)
	{ // byte
		pinMode(buttonPins[i], INPUT_PULLUP);
	}

	// Pitch inputs
	for (int i = 0; i < 3; i++)
	{ // byte
		pinMode(pitchPins[i], INPUT);
	}

	pinMode(LFilterPin, INPUT);

	// 8-bit PWM DAC pin
	pinMode(11, OUTPUT);

	// Set up Timer 1 to send a sample every interrupt.
	cli();
	// Set CTC mode
	// Have to set OCR1A *after*, otherwise it gets reset to 0!
	TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
	TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
	// No prescaler
	TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
	// Set the compare register (OCR1A).
	// OCR1A is a 16-bit register, so we have to do this with
	// interrupts disabled to be safe.
	// OCR1A = F_CPU / SAMPLE_RATE;
	// Enable interrupt when TCNT1 == OCR1A
	TIMSK1 |= _BV(OCIE1A);
	OCR1A = 400; // 40KHz Samplefreq

	// Set up Timer 2 to do pulse width modulation on D11

	// Use internal clock (datasheet p.160)
	ASSR &= ~(_BV(EXCLK) | _BV(AS2));

	// Set fast PWM mode  (p.157)
	TCCR2A |= _BV(WGM21) | _BV(WGM20);
	TCCR2B &= ~_BV(WGM22);

	// Do non-inverting PWM on pin OC2A (p.155)
	// On the Arduino this is pin 11.
	TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
	TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
	// No prescaler (p.158)
	TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

	// Set initial pulse width to the first sample.
	OCR2A = 128;

	// set timer0 interrupt at 61Hz
	TCCR0A = 0; // set entire TCCR0A register to 0
	TCCR0B = 0; // same for TCCR0B
	TCNT0 = 0;	// initialize counter value to 0
	// set compare match register for 62hz increments
	OCR0A = 255; // = 61Hz
	// turn on CTC mode
	TCCR0A |= (1 << WGM01);
	// Set CS01 and CS00 bits for prescaler 1024
	TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00); // 1024 prescaler

	TIMSK0 = 0;

	// set up the ADC
	ADCSRA &= ~PS_128; // remove bits set by Arduino library
	// Choose prescaler PS_128.
	ADCSRA |= PS_128;
	ADMUX = 64;
	sbi(ADCSRA, ADSC);

	filteredOutput = 128;
	
	Serial.begin(9600);
	Serial.println("Modified [Jan Ostman] dsp-D8:");
	Serial.println("https://janostman.wordpress.com/the-dsp-d8-drum-chip-source-code/");
	Serial.println("Press asdfjkl; to play the drums.");

	sei();
}

void loop()
{

	//------ Add current sample word to ringbuffer FIFO --------------------

	if (RingCount < 255)
	{ // if space in ringbuffer
		total = 0;

		// we can't wrap the following a for loop, for some reason when we do the samples get corrupted, or we have side-effects/compiler optimising issues
		if (samplecnts[0])
		{
			phaccs[0] += pitches[0];
			if (phaccs[0] & 128)
			{
				phaccs[0] &= 127;
				samplepnts[0]++;
				samplecnts[0]--;
			}
			total += (pgm_read_byte_near(drumSamples[0] + samplepnts[0]) - 128);
		}

		if (samplecnts[1])
		{
			phaccs[1] += pitches[1];
			if (phaccs[1] & 128)
			{
				phaccs[1] &= 127;
				samplepnts[1]++;
				samplecnts[1]--;
			}
			total += (pgm_read_byte_near(drumSamples[1] + samplepnts[1]) - 128);
		}

		if (samplecnts[2])
		{
			phaccs[2] += pitches[2];
			if (phaccs[2] & 128)
			{
				phaccs[2] &= 127;
				samplepnts[2]++;
				samplecnts[2]--;
			}
			total += (pgm_read_byte_near(drumSamples[2] + samplepnts[2]) - 128);
		}

		if (samplecnts[3])
		{
			phaccs[3] += pitches[3];
			if (phaccs[3] & 128)
			{
				phaccs[3] &= 127;
				samplepnts[3]++;
				samplecnts[3]--;
			}
			total += (pgm_read_byte_near(drumSamples[3] + samplepnts[3]) - 128);
		}

		if (samplecnts[4])
		{
			phaccs[4] += pitches[4];
			if (phaccs[4] & 128)
			{
				phaccs[4] &= 127;
				samplepnts[4]++;
				samplecnts[4]--;
			}
			total += (pgm_read_byte_near(drumSamples[4] + samplepnts[4]) - 128);
		}

		if (samplecnts[5])
		{
			phaccs[5] += pitches[5];
			if (phaccs[5] & 128)
			{
				phaccs[5] &= 127;
				samplepnts[5]++;
				samplecnts[5]--;
			}
			total += (pgm_read_byte_near(drumSamples[5] + samplepnts[5]) - 128);
		}

		if (samplecnts[6])
		{
			phaccs[6] += pitches[6];
			if (phaccs[6] & 128)
			{
				phaccs[6] &= 127;
				samplepnts[6]++;
				samplecnts[6]--;
			}
			total += (pgm_read_byte_near(drumSamples[6] + samplepnts[6]) - 128);
		}

		if (samplecnts[7])
		{
			phaccs[7] += pitches[7];
			if (phaccs[7] & 128)
			{
				phaccs[7] &= 127;
				samplepnts[7]++;
				samplecnts[7]--;
			}
			total += (pgm_read_byte_near(drumSamples[7] + samplepnts[7]) - 128);
		}

		total >>= 1;
		if (!(PINB & 4))
			total >>= 1;
		total += 128;
		if (total > 255)
			total = 255;

		// just one button for triggering bass drum for now
		currentButtonStates[0] = digitalRead(buttonPins[0]);
		if (lastButtonStates[0] == HIGH && currentButtonStates[0] == LOW)
		{
			trigger(0);
		}
		lastButtonStates[0] = currentButtonStates[0];

		cli();
		Ringbuffer[RingWrite] = total;
		RingWrite++;
		RingCount++;
		sei();
	}

	//----------------------------------------------------------------------------

	//----------------- Handle Triggers ------------------------------
	if (Serial.available() > 0)
	{
		ser = Serial.read();
		Serial.write(ser);
		switch (ser)
		{
		case ('a'):
			trigger(0);
			break;
		case ('s'):
			trigger(1);
			break;
		case ('d'):
			trigger(2);
			break;
		case ('f'):
			trigger(3);
			break;
		case ('j'):
			trigger(4);
			break;
		case ('k'):
			trigger(5);
			break;
		case ('l'):
			trigger(6);
			break;
		case (';'):
			trigger(7);
			break;
		default:
			break;
		}
	}

	//-----------------------------------------------------------------
}

void trigger(int drumNum)
{
	// int potValue = analogRead(A0);  // Read the potentiometer value (0-1023)
	// pitches[drumNum] = map(potValue, 0, 1023, 16, 128);  // Map the value to the desired pitch range (16-128)
	samplepnts[drumNum] = 0;
	samplecnts[drumNum] = samplelens[drumNum]; // hmm - need to set length at all?
}