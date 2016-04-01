/* Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 *
 * Version 1.2 - fix -2 bug in C-only code
 * Version 1.1 - expand to support boards with up to 60 interrupts
 * Version 1.0 - initial release
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#ifndef EncoderMod_h_
#define EncoderMod_h_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#elif defined(WIRING)
#include "Wiring.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#include "utility/direct_pin_read.h"

#if defined(ENCODER_USE_INTERRUPTS) || !defined(ENCODER_DO_NOT_USE_INTERRUPTS)
#define ENCODER_USE_INTERRUPTS
#define ENCODER_ARGLIST_SIZE CORE_NUM_INTERRUPT
#include "utility/interrupt_pins.h"
#ifdef ENCODER_OPTIMIZE_INTERRUPTS
#include "utility/interrupt_config.h"
#endif
#else
#define ENCODER_ARGLIST_SIZE 0
#endif

#define FILTER_TIME_LIMIT 10000 //uS at 100/255 pwm  we get about 1/0.004 = 250uS/tick so this averages about 4 ticks
#define US_INTERVAL 50 //This must be lower than time difference between ticks will ever be.
#define FILTER_INTERVALS (FILTER_TIME_LIMIT/US_INTERVAL)
#define MAX_BUFFER_SIZE 5000 //This needs to be larger than the max number of ticks that can happen in our filter time limit it needs to be larger so that memory access violations don't occur


// All the data needed by interrupts is consolidated into this ugly struct
// to facilitate assembly language optimizing of the speed critical update.
// The assembly code uses auto-incrementing addressing modes, so the struct
// must remain in exactly this order.
typedef struct {
	volatile IO_REG_TYPE * pin1_register;
	volatile IO_REG_TYPE * pin2_register;
	IO_REG_TYPE            pin1_bitmask;
	IO_REG_TYPE            pin2_bitmask;
	uint8_t                state;
	int32_t                position;
	unsigned long stepTime1;
	unsigned long stepTime2;
	signed int uSBuffer[MAX_BUFFER_SIZE];
	int32_t bufferIndex;
	int32_t newTicks;
	unsigned long timeOfLastTick;
} Encoder_internal_state_t;

class Encoder
{
public:
    
	Encoder(uint8_t pin1, uint8_t pin2) {
		#ifdef INPUT_PULLUP
		pinMode(pin1, INPUT_PULLUP);
		pinMode(pin2, INPUT_PULLUP);
		#else
		pinMode(pin1, INPUT);
		digitalWrite(pin1, HIGH);
		pinMode(pin2, INPUT);
		digitalWrite(pin2, HIGH);
		#endif
		encoder.pin1_register = PIN_TO_BASEREG(pin1);
		encoder.pin1_bitmask = PIN_TO_BITMASK(pin1);
		encoder.pin2_register = PIN_TO_BASEREG(pin2);
		encoder.pin2_bitmask = PIN_TO_BITMASK(pin2);
		encoder.position = 0;
		// allow time for a passive R-C filter to charge
		// through the pullup resistors, before reading
		// the initial state
		delayMicroseconds(2000);
		uint8_t s = 0;
        encoder.stepTime1 = micros();
		encoder.stepTime2 = encoder.stepTime1;
		memset(encoder.uSBuffer, 0, MAX_BUFFER_SIZE*sizeof(int));
		encoder.bufferIndex = 0;
		encoder.newTicks = 0;
		encoder.timeOfLastTick = micros();
		if (DIRECT_PIN_READ(encoder.pin1_register, encoder.pin1_bitmask)) s |= 1;
		if (DIRECT_PIN_READ(encoder.pin2_register, encoder.pin2_bitmask)) s |= 2;
		encoder.state = s;
#ifdef ENCODER_USE_INTERRUPTS
		interrupts_in_use = attach_interrupt(pin1, &encoder);
		interrupts_in_use += attach_interrupt(pin2, &encoder);
#endif
		//update_finishup();  // to force linker to include the code (does not work)
	}


#ifdef ENCODER_USE_INTERRUPTS
	inline int32_t read() {
        uint8_t old_SREG = SREG;
		if (interrupts_in_use < 2) {
			noInterrupts();
			update(&encoder);
		} else {
			noInterrupts();
		}
		int32_t ret = encoder.position;
        SREG = old_SREG;
		return ret;
	}
	inline void write(int32_t p) {
        uint8_t old_SREG = SREG;
		noInterrupts();
		encoder.position = p;
        SREG = old_SREG;
	}
	
	inline float stepRate() {
        int8_t old_SREG = SREG;
        if (interrupts_in_use < 2) {
            noInterrupts();
            update(&encoder);
        }
        else {
            noInterrupts();
        }

		float velocitySum = 0;
		int index = encoder.bufferIndex - 1;
		if(index < 0){
			index += MAX_BUFFER_SIZE;
		}
		int timeSum = 0;
		int ticks = 0;
		unsigned long timeSinceLastTick = micros() - encoder.timeOfLastTick;
		
        SREG = old_SREG;
		interrupts();

		if(encoder.uSBuffer[index] == 0){
			//The buffer is not initialized in the first position so just stop with what we have now. there has never been a tick
			return 0.0;
		}
		//int sumIntervals = timeSinceLastTick/US_INTERVAL;
		int sumIntervals = 0;
		if(timeSinceLastTick > FILTER_TIME_LIMIT){
			return 0.0;
		}
		
		while(1){
			//Get how many discrete intervals
			if(encoder.uSBuffer[index] == 0){
				//The buffer is not initialized so just stop with what we have now.
				break;
			}
			int intervals = abs(encoder.uSBuffer[index])/US_INTERVAL;
			sumIntervals += intervals;
			if(sumIntervals <= FILTER_INTERVALS){
				velocitySum += intervals * (2.0/(float)encoder.uSBuffer[index]);
			} else {
				velocitySum += (intervals - (sumIntervals - FILTER_INTERVALS)) * (2.0/(float)encoder.uSBuffer[index]);
				break;
			}
			index--;
			if(index < 0 ){
				index += MAX_BUFFER_SIZE;
			}
		}
		float velocity = velocitySum / (float)FILTER_INTERVALS;
        return velocity;
    }
	
	
	inline float stepRateOptimized(){
		int8_t old_SREG = SREG;
        if (interrupts_in_use < 2) {
            noInterrupts();
            update(&encoder);
        }
        else {
            noInterrupts();
        }

		float static velocitySum = 0;
		int static timeSum = 0;
		//int static oldTicks = 0;
		int static ticksInAverage = 0;
		//int ticks = 0;
		int bufferIndex = encoder.bufferIndex - 1;
		if(bufferIndex < 0){
			bufferIndex+=MAX_BUFFER_SIZE;
		}
		int newTicks = encoder.newTicks;
		encoder.newTicks = 0;
		
        SREG = old_SREG;
		interrupts();
		
		//Only do this if we have newTicks
		if(newTicks > 0){
			//Add the all the newticks
			int stopIndex = bufferIndex - newTicks; //We are going backwards, stop at the index where the newticks stop
			if(stopIndex < 0){
				stopIndex += MAX_BUFFER_SIZE;
			}
			int i = bufferIndex;
			//timeSum = 0;
			//Serial.print("Newticks: ");
			//Serial.print(newTicks);
			//Serial.print("    ");
			//Serial.println();
			//Serial.print("velocitysum before:");
			//Serial.println(velocitySum);
			//Serial.print("    ");
			for(int j = 0; j < newTicks; j++){
				timeSum += abs(encoder.uSBuffer[i]); //Add the time to the buffer
				velocitySum += 1.0/(float)encoder.uSBuffer[i]; //Should catch this access to uSBuffer, also this is just adding the newvelocities to the sum this could lose precision on the float
				//Serial.print("velocitysum:");
				//Serial.print(velocitySum);
				//Serial.print("    ");
				//Serial.print("added:");
				//Serial.println(1.0/(float)encoder.uSBuffer[i]);
				/*if(abs(velocitySum) > 100){
					int temp = encoder.uSBuffer[i];
					//Serial.print("    ");
					//delay(5000);
					Serial.print("index:");
					Serial.print(i);
					Serial.print("   ");
					Serial.print("buffer value:");
					Serial.print(temp);
					Serial.print("    ");
					Serial.print("velocitysum:");
					Serial.print(velocitySum);
					Serial.print("    ");
					Serial.print("added:");
					Serial.print(1.0/temp);
					Serial.print("    ");
					delay(5000);
					/*for(int i = 0; i < 100000; i++){
						Serial.println(temp);
						delay(10);
					}*/
				//}
				//if(abs(velocitySum) > 10000000000){
				//	velocitySum = 0; //Should catch this access to uSBuffer, also this is just adding the newvelocities to the sum this could lose precision on the float
				//}
				ticksInAverage++; //Increase the number of ticks
				i--;
				if(i <= stopIndex){
					break;
				}
				if(i < 0){
					i += MAX_BUFFER_SIZE;
				}
			}
			//velocitySum = 0;
			//Serial.print("velocitysum:");
			//Serial.print(velocitySum);
			//Serial.print("    ");
			/*if(abs(velocitySum) > 10000000000){
				Serial.print("BAD   ");
			}*/
			
			
			if(ticksInAverage > MAX_BUFFER_SIZE){
				ticksInAverage = MAX_BUFFER_SIZE;
			}

			int startIndex = bufferIndex - ticksInAverage;
			//Serial.print("start:");
			//Serial.print(startIndex);
			//Serial.print("   ");
			if(startIndex < 0){
				startIndex += MAX_BUFFER_SIZE;
			}
			/*Serial.print("start:");
			Serial.print(startIndex);
			Serial.print("   ");
			Serial.print(ticksInAverage);
			Serial.print("   ");
			//Serial.print(timeSum);
			//Serial.print("   ");*/
			int loopcounts = 0;
			int totalTicks = ticksInAverage;
			for(int i = 0; i < totalTicks; i++){
			//for(int i = startIndex; i < bufferIndex; i = (i+1)%MAX_BUFFER_SIZE) { //Stop just before our current buffer index if we do get that far
				loopcounts++;
				if(timeSum < FILTER_TIME_LIMIT){ //This is our normal case. Stop subtracting when we are back within bounds
					/*Serial.print("TIMELIMIT:");
					Serial.print(ticksInAverage);
					Serial.print("    ");
					Serial.print(timeSum);
					Serial.print("   ");*/
					break;
				}

				int index = bufferIndex-totalTicks+i;
				if(index < 0)
					index+=MAX_BUFFER_SIZE;
				timeSum -= abs(encoder.uSBuffer[index]); //remove the time;
				ticksInAverage--; //We have less ticks now
				velocitySum -= 1.0/(float)encoder.uSBuffer[index]; //Again this access could be cached
			}
			/*Serial.print("loops:");
			Serial.print(loopcounts);
			Serial.print("    ");
			Serial.print(ticksInAverage);
			Serial.print("   ");
			Serial.print(timeSum);
			Serial.print("   ");*/
		}
		
		float velocity = velocitySum / (float)(ticksInAverage/2.0);
		Serial.print("ticksinAverage: ");
		Serial.print(ticksInAverage);
		Serial.print("   ");
		//Serial.print("VELOCITY: ");
		//Serial.println(velocity * 10000);
        return velocity;
	}
    /*inline float extrapolate() {
        uint8_t old_SREG = SREG;
        if (interrupts_in_use < 2) {
            noInterrupts();
            update(&encoder);
        }
        else {
            noInterrupts();
        }
        float lastRate = encoder.rate;
        int32_t lastPosition = encoder.position;
        float extrapolatedPosition = encoder.stepTime;
        float lastAccel = encoder.accel;
        SREG = old_SREG;
        extrapolatedPosition = lastRate * extrapolatedPosition + 0.5 * lastAccel * extrapolatedPosition * extrapolatedPosition;
        if (extrapolatedPosition > 1) {
            return (lastPosition + 1);
        }
        else if (extrapolatedPosition < -1) {
            return (lastPosition - 1);
        }
        else {
            return (extrapolatedPosition + lastPosition);
        }
    }*/
#else
	inline int32_t read() {
		update(&encoder);
		return encoder.position;
	}
	inline void write(int32_t p) {
		encoder.position = p;
	}
    inline float stepRate() {
        float extrapolatedPosition = encoder.rate * encoder.stepTime + 0.5 * encoder.accel * encoder.stepTime * encoder.stepTime;
        
        if (extrapolatedPosition > 1) {
            return (1 / encoder.stepTime);
        }
        else if (extrapolatedPosition < -1) {
            return (-1 / encoder.stepTime);
        }
        else {
            return (encoder.rate + encoder.accel * encoder.stepTime)
        }
        
    }
    inline float extrapolate() {
        float extrapolatedPosition = encoder.rate * encoder.stepTime + 0.5 * encoder.accel * encoder.stepTime * encoder.stepTime;
        if (extrapolatedPosition > 1) {
            return (encoder.position + 1);
        }
        else if (extrapolatedPosition < -1) {
            return (encoder.position - 1);
        }
        else {
            return (extrapolatedPosition + encoder.position);
        }
    }
#endif
private:
    Encoder_internal_state_t encoder;
#ifdef ENCODER_USE_INTERRUPTS
	uint8_t interrupts_in_use;
#endif
public:
	static Encoder_internal_state_t * interruptArgs[ENCODER_ARGLIST_SIZE];

//                           _______         _______       
//               Pin1 ______|       |_______|       |______ Pin1
// negative <---         _______         _______         __      --> positive
//               Pin2 __|       |_______|       |_______|   Pin2

		//	new	new	old	old
		//	pin2	pin1	pin2	pin1	Result
		//	----	----	----	----	------
		//	0	0	0	0	no movement
		//	0	0	0	1	+1
		//	0	0	1	0	-1
		//	0	0	1	1	+2  (assume pin1 edges only)
		//	0	1	0	0	-1
		//	0	1	0	1	no movement
		//	0	1	1	0	-2  (assume pin1 edges only)
		//	0	1	1	1	+1
		//	1	0	0	0	+1
		//	1	0	0	1	-2  (assume pin1 edges only)
		//	1	0	1	0	no movement
		//	1	0	1	1	-1
		//	1	1	0	0	+2  (assume pin1 edges only)
		//	1	1	0	1	-1
		//	1	1	1	0	+1
		//	1	1	1	1	no movement
/*
	// Simple, easy-to-read "documentation" version :-)
	//
	void update(void) {
		uint8_t s = state & 3;
		if (digitalRead(pin1)) s |= 4;
		if (digitalRead(pin2)) s |= 8;
		switch (s) {
			case 0: case 5: case 10: case 15:
				break;
			case 1: case 7: case 8: case 14:
				position++; break;
			case 2: case 4: case 11: case 13:
				position--; break;
			case 3: case 12:
				position += 2; break;
			default:
				position -= 2; break;
		}
		state = (s >> 2);
	}
*/

private:
	static void update(Encoder_internal_state_t *arg) {
#if defined(__AVR__)
		// The compiler believes this is just 1 line of code, so
		// it will inline this function into each interrupt
		// handler.  That's a tiny bit faster, but grows the code.
		// Especially when used with ENCODER_OPTIMIZE_INTERRUPTS,
		// the inline nature allows the ISR prologue and epilogue
		// to only save/restore necessary registers, for very nice
		// speed increase.
		asm volatile (
			"ld	r30, X+"		"\n\t"
			"ld	r31, X+"		"\n\t"
			"ld	r24, Z"			"\n\t"	// r24 = pin1 input
			"ld	r30, X+"		"\n\t"
			"ld	r31, X+"		"\n\t"
			"ld	r25, Z"			"\n\t"  // r25 = pin2 input
			"ld	r30, X+"		"\n\t"  // r30 = pin1 mask
			"ld	r31, X+"		"\n\t"	// r31 = pin2 mask
			"ld	r22, X"			"\n\t"	// r22 = state
			"andi	r22, 3"			"\n\t"
			"and	r24, r30"		"\n\t"
			"breq	L%=1"			"\n\t"	// if (pin1)
			"ori	r22, 4"			"\n\t"	//	state |= 4
		"L%=1:"	"and	r25, r31"		"\n\t"
			"breq	L%=2"			"\n\t"	// if (pin2)
			"ori	r22, 8"			"\n\t"	//	state |= 8
		"L%=2:" "ldi	r30, lo8(pm(L%=table))"	"\n\t"
			"ldi	r31, hi8(pm(L%=table))"	"\n\t"
			"add	r30, r22"		"\n\t"
			"adc	r31, __zero_reg__"	"\n\t"
			"asr	r22"			"\n\t"
			"asr	r22"			"\n\t"
			"st	X+, r22"		"\n\t"  // store new state
			"ld	r22, X+"		"\n\t"
			"ld	r23, X+"		"\n\t"
			"ld	r24, X+"		"\n\t"
			"ld	r25, X+"		"\n\t"
			"ijmp"				"\n\t"	// jumps to update_finishup()
			// TODO move this table to another static function,
			// so it doesn't get needlessly duplicated.  Easier
			// said than done, due to linker issues and inlining
		"L%=table:"				"\n\t"
			"rjmp	L%=end"			"\n\t"	// 0
			"rjmp	L%=plus1"		"\n\t"	// 1
			"rjmp	L%=minus1"		"\n\t"	// 2
			"rjmp	L%=plus2"		"\n\t"	// 3
			"rjmp	L%=minus1"		"\n\t"	// 4
			"rjmp	L%=end"			"\n\t"	// 5
			"rjmp	L%=minus2"		"\n\t"	// 6
			"rjmp	L%=plus1"		"\n\t"	// 7
			"rjmp	L%=plus1"		"\n\t"	// 8
			"rjmp	L%=minus2"		"\n\t"	// 9
			"rjmp	L%=end"			"\n\t"	// 10
			"rjmp	L%=minus1"		"\n\t"	// 11
			"rjmp	L%=plus2"		"\n\t"	// 12
			"rjmp	L%=minus1"		"\n\t"	// 13
			"rjmp	L%=plus1"		"\n\t"	// 14
			"rjmp	L%=end"			"\n\t"	// 15
		"L%=minus2:"				"\n\t"
			"subi	r22, 2"			"\n\t"
			"sbci	r23, 0"			"\n\t"
			"sbci	r24, 0"			"\n\t"
			"sbci	r25, 0"			"\n\t"
			"rjmp	L%=store"		"\n\t"
		"L%=minus1:"				"\n\t"
			"subi	r22, 1"			"\n\t"
			"sbci	r23, 0"			"\n\t"
			"sbci	r24, 0"			"\n\t"
			"sbci	r25, 0"			"\n\t"
			"rjmp	L%=store"		"\n\t"
		"L%=plus2:"				"\n\t"
			"subi	r22, 254"		"\n\t"
			"rjmp	L%=z"			"\n\t"
		"L%=plus1:"				"\n\t"
			"subi	r22, 255"		"\n\t"
		"L%=z:"	"sbci	r23, 255"		"\n\t"
			"sbci	r24, 255"		"\n\t"
			"sbci	r25, 255"		"\n\t"
		"L%=store:"				"\n\t"
			"st	-X, r25"		"\n\t"
			"st	-X, r24"		"\n\t"
			"st	-X, r23"		"\n\t"
			"st	-X, r22"		"\n\t"
		"L%=end:"				"\n"
		: : "x" (arg) : "r22", "r23", "r24", "r25", "r30", "r31");
#else
		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);
		uint8_t state = arg->state & 3;
		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);
//                           _______         _______       
//               Pin1 ______|       |_______|       |______ Pin1
// negative <---         _______         _______         __      --> positive
//               Pin2 __|       |_______|       |_______|   Pin2
				//	new	new	old	old
		//	pin2	pin1	pin2	pin1	Result
		//	----	----	----	----	------
		//0	0	0	0	0	no movement
		//1	0	0	0	1	+1 pin1 edge
		//2	0	0	1	0	-1 pin2 edge
		//3	0	0	1	1	+2  (assume pin1 edges only)
		//4	0	1	0	0	-1 pin1 edge
		//5	0	1	0	1	no movement
		//6	0	1	1	0	-2  (assume pin1 edges only)
		//7	0	1	1	1	+1 pin2 edge
		//8	1	0	0	0	+1 pin2 edge
		//9	1	0	0	1	-2  (assume pin1 edges only)
		//10	1	0	1	0	no movement
		//11	1	0	1	1	-1 pin1 edge
		//12	1	1	0	0	+2  (assume pin1 edges only)
		//13	1	1	0	1	-1 pin2 edge
		//14	1	1	1	0	+1 pin1 edge
		//15	1	1	1	1	no movement
		unsigned long microsTemp = micros();
		arg->timeOfLastTick = microsTemp;
		switch (state) {
			case 1: case 14: //+1 pin1 edge
				arg->uSBuffer[arg->bufferIndex] = microsTemp - arg->stepTime1;
				arg->bufferIndex++;
				arg->stepTime1 = microsTemp;
				if(arg->bufferIndex >= MAX_BUFFER_SIZE){
					arg->bufferIndex = 0;
				}
				arg->newTicks++;
				arg->position++;
				return;
			
			case 7: case 8: //+1 pin2 edge
				arg->uSBuffer[arg->bufferIndex] = microsTemp - arg->stepTime2;
				arg->bufferIndex++;
				arg->stepTime2 = microsTemp;
				if(arg->bufferIndex >= MAX_BUFFER_SIZE){
					arg->bufferIndex = 0;
				}
				arg->newTicks++;
				arg->position++;
				return;
			
			case 2: case 13: //-1 pin2 edge
				arg->uSBuffer[arg->bufferIndex] = -(microsTemp - arg->stepTime2);
				arg->bufferIndex++;
				arg->stepTime2 = microsTemp;
				if(arg->bufferIndex >= MAX_BUFFER_SIZE){
					arg->bufferIndex = 0;
				}
				arg->newTicks++;
				arg->position--;
				return;
			
			case 4: case 11: //-1 pin1 edge
				arg->uSBuffer[arg->bufferIndex] = -(microsTemp - arg->stepTime1);
				arg->bufferIndex++;
				arg->stepTime1 = microsTemp;
				if(arg->bufferIndex >= MAX_BUFFER_SIZE){
					arg->bufferIndex = 0;
				}
				arg->newTicks++;
				arg->position--;
				return;
			
			//+2's -2's to come later
			
			/*
			case 1: case 7: case 8: case 14:
                arg->previousRate = arg->rate;  // remember previous rate for calculating
                if (arg->position % 2 == 0) {  // if the previous position was even (0 to 1 step)
                    arg->rate1 = 0.5 / arg->stepTime; // the 0 to 1 step rate is set to rate1
                    if (arg->lastRateTimer == 0) { // if the 0 to 1 step was the previous one calculated
                        arg->rate2 = 0; // then the rate2 step was skipped due to a direction change, so set it to zero
                        arg->previousRate = 0; // previous rate is also set to zero.  there may be a better way but I have yet to think about it
                    }
                    arg->lastRateTimer = 0; // remember that rate1 was the last one calculated
                }
                else {  // if the previous position was odd (step -1 to 0)
                    arg->rate2 = 0.5 / arg->stepTime; // the -1 to 0 step rate is set to rate2
                    if (arg->lastRateTimer == 1) { // if the -1 to 0 step was the previous one calculated
                        arg->rate1 = 0;  // then rate1 step was skipped due to direction change, so it is set to zero
                        arg->previousRate = 0; // previous rate is also set to zero.  there may be a better way but I have yet to think about it
                    }
                    arg->lastRateTimer = 1; // remember that rate2 was the last one calculated
                }
                arg->rate = (arg->rate1 + arg->rate2);
                arg->accel = (arg->rate - arg->previousRate) / arg->stepTime;
                arg->stepTime = 0;
                arg->position++;
                return;
			case 2: case 4: case 11: case 13:
                arg->previousRate = arg->rate;
                if (arg->position % 2 != 0) {  // if the previous position was odd (1 to 0 step)
                    arg->rate1 = -0.5 / arg->stepTime; // the 1 to 0 step rate is set to rate1
                    if (arg->lastRateTimer == 0) {
                        arg->rate2 = 0;
                        arg->previousRate = 0;
                    }
                    arg->lastRateTimer = 0;
                }
                else {  // if the previous position was even
                    arg->rate2 = -0.5 / arg->stepTime;
                    if (arg->lastRateTimer == 1) {
                        arg->rate1 = 0;
                        arg->previousRate = 0;
                    }
                    arg->lastRateTimer = 1;
                }
                arg->rate = (arg->rate1 + arg->rate2);
                arg->accel = (arg->rate - arg->previousRate) / arg->stepTime;
                arg->stepTime = 0;
                arg->position--;
				return;
            case 3: case 12:
                arg->previousRate = arg->rate;
                if (arg->position % 2 == 0) {
                    arg->rate1 = 1 / arg->stepTime;
                    arg->rate2 = arg->rate1;
                    arg->rate = arg->rate1;
                }
                else {
                    arg->rate2 = 1 / arg->stepTime;
                    arg->rate1 = arg->rate2;
                    arg->rate = arg->rate2;
                }
                arg->accel = (arg->rate - arg->previousRate) / arg->stepTime;
                arg->stepTime = 0;
                arg->position += 2;
				return;
			case 6: case 9:
                arg->previousRate = arg->rate;
                if (arg->position % 2 != 0) {
                    arg->rate1 = -1 / arg->stepTime;
                    arg->rate2 = arg->rate1;
                    arg->rate = arg->rate1;
                }
                else {
                    arg->rate2 = -1 / arg->stepTime;
                    arg->rate1 = arg->rate2;
                    arg->rate = arg->rate2;
                }
                arg->accel = (arg->rate - arg->previousRate) / arg->stepTime;
                arg->stepTime = 0;
                arg->position -= 2;
				return;
				*/
		}
#endif
	}
/*
#if defined(__AVR__)
	// TODO: this must be a no inline function
	// even noinline does not seem to solve difficult
	// problems with this.  Oh well, it was only meant
	// to shrink code size - there's no performance
	// improvement in this, only code size reduction.
	__attribute__((noinline)) void update_finishup(void) {
		asm volatile (
			"ldi	r30, lo8(pm(Ltable))"	"\n\t"
			"ldi	r31, hi8(pm(Ltable))"	"\n\t"
		"Ltable:"				"\n\t"
			"rjmp	L%=end"			"\n\t"	// 0
			"rjmp	L%=plus1"		"\n\t"	// 1
			"rjmp	L%=minus1"		"\n\t"	// 2
			"rjmp	L%=plus2"		"\n\t"	// 3
			"rjmp	L%=minus1"		"\n\t"	// 4
			"rjmp	L%=end"			"\n\t"	// 5
			"rjmp	L%=minus2"		"\n\t"	// 6
			"rjmp	L%=plus1"		"\n\t"	// 7
			"rjmp	L%=plus1"		"\n\t"	// 8
			"rjmp	L%=minus2"		"\n\t"	// 9
			"rjmp	L%=end"			"\n\t"	// 10
			"rjmp	L%=minus1"		"\n\t"	// 11
			"rjmp	L%=plus2"		"\n\t"	// 12
			"rjmp	L%=minus1"		"\n\t"	// 13
			"rjmp	L%=plus1"		"\n\t"	// 14
			"rjmp	L%=end"			"\n\t"	// 15
		"L%=minus2:"				"\n\t"
			"subi	r22, 2"			"\n\t"
			"sbci	r23, 0"			"\n\t"
			"sbci	r24, 0"			"\n\t"
			"sbci	r25, 0"			"\n\t"
			"rjmp	L%=store"		"\n\t"
		"L%=minus1:"				"\n\t"
			"subi	r22, 1"			"\n\t"
			"sbci	r23, 0"			"\n\t"
			"sbci	r24, 0"			"\n\t"
			"sbci	r25, 0"			"\n\t"
			"rjmp	L%=store"		"\n\t"
		"L%=plus2:"				"\n\t"
			"subi	r22, 254"		"\n\t"
			"rjmp	L%=z"			"\n\t"
		"L%=plus1:"				"\n\t"
			"subi	r22, 255"		"\n\t"
		"L%=z:"	"sbci	r23, 255"		"\n\t"
			"sbci	r24, 255"		"\n\t"
			"sbci	r25, 255"		"\n\t"
		"L%=store:"				"\n\t"
			"st	-X, r25"		"\n\t"
			"st	-X, r24"		"\n\t"
			"st	-X, r23"		"\n\t"
			"st	-X, r22"		"\n\t"
		"L%=end:"				"\n"
		: : : "r22", "r23", "r24", "r25", "r30", "r31");
	}
#endif
*/


#ifdef ENCODER_USE_INTERRUPTS
	// this giant function is an unfortunate consequence of Arduino's
	// attachInterrupt function not supporting any way to pass a pointer
	// or other context to the attached function.
	static uint8_t attach_interrupt(uint8_t pin, Encoder_internal_state_t *state) {
		switch (pin) {
		#ifdef CORE_INT0_PIN
			case CORE_INT0_PIN:
				interruptArgs[0] = state;
				attachInterrupt(0, isr0, CHANGE);
				break;
		#endif
		#ifdef CORE_INT1_PIN
			case CORE_INT1_PIN:
				interruptArgs[1] = state;
				attachInterrupt(1, isr1, CHANGE);
				break;
		#endif
		#ifdef CORE_INT2_PIN
			case CORE_INT2_PIN:
				interruptArgs[2] = state;
				attachInterrupt(2, isr2, CHANGE);
				break;
		#endif
		#ifdef CORE_INT3_PIN
			case CORE_INT3_PIN:
				interruptArgs[3] = state;
				attachInterrupt(3, isr3, CHANGE);
				break;
		#endif
		#ifdef CORE_INT4_PIN
			case CORE_INT4_PIN:
				interruptArgs[4] = state;
				attachInterrupt(4, isr4, CHANGE);
				break;
		#endif
		#ifdef CORE_INT5_PIN
			case CORE_INT5_PIN:
				interruptArgs[5] = state;
				attachInterrupt(5, isr5, CHANGE);
				break;
		#endif
		#ifdef CORE_INT6_PIN
			case CORE_INT6_PIN:
				interruptArgs[6] = state;
				attachInterrupt(6, isr6, CHANGE);
				break;
		#endif
		#ifdef CORE_INT7_PIN
			case CORE_INT7_PIN:
				interruptArgs[7] = state;
				attachInterrupt(7, isr7, CHANGE);
				break;
		#endif
		#ifdef CORE_INT8_PIN
			case CORE_INT8_PIN:
				interruptArgs[8] = state;
				attachInterrupt(8, isr8, CHANGE);
				break;
		#endif
		#ifdef CORE_INT9_PIN
			case CORE_INT9_PIN:
				interruptArgs[9] = state;
				attachInterrupt(9, isr9, CHANGE);
				break;
		#endif
		#ifdef CORE_INT10_PIN
			case CORE_INT10_PIN:
				interruptArgs[10] = state;
				attachInterrupt(10, isr10, CHANGE);
				break;
		#endif
		#ifdef CORE_INT11_PIN
			case CORE_INT11_PIN:
				interruptArgs[11] = state;
				attachInterrupt(11, isr11, CHANGE);
				break;
		#endif
		#ifdef CORE_INT12_PIN
			case CORE_INT12_PIN:
				interruptArgs[12] = state;
				attachInterrupt(12, isr12, CHANGE);
				break;
		#endif
		#ifdef CORE_INT13_PIN
			case CORE_INT13_PIN:
				interruptArgs[13] = state;
				attachInterrupt(13, isr13, CHANGE);
				break;
		#endif
		#ifdef CORE_INT14_PIN
			case CORE_INT14_PIN:
				interruptArgs[14] = state;
				attachInterrupt(14, isr14, CHANGE);
				break;
		#endif
		#ifdef CORE_INT15_PIN
			case CORE_INT15_PIN:
				interruptArgs[15] = state;
				attachInterrupt(15, isr15, CHANGE);
				break;
		#endif
		#ifdef CORE_INT16_PIN
			case CORE_INT16_PIN:
				interruptArgs[16] = state;
				attachInterrupt(16, isr16, CHANGE);
				break;
		#endif
		#ifdef CORE_INT17_PIN
			case CORE_INT17_PIN:
				interruptArgs[17] = state;
				attachInterrupt(17, isr17, CHANGE);
				break;
		#endif
		#ifdef CORE_INT18_PIN
			case CORE_INT18_PIN:
				interruptArgs[18] = state;
				attachInterrupt(18, isr18, CHANGE);
				break;
		#endif
		#ifdef CORE_INT19_PIN
			case CORE_INT19_PIN:
				interruptArgs[19] = state;
				attachInterrupt(19, isr19, CHANGE);
				break;
		#endif
		#ifdef CORE_INT20_PIN
			case CORE_INT20_PIN:
				interruptArgs[20] = state;
				attachInterrupt(20, isr20, CHANGE);
				break;
		#endif
		#ifdef CORE_INT21_PIN
			case CORE_INT21_PIN:
				interruptArgs[21] = state;
				attachInterrupt(21, isr21, CHANGE);
				break;
		#endif
		#ifdef CORE_INT22_PIN
			case CORE_INT22_PIN:
				interruptArgs[22] = state;
				attachInterrupt(22, isr22, CHANGE);
				break;
		#endif
		#ifdef CORE_INT23_PIN
			case CORE_INT23_PIN:
				interruptArgs[23] = state;
				attachInterrupt(23, isr23, CHANGE);
				break;
		#endif
		#ifdef CORE_INT24_PIN
			case CORE_INT24_PIN:
				interruptArgs[24] = state;
				attachInterrupt(24, isr24, CHANGE);
				break;
		#endif
		#ifdef CORE_INT25_PIN
			case CORE_INT25_PIN:
				interruptArgs[25] = state;
				attachInterrupt(25, isr25, CHANGE);
				break;
		#endif
		#ifdef CORE_INT26_PIN
			case CORE_INT26_PIN:
				interruptArgs[26] = state;
				attachInterrupt(26, isr26, CHANGE);
				break;
		#endif
		#ifdef CORE_INT27_PIN
			case CORE_INT27_PIN:
				interruptArgs[27] = state;
				attachInterrupt(27, isr27, CHANGE);
				break;
		#endif
		#ifdef CORE_INT28_PIN
			case CORE_INT28_PIN:
				interruptArgs[28] = state;
				attachInterrupt(28, isr28, CHANGE);
				break;
		#endif
		#ifdef CORE_INT29_PIN
			case CORE_INT29_PIN:
				interruptArgs[29] = state;
				attachInterrupt(29, isr29, CHANGE);
				break;
		#endif

		#ifdef CORE_INT30_PIN
			case CORE_INT30_PIN:
				interruptArgs[30] = state;
				attachInterrupt(30, isr30, CHANGE);
				break;
		#endif
		#ifdef CORE_INT31_PIN
			case CORE_INT31_PIN:
				interruptArgs[31] = state;
				attachInterrupt(31, isr31, CHANGE);
				break;
		#endif
		#ifdef CORE_INT32_PIN
			case CORE_INT32_PIN:
				interruptArgs[32] = state;
				attachInterrupt(32, isr32, CHANGE);
				break;
		#endif
		#ifdef CORE_INT33_PIN
			case CORE_INT33_PIN:
				interruptArgs[33] = state;
				attachInterrupt(33, isr33, CHANGE);
				break;
		#endif
		#ifdef CORE_INT34_PIN
			case CORE_INT34_PIN:
				interruptArgs[34] = state;
				attachInterrupt(34, isr34, CHANGE);
				break;
		#endif
		#ifdef CORE_INT35_PIN
			case CORE_INT35_PIN:
				interruptArgs[35] = state;
				attachInterrupt(35, isr35, CHANGE);
				break;
		#endif
		#ifdef CORE_INT36_PIN
			case CORE_INT36_PIN:
				interruptArgs[36] = state;
				attachInterrupt(36, isr36, CHANGE);
				break;
		#endif
		#ifdef CORE_INT37_PIN
			case CORE_INT37_PIN:
				interruptArgs[37] = state;
				attachInterrupt(37, isr37, CHANGE);
				break;
		#endif
		#ifdef CORE_INT38_PIN
			case CORE_INT38_PIN:
				interruptArgs[38] = state;
				attachInterrupt(38, isr38, CHANGE);
				break;
		#endif
		#ifdef CORE_INT39_PIN
			case CORE_INT39_PIN:
				interruptArgs[39] = state;
				attachInterrupt(39, isr39, CHANGE);
				break;
		#endif
		#ifdef CORE_INT40_PIN
			case CORE_INT40_PIN:
				interruptArgs[40] = state;
				attachInterrupt(40, isr40, CHANGE);
				break;
		#endif
		#ifdef CORE_INT41_PIN
			case CORE_INT41_PIN:
				interruptArgs[41] = state;
				attachInterrupt(41, isr41, CHANGE);
				break;
		#endif
		#ifdef CORE_INT42_PIN
			case CORE_INT42_PIN:
				interruptArgs[42] = state;
				attachInterrupt(42, isr42, CHANGE);
				break;
		#endif
		#ifdef CORE_INT43_PIN
			case CORE_INT43_PIN:
				interruptArgs[43] = state;
				attachInterrupt(43, isr43, CHANGE);
				break;
		#endif
		#ifdef CORE_INT44_PIN
			case CORE_INT44_PIN:
				interruptArgs[44] = state;
				attachInterrupt(44, isr44, CHANGE);
				break;
		#endif
		#ifdef CORE_INT45_PIN
			case CORE_INT45_PIN:
				interruptArgs[45] = state;
				attachInterrupt(45, isr45, CHANGE);
				break;
		#endif
		#ifdef CORE_INT46_PIN
			case CORE_INT46_PIN:
				interruptArgs[46] = state;
				attachInterrupt(46, isr46, CHANGE);
				break;
		#endif
		#ifdef CORE_INT47_PIN
			case CORE_INT47_PIN:
				interruptArgs[47] = state;
				attachInterrupt(47, isr47, CHANGE);
				break;
		#endif
		#ifdef CORE_INT48_PIN
			case CORE_INT48_PIN:
				interruptArgs[48] = state;
				attachInterrupt(48, isr48, CHANGE);
				break;
		#endif
		#ifdef CORE_INT49_PIN
			case CORE_INT49_PIN:
				interruptArgs[49] = state;
				attachInterrupt(49, isr49, CHANGE);
				break;
		#endif
		#ifdef CORE_INT50_PIN
			case CORE_INT50_PIN:
				interruptArgs[50] = state;
				attachInterrupt(50, isr50, CHANGE);
				break;
		#endif
		#ifdef CORE_INT51_PIN
			case CORE_INT51_PIN:
				interruptArgs[51] = state;
				attachInterrupt(51, isr51, CHANGE);
				break;
		#endif
		#ifdef CORE_INT52_PIN
			case CORE_INT52_PIN:
				interruptArgs[52] = state;
				attachInterrupt(52, isr52, CHANGE);
				break;
		#endif
		#ifdef CORE_INT53_PIN
			case CORE_INT53_PIN:
				interruptArgs[53] = state;
				attachInterrupt(53, isr53, CHANGE);
				break;
		#endif
		#ifdef CORE_INT54_PIN
			case CORE_INT54_PIN:
				interruptArgs[54] = state;
				attachInterrupt(54, isr54, CHANGE);
				break;
		#endif
		#ifdef CORE_INT55_PIN
			case CORE_INT55_PIN:
				interruptArgs[55] = state;
				attachInterrupt(55, isr55, CHANGE);
				break;
		#endif
		#ifdef CORE_INT56_PIN
			case CORE_INT56_PIN:
				interruptArgs[56] = state;
				attachInterrupt(56, isr56, CHANGE);
				break;
		#endif
		#ifdef CORE_INT57_PIN
			case CORE_INT57_PIN:
				interruptArgs[57] = state;
				attachInterrupt(57, isr57, CHANGE);
				break;
		#endif
		#ifdef CORE_INT58_PIN
			case CORE_INT58_PIN:
				interruptArgs[58] = state;
				attachInterrupt(58, isr58, CHANGE);
				break;
		#endif
		#ifdef CORE_INT59_PIN
			case CORE_INT59_PIN:
				interruptArgs[59] = state;
				attachInterrupt(59, isr59, CHANGE);
				break;
		#endif
			default:
				return 0;
		}
		return 1;
	}
#endif // ENCODER_USE_INTERRUPTS


#if defined(ENCODER_USE_INTERRUPTS) && !defined(ENCODER_OPTIMIZE_INTERRUPTS)
	#ifdef CORE_INT0_PIN
	static void isr0(void) { update(interruptArgs[0]); }
	#endif
	#ifdef CORE_INT1_PIN
	static void isr1(void) { update(interruptArgs[1]); }
	#endif
	#ifdef CORE_INT2_PIN
	static void isr2(void) { update(interruptArgs[2]); }
	#endif
	#ifdef CORE_INT3_PIN
	static void isr3(void) { update(interruptArgs[3]); }
	#endif
	#ifdef CORE_INT4_PIN
	static void isr4(void) { update(interruptArgs[4]); }
	#endif
	#ifdef CORE_INT5_PIN
	static void isr5(void) { update(interruptArgs[5]); }
	#endif
	#ifdef CORE_INT6_PIN
	static void isr6(void) { update(interruptArgs[6]); }
	#endif
	#ifdef CORE_INT7_PIN
	static void isr7(void) { update(interruptArgs[7]); }
	#endif
	#ifdef CORE_INT8_PIN
	static void isr8(void) { update(interruptArgs[8]); }
	#endif
	#ifdef CORE_INT9_PIN
	static void isr9(void) { update(interruptArgs[9]); }
	#endif
	#ifdef CORE_INT10_PIN
	static void isr10(void) { update(interruptArgs[10]); }
	#endif
	#ifdef CORE_INT11_PIN
	static void isr11(void) { update(interruptArgs[11]); }
	#endif
	#ifdef CORE_INT12_PIN
	static void isr12(void) { update(interruptArgs[12]); }
	#endif
	#ifdef CORE_INT13_PIN
	static void isr13(void) { update(interruptArgs[13]); }
	#endif
	#ifdef CORE_INT14_PIN
	static void isr14(void) { update(interruptArgs[14]); }
	#endif
	#ifdef CORE_INT15_PIN
	static void isr15(void) { update(interruptArgs[15]); }
	#endif
	#ifdef CORE_INT16_PIN
	static void isr16(void) { update(interruptArgs[16]); }
	#endif
	#ifdef CORE_INT17_PIN
	static void isr17(void) { update(interruptArgs[17]); }
	#endif
	#ifdef CORE_INT18_PIN
	static void isr18(void) { update(interruptArgs[18]); }
	#endif
	#ifdef CORE_INT19_PIN
	static void isr19(void) { update(interruptArgs[19]); }
	#endif
	#ifdef CORE_INT20_PIN
	static void isr20(void) { update(interruptArgs[20]); }
	#endif
	#ifdef CORE_INT21_PIN
	static void isr21(void) { update(interruptArgs[21]); }
	#endif
	#ifdef CORE_INT22_PIN
	static void isr22(void) { update(interruptArgs[22]); }
	#endif
	#ifdef CORE_INT23_PIN
	static void isr23(void) { update(interruptArgs[23]); }
	#endif
	#ifdef CORE_INT24_PIN
	static void isr24(void) { update(interruptArgs[24]); }
	#endif
	#ifdef CORE_INT25_PIN
	static void isr25(void) { update(interruptArgs[25]); }
	#endif
	#ifdef CORE_INT26_PIN
	static void isr26(void) { update(interruptArgs[26]); }
	#endif
	#ifdef CORE_INT27_PIN
	static void isr27(void) { update(interruptArgs[27]); }
	#endif
	#ifdef CORE_INT28_PIN
	static void isr28(void) { update(interruptArgs[28]); }
	#endif
	#ifdef CORE_INT29_PIN
	static void isr29(void) { update(interruptArgs[29]); }
	#endif
	#ifdef CORE_INT30_PIN
	static void isr30(void) { update(interruptArgs[30]); }
	#endif
	#ifdef CORE_INT31_PIN
	static void isr31(void) { update(interruptArgs[31]); }
	#endif
	#ifdef CORE_INT32_PIN
	static void isr32(void) { update(interruptArgs[32]); }
	#endif
	#ifdef CORE_INT33_PIN
	static void isr33(void) { update(interruptArgs[33]); }
	#endif
	#ifdef CORE_INT34_PIN
	static void isr34(void) { update(interruptArgs[34]); }
	#endif
	#ifdef CORE_INT35_PIN
	static void isr35(void) { update(interruptArgs[35]); }
	#endif
	#ifdef CORE_INT36_PIN
	static void isr36(void) { update(interruptArgs[36]); }
	#endif
	#ifdef CORE_INT37_PIN
	static void isr37(void) { update(interruptArgs[37]); }
	#endif
	#ifdef CORE_INT38_PIN
	static void isr38(void) { update(interruptArgs[38]); }
	#endif
	#ifdef CORE_INT39_PIN
	static void isr39(void) { update(interruptArgs[39]); }
	#endif
	#ifdef CORE_INT40_PIN
	static void isr40(void) { update(interruptArgs[40]); }
	#endif
	#ifdef CORE_INT41_PIN
	static void isr41(void) { update(interruptArgs[41]); }
	#endif
	#ifdef CORE_INT42_PIN
	static void isr42(void) { update(interruptArgs[42]); }
	#endif
	#ifdef CORE_INT43_PIN
	static void isr43(void) { update(interruptArgs[43]); }
	#endif
	#ifdef CORE_INT44_PIN
	static void isr44(void) { update(interruptArgs[44]); }
	#endif
	#ifdef CORE_INT45_PIN
	static void isr45(void) { update(interruptArgs[45]); }
	#endif
	#ifdef CORE_INT46_PIN
	static void isr46(void) { update(interruptArgs[46]); }
	#endif
	#ifdef CORE_INT47_PIN
	static void isr47(void) { update(interruptArgs[47]); }
	#endif
	#ifdef CORE_INT48_PIN
	static void isr48(void) { update(interruptArgs[48]); }
	#endif
	#ifdef CORE_INT49_PIN
	static void isr49(void) { update(interruptArgs[49]); }
	#endif
	#ifdef CORE_INT50_PIN
	static void isr50(void) { update(interruptArgs[50]); }
	#endif
	#ifdef CORE_INT51_PIN
	static void isr51(void) { update(interruptArgs[51]); }
	#endif
	#ifdef CORE_INT52_PIN
	static void isr52(void) { update(interruptArgs[52]); }
	#endif
	#ifdef CORE_INT53_PIN
	static void isr53(void) { update(interruptArgs[53]); }
	#endif
	#ifdef CORE_INT54_PIN
	static void isr54(void) { update(interruptArgs[54]); }
	#endif
	#ifdef CORE_INT55_PIN
	static void isr55(void) { update(interruptArgs[55]); }
	#endif
	#ifdef CORE_INT56_PIN
	static void isr56(void) { update(interruptArgs[56]); }
	#endif
	#ifdef CORE_INT57_PIN
	static void isr57(void) { update(interruptArgs[57]); }
	#endif
	#ifdef CORE_INT58_PIN
	static void isr58(void) { update(interruptArgs[58]); }
	#endif
	#ifdef CORE_INT59_PIN
	static void isr59(void) { update(interruptArgs[59]); }
	#endif
#endif
};

#if defined(ENCODER_USE_INTERRUPTS) && defined(ENCODER_OPTIMIZE_INTERRUPTS)
#if defined(__AVR__)
#if defined(INT0_vect) && CORE_NUM_INTERRUPT > 0
ISR(INT0_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(0)]); }
#endif
#if defined(INT1_vect) && CORE_NUM_INTERRUPT > 1
ISR(INT1_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(1)]); }
#endif
#if defined(INT2_vect) && CORE_NUM_INTERRUPT > 2
ISR(INT2_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(2)]); }
#endif
#if defined(INT3_vect) && CORE_NUM_INTERRUPT > 3
ISR(INT3_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(3)]); }
#endif
#if defined(INT4_vect) && CORE_NUM_INTERRUPT > 4
ISR(INT4_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(4)]); }
#endif
#if defined(INT5_vect) && CORE_NUM_INTERRUPT > 5
ISR(INT5_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(5)]); }
#endif
#if defined(INT6_vect) && CORE_NUM_INTERRUPT > 6
ISR(INT6_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(6)]); }
#endif
#if defined(INT7_vect) && CORE_NUM_INTERRUPT > 7
ISR(INT7_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(7)]); }
#endif
#endif // AVR
#endif // ENCODER_OPTIMIZE_INTERRUPTS


#endif
