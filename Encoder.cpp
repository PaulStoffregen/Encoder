
#include "Encoder.h"

// Most the code is in the header file, to provide the user
// configure options with #define (before they include it), and
// to facilitate some crafty optimizations!

Encoder_internal_state_t * Encoder::interruptArgs[];

void ENCODER_ISR_ATTR Encoder::update(Encoder_internal_state_t *arg) ENCODER_ISR_ATTR {
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
	switch (state) {
		case 1: case 7: case 8: case 14:
			arg->position++;
			return;
		case 2: case 4: case 11: case 13:
			arg->position--;
			return;
		case 3: case 12:
			arg->position += 2;
			return;
		case 6: case 9:
			arg->position -= 2;
			return;
	}
#endif
}

#if defined(ENCODER_USE_INTERRUPTS) && !defined(ENCODER_OPTIMIZE_INTERRUPTS)
	#ifdef CORE_INT0_PIN
	void Encoder::isr0(void) ENCODER_ISR_ATTR { update(interruptArgs[0]); }
	#endif
	#ifdef CORE_INT1_PIN
	void Encoder::isr1(void) ENCODER_ISR_ATTR { update(interruptArgs[1]); }
	#endif
	#ifdef CORE_INT2_PIN
	void Encoder::isr2(void) ENCODER_ISR_ATTR { update(interruptArgs[2]); }
	#endif
	#ifdef CORE_INT3_PIN
	void Encoder::isr3(void) ENCODER_ISR_ATTR { update(interruptArgs[3]); }
	#endif
	#ifdef CORE_INT4_PIN
	void Encoder::isr4(void) ENCODER_ISR_ATTR { update(interruptArgs[4]); }
	#endif
	#ifdef CORE_INT5_PIN
	void Encoder::isr5(void) ENCODER_ISR_ATTR { update(interruptArgs[5]); }
	#endif
	#ifdef CORE_INT6_PIN
	void Encoder::isr6(void) ENCODER_ISR_ATTR { update(interruptArgs[6]); }
	#endif
	#ifdef CORE_INT7_PIN
	void Encoder::isr7(void) ENCODER_ISR_ATTR { update(interruptArgs[7]); }
	#endif
	#ifdef CORE_INT8_PIN
	void Encoder::isr8(void) ENCODER_ISR_ATTR { update(interruptArgs[8]); }
	#endif
	#ifdef CORE_INT9_PIN
	void Encoder::isr9(void) ENCODER_ISR_ATTR { update(interruptArgs[9]); }
	#endif
	#ifdef CORE_INT10_PIN
	void Encoder::isr10(void) ENCODER_ISR_ATTR { update(interruptArgs[10]); }
	#endif
	#ifdef CORE_INT11_PIN
	void Encoder::isr11(void) ENCODER_ISR_ATTR { update(interruptArgs[11]); }
	#endif
	#ifdef CORE_INT12_PIN
	void Encoder::isr12(void) ENCODER_ISR_ATTR { update(interruptArgs[12]); }
	#endif
	#ifdef CORE_INT13_PIN
	void Encoder::isr13(void) ENCODER_ISR_ATTR { update(interruptArgs[13]); }
	#endif
	#ifdef CORE_INT14_PIN
	void Encoder::isr14(void) ENCODER_ISR_ATTR { update(interruptArgs[14]); }
	#endif
	#ifdef CORE_INT15_PIN
	void Encoder::isr15(void) ENCODER_ISR_ATTR { update(interruptArgs[15]); }
	#endif
	#ifdef CORE_INT16_PIN
	void Encoder::isr16(void) ENCODER_ISR_ATTR { update(interruptArgs[16]); }
	#endif
	#ifdef CORE_INT17_PIN
	void Encoder::isr17(void) ENCODER_ISR_ATTR { update(interruptArgs[17]); }
	#endif
	#ifdef CORE_INT18_PIN
	void Encoder::isr18(void) ENCODER_ISR_ATTR { update(interruptArgs[18]); }
	#endif
	#ifdef CORE_INT19_PIN
	void Encoder::isr19(void) ENCODER_ISR_ATTR { update(interruptArgs[19]); }
	#endif
	#ifdef CORE_INT20_PIN
	void Encoder::isr20(void) ENCODER_ISR_ATTR { update(interruptArgs[20]); }
	#endif
	#ifdef CORE_INT21_PIN
	void Encoder::isr21(void) ENCODER_ISR_ATTR { update(interruptArgs[21]); }
	#endif
	#ifdef CORE_INT22_PIN
	void Encoder::isr22(void) ENCODER_ISR_ATTR { update(interruptArgs[22]); }
	#endif
	#ifdef CORE_INT23_PIN
	void Encoder::isr23(void) ENCODER_ISR_ATTR { update(interruptArgs[23]); }
	#endif
	#ifdef CORE_INT24_PIN
	void Encoder::isr24(void) ENCODER_ISR_ATTR { update(interruptArgs[24]); }
	#endif
	#ifdef CORE_INT25_PIN
	void Encoder::isr25(void) ENCODER_ISR_ATTR { update(interruptArgs[25]); }
	#endif
	#ifdef CORE_INT26_PIN
	void Encoder::isr26(void) ENCODER_ISR_ATTR { update(interruptArgs[26]); }
	#endif
	#ifdef CORE_INT27_PIN
	void Encoder::isr27(void) ENCODER_ISR_ATTR { update(interruptArgs[27]); }
	#endif
	#ifdef CORE_INT28_PIN
	void Encoder::isr28(void) ENCODER_ISR_ATTR { update(interruptArgs[28]); }
	#endif
	#ifdef CORE_INT29_PIN
	void Encoder::isr29(void) ENCODER_ISR_ATTR { update(interruptArgs[29]); }
	#endif
	#ifdef CORE_INT30_PIN
	void Encoder::isr30(void) ENCODER_ISR_ATTR { update(interruptArgs[30]); }
	#endif
	#ifdef CORE_INT31_PIN
	void Encoder::isr31(void) ENCODER_ISR_ATTR { update(interruptArgs[31]); }
	#endif
	#ifdef CORE_INT32_PIN
	void Encoder::isr32(void) ENCODER_ISR_ATTR { update(interruptArgs[32]); }
	#endif
	#ifdef CORE_INT33_PIN
	void Encoder::isr33(void) ENCODER_ISR_ATTR { update(interruptArgs[33]); }
	#endif
	#ifdef CORE_INT34_PIN
	void Encoder::isr34(void) ENCODER_ISR_ATTR { update(interruptArgs[34]); }
	#endif
	#ifdef CORE_INT35_PIN
	void Encoder::isr35(void) ENCODER_ISR_ATTR { update(interruptArgs[35]); }
	#endif
	#ifdef CORE_INT36_PIN
	void Encoder::isr36(void) ENCODER_ISR_ATTR { update(interruptArgs[36]); }
	#endif
	#ifdef CORE_INT37_PIN
	void Encoder::isr37(void) ENCODER_ISR_ATTR { update(interruptArgs[37]); }
	#endif
	#ifdef CORE_INT38_PIN
	void Encoder::isr38(void) ENCODER_ISR_ATTR { update(interruptArgs[38]); }
	#endif
	#ifdef CORE_INT39_PIN
	void Encoder::isr39(void) ENCODER_ISR_ATTR { update(interruptArgs[39]); }
	#endif
	#ifdef CORE_INT40_PIN
	void Encoder::isr40(void) ENCODER_ISR_ATTR { update(interruptArgs[40]); }
	#endif
	#ifdef CORE_INT41_PIN
	void Encoder::isr41(void) ENCODER_ISR_ATTR { update(interruptArgs[41]); }
	#endif
	#ifdef CORE_INT42_PIN
	void Encoder::isr42(void) ENCODER_ISR_ATTR { update(interruptArgs[42]); }
	#endif
	#ifdef CORE_INT43_PIN
	void Encoder::isr43(void) ENCODER_ISR_ATTR { update(interruptArgs[43]); }
	#endif
	#ifdef CORE_INT44_PIN
	void Encoder::isr44(void) ENCODER_ISR_ATTR { update(interruptArgs[44]); }
	#endif
	#ifdef CORE_INT45_PIN
	void Encoder::isr45(void) ENCODER_ISR_ATTR { update(interruptArgs[45]); }
	#endif
	#ifdef CORE_INT46_PIN
	void Encoder::isr46(void) ENCODER_ISR_ATTR { update(interruptArgs[46]); }
	#endif
	#ifdef CORE_INT47_PIN
	void Encoder::isr47(void) ENCODER_ISR_ATTR { update(interruptArgs[47]); }
	#endif
	#ifdef CORE_INT48_PIN
	void Encoder::isr48(void) ENCODER_ISR_ATTR { update(interruptArgs[48]); }
	#endif
	#ifdef CORE_INT49_PIN
	void Encoder::isr49(void) ENCODER_ISR_ATTR { update(interruptArgs[49]); }
	#endif
	#ifdef CORE_INT50_PIN
	void Encoder::isr50(void) ENCODER_ISR_ATTR { update(interruptArgs[50]); }
	#endif
	#ifdef CORE_INT51_PIN
	void Encoder::isr51(void) ENCODER_ISR_ATTR { update(interruptArgs[51]); }
	#endif
	#ifdef CORE_INT52_PIN
	void Encoder::isr52(void) ENCODER_ISR_ATTR { update(interruptArgs[52]); }
	#endif
	#ifdef CORE_INT53_PIN
	void Encoder::isr53(void) ENCODER_ISR_ATTR { update(interruptArgs[53]); }
	#endif
	#ifdef CORE_INT54_PIN
	void Encoder::isr54(void) ENCODER_ISR_ATTR { update(interruptArgs[54]); }
	#endif
	#ifdef CORE_INT55_PIN
	void Encoder::isr55(void) ENCODER_ISR_ATTR { update(interruptArgs[55]); }
	#endif
	#ifdef CORE_INT56_PIN
	void Encoder::isr56(void) ENCODER_ISR_ATTR { update(interruptArgs[56]); }
	#endif
	#ifdef CORE_INT57_PIN
	void Encoder::isr57(void) ENCODER_ISR_ATTR { update(interruptArgs[57]); }
	#endif
	#ifdef CORE_INT58_PIN
	void Encoder::isr58(void) ENCODER_ISR_ATTR { update(interruptArgs[58]); }
	#endif
	#ifdef CORE_INT59_PIN
	void Encoder::isr59(void) ENCODER_ISR_ATTR { update(interruptArgs[59]); }
	#endif
#endif
