.def delay_counter = r16
.org 0x0000
	rjmp init

init:
	SBI DDRB, 1
	rjmp main

main:
	SBI PINB, 1 ; toggle the state of PB1
	RCALL delay
	rjmp main

delay:
	LDI delay_counter, 255
	delay_loop:
		DEC delay_counter
		BRNE delay_loop
		RET