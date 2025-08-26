.def delay_counter = r16
.org 0x0000
rjmp init

init:
	sbi DDRB, 1
	rjmp main

main:
	sbi PINB, 1 ; toggle the state of PB1
	rcall delay
	rjmp main

delay:
	ldi delay_counter, 255
	delay_loop:
		dec delay_counter
		brne delay_loop
		ret