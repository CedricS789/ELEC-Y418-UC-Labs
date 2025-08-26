.include "m328pdef.inc"

.org 0x0000
	rjmp init

init:
	sbi DDRB, 5		;setting PB5 as an output
	cbi PORTB, 5	;setting all the value to 0
	sbi DDRB, 4		;setting PB4 as an output
	cbi PORTB, 4	;setting all the value to 0
	sbi DDRB, 3		;setting PB3 as an output
	cbi PORTB, 3	;setting all the value to 0
start:	
	
	ldi R16,81
	ldi R17,9

loop:
	DEC R16
	
	SBI PORTB,3
	CPI R16,1
	BREQ set_1
	CBI PORTB,5
	SBI PORTB,5
	CBI PORTB,5
	rjmp loop

set_1:
	CBI PORTB,3

	CBI PORTB,5
	SBI PORTB,5
	CBI PORTB,5
	CPI R17,0
	BREQ latch

loop2:
	DEC R17

	SBI PORTB,3
	CPI R17,2
	BREQ set_1
	CBI PORTB,5
	SBI PORTB,5
	CBI PORTB,5
	rjmp loop2

 latch:
	CBI PORTB,4
	SBI PORTB,4
	CBI PORTB,4
	rjmp start
