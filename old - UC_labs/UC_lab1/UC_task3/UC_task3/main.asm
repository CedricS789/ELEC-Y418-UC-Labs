; UC_task2.asm
;
; Created: 19/07/2025 13:13:27
; Author : Cedric
;

.include "m328pdef.inc"

.def counter1 = r18		; Outer loop counter
.def counter2 = r19		; Middle loop counter
.def counter3 = r20		; Inner loop counter

.org 0x0000
	rjmp init


init:
		sbi DDRC, 2		; Set pin PC2 as an output (writing 1 into bit 2 of the register DDRC)
		rjmp main


main:
		cbi PORTC, 2	; Turn LED on
		rcall delay		; Wait
		sbi PORTC, 2	; Turn LED off
		rcall delay		; Wait
		rjmp main		; Repeat indefinetely


; Delay Subroutine
; Total cycles : 150 * 150 * 150 * 1 (nop) = 3 375 000 cycles
delay:
		ldi counter1, 150
		loop1: 
			ldi counter2, 150
			loop2: 
				ldi counter3, 150
				loop3: 
					dec counter3
					brne loop3
				dec counter2
				brne loop2
			dec counter1
			brne loop1
		ret
