.def temp1 = r16
.def temp2 = r17
.def temp3 = r18

.org 0x0000
rjmp init

init:
	sbi DDRC, 2		; Set bit 2 of DDRC (PC2) to 1 -> as output
	rjmp main

main:
	cbi PORTC, 2
	rcall delay
	sbi PORTC, 2
	rcall delay
	rjmp main

delay:
	ldi temp1, 255
	delay_outer:
		ldi temp2, 255
		delay_middle:
			ldi temp3, 255
			delay_inner:
				dec temp3
				brne delay_inner
			dec temp2
			brne delay_middle
		dec temp1
		brne delay_outer
	ret