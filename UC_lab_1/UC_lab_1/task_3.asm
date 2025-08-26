.def temp1 = r16
.def temp2 = r17
.def temp3 = r18

.org 0x0000
rjmp init

init:
	SBI DDRC, 2		; Set bit 2 of DDRC (PC2) to 1 -> as output
	rjmp main

main:
	CBI PORTC, 2
	RCALL delay
	SBI PORTC, 2
	RCALL delay
	rjmp main

delay:
	LDI temp1, 255
	delay_outer:
		LDI temp2, 255
		delay_middle:
			LDI temp3, 255
			delay_inner:
				DEC temp3
				BRNE delay_inner
			DEC temp2
			BRNE delay_middle
		DEC temp1
		BRNE delay_outer
	RET