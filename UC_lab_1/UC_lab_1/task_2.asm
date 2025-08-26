.org 0x0000
rjmp init

init:
	; Configure Pin PB0 as input with pull-up
	cbi DDRB, 0			; Set bit 0 of DDRB to 0 -> PB0 is input
	sbi PORTB, 0		; Set bit 0 of PORTB to 1 -> Enable pull-up on PB0

	; Cofigure LEB Pin PC2 as input
	sbi DDRC, 2			; Set bit 2 of DDRC to 1 -> PC2 is an output
	sbi PORTC, 2		; Initially set PC2 to HIGH to turn the LED off
	rjmp main


main:
	in r16, PINB		; Read the state of all port B pins and store them into the general purpose register r16
	bst r16, 0			; Bit Store into T flag: store bit 0 of r16 (PB0) into T-flag
	brts SwitchUp		; Branch if T-flag is set to 1

	; If switch down
	sbi PORTC, 2
	rjmp main

SwitchUp:
	cbi PORTC, 2
	rjmp main