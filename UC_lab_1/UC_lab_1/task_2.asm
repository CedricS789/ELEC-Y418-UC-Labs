.org 0x0000
rjmp init

init:
	; Configure Pin PB0 as input with pull-up
	CBI DDRB, 0			; Set bit 0 of DDRB to 0 -> PB0 is input
	SBI PORTB, 0		; Set bit 0 of PORTB to 1 -> Enable pull-up on PB0

	; Cofigure LEB Pin PC2 as input
	SBI DDRC, 2			; Set bit 2 of DDRC to 1 -> PC2 is an output
	SBI PORTC, 2		; Initially set PC2 to HIGH to turn the LED off
	rjmp main


main:
	IN r16, PINB		; Read the state of all port B pins and store them into the general purpose register r16
	BST r16, 0			; Bit Store into T flag: store bit 0 of r16 (PB0) into T-flag
	BRTS SwitchUp		; Branch if T-flag is set to 1

	; If switch down
	SBI PORTC, 2
	rjmp main

SwitchUp:
	CBI PORTC, 2
	rjmp main