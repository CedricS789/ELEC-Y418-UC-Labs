.org 0x0000
	rjmp init

init:
	; Configure Pin PB2 as input with pull-up
	CBI DDRB, 2			; Set bit 2 of DDRB to 0 -> PB2 is input
	SBI PORTB, 2		; Set bit 2 of PORTB to 1 -<-> Enable pull-up on PB2

	; Cofigure LEB Pin PC2 as input
	SBI DDRC, 2			; Set bit 2 of DDRC to 1 -> PC2 is an output
	SBI PORTC, 2		; Initially set PC2 to HIGH to turn the LED off
	rjmp main


main:
	in r16, PINB		; Read the state of all port B pins and store them into the general purpose register r16
	BST r16, 2			; Bit Store into T flag: store bit 2 of r16 (PB2) into T-flag
	BRTC JoyPressed		; Branch if T-flag is cleared (set to 0)

	; If Joystick is not pressed
	SBI PORTC, 2
	rjmp main

JoyPressed:
	CBI PORTC, 2
	rjmp main