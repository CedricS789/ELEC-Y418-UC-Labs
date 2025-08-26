;
; UC_task1.asm
;
; Created: 19/07/2025 13:13:27
; Author : Cédric
;

.include "m328pdef.inc"


.org 0x0000
	rjmp init


init:
		; Configure PB2 (joystick) as input with pull up
		cbi DDRB, 2			; set bit 2 in DDRB  to 0	
		sbi PORTB, 2		; set bit 2 of PORTB to 1

		; Configure PC2 (LED1) as output
		sbi DDRC, 2			; Set bit 2 of DDRC to 1
		sbi PORTC, 2		; Set bit 2 of PORTC to 1
		rjmp main			; Jump to the main program loop


main:
		in r16, PINB		; Load data from all port Pin B into the register r16
		bst r16, 2			; Store bit 2 of r16 in T flag
		brtc JoyPressed		; Branch if T flag is set to 0


JoyNotPressed:
		sbi PORTC, 2		; Set bit 2 of PORTC to 1 (Turn off the LED)
		rjmp main			; Jump back to the start of the main loop


JoyPressed:
		cbi PORTC, 2		; Set bit 2 of PORTC to 0 (Turn on the LED)
		rjmp main			; Jump back to the start of the main loop
