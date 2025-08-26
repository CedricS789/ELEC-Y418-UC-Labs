;
; UC_task4.asm
;
; Created: 20/07/2025 10:43:38
; Author : Cédric
;

.include "m328pdef.inc"


.def delay_counter = r18	; Counter for the sound delay


.org 0x0000
		rjmp init


init:
		sbi DDRB, 1				; Configure PB1 as an output
		rjmp main


main:
		
		sbi PINB, 0				; Toggle the state of PB0
		rcall sound_delay		; This delay set the pitch of the tone
		rjmp main


; ----- Sound Delay subroutine -----
sound_delay:
		ldi delay_counter, 165
		loop:
				dec delay_counter
				brne loop		; Repeat as long as delay_counter is not equal zero
		ret						; Return from subroutine