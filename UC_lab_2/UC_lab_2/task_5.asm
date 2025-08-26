; ============================================================
;
; Task 5 - Buzzer at 440 Hz using Timer0 Interrupt
;
; ============================================================


.org 0x0000
rjmp init

.org 0x0020			; Timer0 Overflow vector adress
rjmp Timer0_Overflow_ISR

init:
	; ---- Pin Configuration ----
	cbi DDRB, 2		; clear bit 2 of DDRB register -> set PB2 as an input
	sbi PORTB, 2	; set bit 2 of PORTB register -> activate internal pull up to VCC

	sbi DDRB, 1		; set bit 1 of DDRB register -> set PB1 as an output (buzzer)
	
	 ; ---- Timer 0 Configuration ----
	 ; I found that a prescaler of 256 is required
	 ; The compiler knows that the constants CS02=2, CS01=1, and CS00=0. what is written here is for clarity
	 ldi r16, (1<<CS02 | (0<<CS01) | (0<<CS00))  ; we are loading in register r16 the value 0b00000100 such that the last three bits are "1 0 0", this corresponds to a prescaler of 256 (p.108 Datasheet)
	 out TCCR0B, r16							; write the value of r16 into register TCCR0B -> start timer with the 256 prescaler

	 ; ---- Interruption configuration ----
	 ; The comiler knows that the constant T0IE0=0
	 ldi r16, (1<<TOIE0)						; Set the bit last bit of Timer/Counter Interrupt Mask Register (TIMSK0) to 1 -> Enable Timer0 overflow Interrupt
	 sts TIMSK0, r16							; We use the command sts because out cannot access the address space the Timer/Counter Interrup Mask register (TIMSK0 register) is in
	 sei										; Set Global Interrupt Enable: this command activate all interupts

	 rjmp main

main:
	in r16, PINB								; Read all values of PINB register into r16 (all PBx states)
	bst r16, 2									; Bit Store from Bit in Resgiter to T Flag in SREG (Status Register): store bit 2 into the T-flag
	brtc JoyPressed

	; ---- Joystick Not Pressed: Stop the timer ----
	clr r16										; Clear register: set r16 to 0b00000000
	out TCCR0B, r16								; Set register 
	rjmp main

JoyPressed:
	; ---- Joystick Is Pressed: Run the timer ----
	ldi r16, (1<<CS02 | (0<<CS01) | (0<<CS00))
	out TCCR0B, r16
	rjmp main

Timer0_Overflow_ISR:
	; Reload the timer's initial value to maintain the correct frequency
	ldi r16, 185
	out TCNT0, r16								; Timer/Counter Register: this is the actual value of the counter that is initialized to TCNT0_init = 185 ----- (TCNT0_init = 256 - (fclk_timer/f_interrupt)) ; (fclk_timer = fclk_sys/prescaler)

	; Toggle the Buzzer pin
	sbi PINB, 1									; This is a convenient shortcut that toggles the value of PORTB_1
	reti										; return from interrupt