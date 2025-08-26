.equ tcnt0_init_440Hz = 185			; The value of TCNT0_init for 440Hz, Prescaler = 256
.equ tcnt0_init_880Hz = 220			; The value of TCNT0_init for 880Hz, Prescaler = 256

.org 0x0000
jmp init

.org 0x0020
rjmp ISR_Timer0_OVF


init:
	; ---- Pin Configuration ----
	cbi DDRB, 2		; clear bit 2 of DDRB register -> set PB2 as an input
	sbi PORTB, 2	; set bit 2 of PORTB register -> activate internal pull up to VCC

	cbi DDRB, 0		; Make PB0 an input
	sbi PORTB, 0	; Activate pull-up resistor

	sbi DDRB, 1		; set bit 1 of DDRB register -> set PB1 as an output (buzzer)

	; ---- Interruption configuration ----
	; The comiler knows that the constant T0IE0=0
	ldi r16, (1<<TOIE0)						; Set the bit last bit of Timer/Counter Interrupt Mask Register (TIMSK0) to 1 -> Enable Timer0 overflow Interrupt
	sts TIMSK0, r16							; We use the command sts because out cannot access the address space the Timer/Counter Interrup Mask register (TIMSK0 register) is in
	sei										; Set Global Interrupt Enable: this command activate all interupts
	rjmp main


main:
	; ---- Check the switch state ---
	in r16, PINB	; read the value of the switch
	bst r16, 0		; store bit 0 of r16 into t-flag
	brts SwitchIsOn	; branch if PB0 = 1

	; Switch is OFF, set for 440Hz			
	ldi r17, tcnt0_init_440Hz
	rjmp CheckJoyButton


SwitchIsOn:
	ldi r17, tcnt0_init_880Hz


CheckJoyButton:
	in r16, PINB							; read the value of joystick
	bst r16, 2								; store the value in t flag
	brtc JoyPressed							; branch if joystick is pressed (PB2=0)

	; Joystick is not pressed
	clr r16									; set r16 to 0x00
	out TCCR0B, r16						; Timer/Counter Control Register 0B. set it to 0x00 -> turn it off
	rjmp main


JoyPressed:
	ldi r16, (1<<CS02)						; Prescaler = 256
	out TCCR0B, r16							; Timer/Counter Control Register 0B. Set it to 0b00000100 -> Prescaler = 256
	rjmp main


ISR_Timer0_OVF:
	push r16								; save the state of r16 register to prevent correuption of data by the ISR
	in r16, SREG							; save SREG to prevent correuption if data by the ISR
	push r16

	out TCNT0, r17							; reload the timer with TCNT0_init to have the desire frequency
	sbi PINB, 1								; toggle the buzzer pin

	pop r16									; restore SREG. With uncorrupted data before resuming the main loop
	out SREG, r16							; restore SREG. With uncorrupted data before resuming the main loop

	pop r16									; restore r16. With uncorrupted data before resuming the main loop

	reti									; return from Interupt