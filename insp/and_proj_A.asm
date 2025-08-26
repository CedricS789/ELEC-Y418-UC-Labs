;;;;;; Main Function
;;;; main_work uses 4 main functional blocks: main_key, write_input, main_over, and buzzer
;;;; main_key scans the key matrix, determines the pressed key, encodes it, and passes it to write_input
;;;; write_input generates the first and second number (MM/SS); result is in R20
;;;; main_over updates the screen using R25 (minutes) and R20 (seconds)
;;;; buzzer handles sound output

;;;;; R26 Codings
;;;; 71 – New button pressed ? update value
;;;; 73 – Timer Start
;;;; 74 – Buzzer active
;;;; 75 – Add extra 10 seconds
;;;; 77 – Buzzer delay state
;;;; 7F – Reset

; Structure of key/numbers (n = not used) 
; 7 8 9   Reset
; 4 5 6   Extra 10 seconds
; 1 2 3   n
; 0 n n n

; During timer counting, only Reset key (F) is active.
; During buzzer state, only Reset (F) and Extra 10 Seconds (E) keys are active.


.INCLUDE "m328pdef.inc"																	; Include AVR definitions
origin:																					
.ORG 0x0000																				
	RJMP init																			; On reset, jump to the 'init' routine
																						
.ORG 0x0016																				
    RJMP TIMER1_COMPA_ISR																; Interrupt vector for Timer1 Compare Match A
																						
																						
;  Initialization Code 															
init:																					
	CLI																					; Disable global interrupts during setup

;  Reset Timer1 counter to 0 													

    LDI R16, 0x00																		
    STS TCNT1H, R16																		; High byte of Timer1 = 0
    STS TCNT1L, R16																		; Low byte of Timer1 = 0
																						
;  Set Timer1 Compare Match value to 15624 									
; (For 1Hz with 16MHz clock and 1024 prescaler)										
	
    LDI   r16, HIGH(15624)																
    STS   OCR1AH, r16																	
    LDI   r16, LOW(15624)																
    STS   OCR1AL, r16																	
																						
;  Configure Timer1: CTC mode, Prescaler = 1024 								
; WGM12 = 1 (CTC mode)																
; CS12 and CS10 = 1 (prescaler 1024)												

    LDI R16, (1 << WGM12) | (1 << CS12) | (1 << CS10)									
    STS TCCR1B, R16																		
																						
;  Clear TCCR1A  																

    LDI R16, 0x00																		
    STS TCCR1A, R16																		
																						
;  Enable Timer1 Compare Match A interrupt 									

    LDI R16, (1 << OCIE1A)																
    STS TIMSK1, R16																			
																						
;  Configure multiple output pins and turn them off (display) 				

	SBI DDRB,5               															; Set PB5 as output										
	CBI PORTB,5              															; Set PB5 as low											
	SBI DDRB,3               															; Set PB3 as output										
	CBI PORTB,3              															; Set PB3 as low											
	SBI DDRB,4               															; Set PB4 as output										
	CBI PORTB,4              															; Set PB4 as low											
																							
;  Configure PB0 as input for button with pull-up (switch) 					

    CBI DDRB, 0          																; Set PB0 as input													
    SBI PORTB, 0         																; Enable internal pull-up (button connected to GND)			
																							
;  Configure PB1 as output for buzzer and turn it off 							

	SBI DDRB, 1              															; Set PB1 as output											
	CBI PORTB, 1             															; Set PB1 low (buzzer off)									

;  Configure PB2 as input for joystick with pull-up 

	CBI DDRB, 2              															; Set PB2 as input 
	SBI PORTB, 2             															; Enable internal pull-up (active LOW when pressed)

;  Configure PC2 as output for led and turn it off 

	SBI DDRC,2               															; Set PC2 as output 
	SBI PORTC,2              															; LED off (assumes active low)

; Configure PC3 as output for led and turn it off initially 

    SBI DDRC, 3         								 								; Set PC3 as output
    SBI PORTC, 3         																; LED off (assumes active low)
																						
; Initialize necessary registers									
	LDI R16, 0			  																; Clear key buffer
	LDI R17, 0			  																; Clear key buffer
	LDI R18, 0		      																; Clear key buffer
	LDI R20, 0            																; Clear seconds register
	LDI R21, 4			  																; set miniute second digits state(decrememnting)
	LDI R22, 0            																; Clear key buffer
	LDI R23, 0			  																; Clear key buffer
	LDI R24, 0            																; Clear temporary key value buffer  
	LDI R25, 0            																; Clear minutes register
	LDI R26, 0            																; Clear key buffer
	LDI R27, 0            																; Clear key buffer
	LDI R28, 0            																; Clear key buffer
	LDI R29, 0            																; Clear key buffer
	LDI R30, 0            																; Clear key buffer
	SEI																					
																						
;Start state machine												
																						
state_machine:																			
	RCALL main_over		   																; show time on display
						   																
	SBIS PINB,2            																; If joystick is pressed
			LDI R21, 0     																; Clear input state (remained numbers are 0s)
						   																
	CPI R26, 0x73		   																; if already in counting or buzzing state don't change state back to counting
	BRGE already_timer	   																
						   																
	CPI R21, 0x00		   																;if all digits are wirtten or joystick pressed start timer
	BRNE already_timer	   																
		LDI R26, 0x73      																;change state extra second before counting
						   																			   																
	already_timer:		   																
	RCALL main_key		   																; find pressed key R26 = 0xF1 if button pressed
	                       																
	CPI R26, 0x71		   																;if button pressed state 
	BREQ  call_write_input 																;update memory register if 
						   																
	CPI R26, 0x7F		   																;reset pressed
	BREQ hard_reset  	   																
						   																
	CPI R26, 0x77		   																;buzzer wait state
		BREQ state_machine 																
						   																
	CPI R26, 0x74		   																;buzzer state
	BRGE call_buzzer       																
						   																
	rjmp state_machine     																
																						
call_write_input:																		
	RCALL write_input     																;call min sec decoding function
	CPI R21,2			  																;check are we in minutes or in seconds
	BRLO not_minutes	  																
	OR R25,R20			  																;if in minutes update minute register 
	LDI R20,0			  																; reset seconds register
	not_minutes:          																
	LDI R26,0			  																; reset state
	RJMP state_machine    																
																																											
hard_reset:																																										
	RJMP origin			  																
						  																
call_buzzer:			  																
	RCALL buzzer		  																
	RJMP state_machine	  																

main_key:				  																						  																
	LDI R27, 0            																; Clear counter register
	LDI R28, 0x00         																; Temporary register to store key column info
																						
; Set all 4 columns (PD0–PD3) HIGH													

	SBI PORTD,0																					
	SBI PORTD,1
	SBI PORTD,2
	SBI PORTD,3

; Set columns as INPUTs with pull-up (to read column lines)

	CBI DDRD,3
	CBI DDRD,2
	CBI DDRD,1
	CBI DDRD,0

; Set rows (PD4–PD7) as OUTPUT

	SBI DDRD,7
	SBI DDRD,6
	SBI DDRD,5
	SBI DDRD,4

; Set rows LOW (this allows pressed key to pull column LOW)

	CBI PORTD,7
	CBI PORTD,6
	CBI PORTD,5
	CBI PORTD,4
	NOP                   																; Small delay
	NOP

	SBIS PIND,0
		RJMP col0
	CPI R26, 0x73																		; don't use first second and thirth column keys during timer state 
	BRGE dont_use_keys
	SBIS PIND,1
		RJMP col1
	SBIS PIND,2
		RJMP col2
	SBIS PIND,3
		RJMP col3
	RET   																				; No key detected in this column pass
	
; Assign column-specific bits to R28

col0:
	ORI R28, 0x30         																; Column 0 (rightmost): OR with 0x30
	RJMP SKIP_COL_CHECK

col1:
	ORI R28, 0x20         																; Column 1
	RJMP SKIP_COL_CHECK

col2:
	ORI R28, 0x10         																; Column 2
	RJMP SKIP_COL_CHECK

col3:
	ORI R28, 0x00         																; Column 3 (leftmost)
	RJMP SKIP_COL_CHECK

; Continue after column scan 

SKIP_COL_CHECK:
	; Set rows HIGH again
	SBI PORTD,7
	SBI PORTD,6
	SBI PORTD,5
	SBI PORTD,4

	; Change rows to INPUT (disconnect)
	CBI DDRD,7
	CBI DDRD,6
	CBI DDRD,5
	CBI DDRD,4

	; Change columns back to OUTPUT LOW (prepare for next scan)
	SBI DDRD,0
	SBI DDRD,1
	SBI DDRD,2
	SBI DDRD,3

	CBI PORTD,0
	CBI PORTD,1
	CBI PORTD,2
	CBI PORTD,3
	NOP
	NOP

; Check each row for key press 

	SBIS PIND,7           																; Top row (first row)
		RJMP lin7
	SBIS PIND,6           																; Second row
		RJMP lin6	
	CPI R26, 0x73		  																;don't use keys bottom first and second rows 
	BRGE dont_use_keys
	CPI R28, 0x30		  																; don't use keys D C
	BREQ not_allowed_keys
	SBIS PIND,5           																; Third row
		RJMP lin5
	CPI R28, 0x00		  																; Don't use keys 0 B
	BRNE not_allowed_keys
	SBIS PIND,4           																; Bottom row
		RJMP lin4

	not_allowed_keys:
		LDI R28,0
	dont_use_keys:
		RET

	check_reset:	
		CBI PORTC,2		 																; turn on light	
		LDI R26, 0x7F	 																; reset code
		SBIC PIND,7  
			RET
		RJMP check_reset

; If a key is pressed on row 7 

lin7:
	CPI R28, 0x30           															; Check if "F" key (used as reset)
	BREQ check_reset        															; If so, go to reset logic
	CPI R27, 0xff           															; Wait until delay/counter hits 1
	BREQ do_7
	INC R27
	RJMP lin7

do_7:
	CBI PORTC,2           																; Turn off visual LED indicator (feedback)
	SBIS PIND,7           																; Wait for key release
		RCALL main_over
	SBIS PIND,7           																; Wait for key release
		RJMP do_7
	SBI PORTC,2           																; Turn LED on (registered input)
	ORI R28, 0x07         																; Register value for this key
	LDI R26, 0x71																			
	RET
																						; other linx: have same logic
lin6:
	CPI R28, 0x30
		BREQ extra_10
	CPI R27, 0xff
	BREQ do_6
	INC R27
	RJMP lin6
do_6:
	CBI PORTC,2
	SBIS PIND,6
		RCALL main_over
	SBIS PIND,6
		RJMP do_6
	SBI PORTC,2
	ORI R28, 0x04
	LDI R26, 0x71
	RET

lin5:
	CPI R27, 0xff
	BREQ do_5
	INC R27
	RJMP lin5
do_5:
	CBI PORTC,2
	SBIS PIND,5
		RCALL main_over
	SBIS PIND,5
		RJMP do_5
	SBI PORTC,2
	ORI R28, 0x01
	LDI R26, 0x71
	RET

lin4:
	CPI R27, 0xff
	BREQ do_4
	INC R27
	RJMP lin4
do_4:
	CBI PORTC,2
	SBIS PIND,4
		RCALL main_over
	SBIS PIND,4
		RJMP do_4
	SBI PORTC,2
	ORI R28, 0x00
	LDI R26, 0x71
	RET

; change to extra 10 seconds state

extra_10:
	CPI R26, 0x74 																		;check are we in buzzer state if no pass
	BRGE not_usable
		RET
	not_usable:
		LDI R26, 0x75	    															; change stateto extra 10
		RET	

; Decode value from register R17 into minutes and seconds (R17[7:4] + R17[3:0]) 

write_input:
	MOV R24, R28              															; Copy R17 (raw key input) into R24 for processing
	ANDI R24, 0xf0            															; Mask upper 4 bits of R24 (high nibble = column info)
	ANDI R28, 0x0f            															; Mask lower 4 bits of R17 (low nibble = row info)
	LSR R24                   															; Logical shift right (1)
	LSR R24                   															; (2)
	LSR R24                   															; (3)
	LSR R24                   															; (4) — total: move high nibble to low nibble position
	ADD R24, R28             															; Add both nibbles to get the final key value (0–15)

; Determine which digit is being entered based on R21 
	
	CPI  R21, 4               															; First minute digit (tens place)
	BREQ first_number

	CPI  R21, 1               															; Second second digit (units place)
	BREQ second_number

	CPI  R21, 2               															; First second digit (tens place)
	BREQ first_number

	CPI  R21, 3               															; Second minute digit (units place)
	BREQ second_number

	not_allowd:
		RET

; First minute digit (tens place of minutes) 

first_number:
	MOV R22, R24
	SUBI R22, 0x06            															; Ensure value is less than 6
	BRSH not_allowd           															; Reject values 6 or more
	MOV R20, R24              															; Move value to R20 (seconds)
	LSL R20                   															; Shift 4 bits to high nibble
	LSL R20
	LSL R20
	LSL R20                   															; R20 = R24 × 0x10 (high nibble of seconds)
	DEC R21
	RET                       															; Update screen

;  Second second digit (units place of seconds) 

second_number:
	MOV R22, R24
	SUBI R22, 0x0a            															; Check if value is < 10
	BRSH not_allowd
	ANDI R24, 0x0f            															; Mask value
	OR R20, R24               															; Add to R20 (seconds low nibble)
	DEC R21
	RET            																		; Update screen

; Start Screen Printing Part 

main_over:
	LDI R17, 0                															; Buffer byte reader for screen pattern
	LDI R23, 0                															; Display byte counter
	LDI R22, 0x0F             															; Mask value for digit extraction
	LDI R18, 0                															; State machine step counter

main_screen:
	LDI R18, 0                															; Reset display state machine

start:

; State machine to draw each part of the display 
	
	CPI R18, 0
	BREQ step_0_screen        															; First minute digit

	CPI R18, 1
	BREQ step_1_screen        															; Second minute digit

	CPI R18, 2
	BREQ step_2_screen        															; Colon ":"

	CPI R18, 3
	BREQ step_3_screen        															; First second's digit

	CPI R18, 4
	BREQ step_4_screen        															; Second second's digit
	RET          																		; If nothing matched, restart

; Extract first minute digit (high nibble of R25) 

step_0_screen:
	LDI R22, 0xF0
	AND R22, R25
	LSR R22
	LSR R22
	LSR R22
	LSR R22                 															; Shift to get actual digit (0–5)
	RCALL check_value
	rjmp start_2

; Extract second minute digit (low nibble of R25) 

step_1_screen:
	LDI R22, 0x0F
	AND R22, R25
	RCALL check_value
	rjmp start_2

; Draw the double dot ":" symbol 

step_2_screen:
	LDI ZL, low(double_dot << 1)
	LDI ZH, high(double_dot << 1)
	INC R18
	RJMP start_2             															; Begin display loop for this symbol

; Extract first second digit (high nibble of R20) 

step_3_screen:
	LDI R22, 0xF0
	AND R22, R20
	LSR R22
	LSR R22
	LSR R22
	LSR R22
	RCALL check_value
	rjmp start_2

; Extract second second digit (low nibble of R20) 

step_4_screen:
	LDI R22, 0x0F
	AND R22, R20
	RCALL check_value
	rjmp start_2

; Start writing character data to the display 

start_2:
	LDI R17, 0x00             															; Reset data byte for shifting
	LDI R19, 0x00             															; Reset column index
	mov R24, R18              															; Copy screen step into position register
	DEC R24                   															; Adjust index since digits start from 0

take_value:
	CPI R19, 11               															; Check if all 11 vertical slices are processed
	BREQ push_val

	CPI R19, 10               															; If row marker, re-adjust
	BREQ update_index
	RJMP dont_update

update_index:
	LDI R24, 4                															; Force to index 4 (for colon maybe?)

dont_update:
	INC R19
	CPI R24, 4
	BREQ update_val          															; If in correct position, load next byte
	LDI R17, 0x00            															; Else blank value
	INC R24
	RJMP update_iter_loop

update_val:
	LPM R17, Z+              															; Load one byte from program memory into R17
	LDI R24, 0               															; Reset position

update_iter_loop:
	LDI R16, 8               															; We need to shift out 8 bits for 1 column
	RJMP loop_col

; Shift register routine for sending bits to display 

loop_col:
	DEC R16
	CLC
	ROR R17                  															; Rotate right to check bit
	BRCS output_1            															; If bit set, send 1
	CBI PORTB,3              															; Else, set data pin (PB3) LOW

bak:
	CBI PORTB,5              															; Toggle clock LOW
	SBI PORTB,5              															; Toggle clock HIGH
	CBI PORTB,5              															; Toggle clock LOW again

	CPI R16, 0
	BREQ take_value          															; If all bits sent, get next column
	RJMP loop_col

output_1:
	SBI PORTB,3              															; Set data pin (PB3) HIGH
	RJMP bak

; Latch data after 11 columns are sent 

push_val:
	CBI PORTB,4              															; Latch LOW
	SBI PORTB,4              															; Latch HIGH
	CBI PORTB,4              															; Latch LOW

	LPM R17, Z+              															; Prepare next value
	CPI R23, 10              															; Check if 10 characters (digits) sent
	BREQ reset_r23
	INC R23
	RJMP start_2

reset_r23:
	LDI R23, 0               															; Reset digit counter
	CPI R18, 5
	BREQ go_back_to_key      															; If all 5 elements (2 min, colon, 2 sec) are drawn
	RJMP start               															; Go to next element

go_back_to_key:
	CPI R21, 0
	BREQ go_back_to_start				 												; If we're done with screen update
	RET						 															; If still in key mode, go to keypad
go_back_to_start:
	RJMP start               															;BREQ  out of range


buzzer:
	LDI R30,0			   																; resset buzzer waiting counter
	CPI R26, 0x75		   																; is there extra 10 sec
		BREQ extra_10_sec  

	i_wake_up_no_dealy_needed:

		inc R30
		CPI R30, 0x40																	; wait 0x40 iter	
		BRNE i_wake_up_no_dealy_needed
		SBI PINB, 1             														; toggle buzzer (toggle PB1)
	
		CPI R29, 0x40																	;toggle in total 0x40 times
		BREQ buzzed             														; if 40 times toggled change state to wait after waiting a second change back to buzzer	
		INC R29                 														; Increment   counter
		RET																				;jump back update screen check buttons

	buzzed:
		LDI R26, 0x77		    														;change state to wait
		RET 

; If top-right key pressed, add a delay before restarting 

	extra_10_sec:
		LDI R20, 0x10            														; Set seconds to 0x10 (16) for 2-second wait
		LDI R25, 0x00            														; Set minutes to 0
		LDI R26, 0x73
		RET						 														; Go back

; Timer1 Compare Match A Interrupt Handler 

TIMER1_COMPA_ISR: 
	
	SBIC PINB, 0               															; switch low work / high pouse 
		RETI   
	CPI R26, 0x77			   															; check wait state
	BREQ update_buzzer

	CPI R26, 0x74			   															; noting needed from interupt during buzzing or extra 10 implementing part
	BRGE go_back

	CPI R26, 0x73			   															; timer state jump to decrementing logic
	BREQ start_iter

	RETI
; Pause functionality 
 update_buzzer:																			; change state to buzzer reset buzzer counter
		LDI R26, 0x74																	;change state to buzzer 
		LDI R29,0																		; reset buzzer counter
		RETI			
	go_back:
		RETI                
; Decrement seconds 
start_iter:
	CPI R20, 0                 															; If seconds already 0...
	BREQ sec_0                 

; Otherwise, just decrement normally

	DEC R20
	LDI R22, 0x0F              															; Check if lower nibble is 0xF (from 0x10 -> 0x0F)
	AND R22, R20
	CPI R22, 0x0F
	BREQ overflow_sec          															; If overflowed, fix it
	RETI

overflow_sec:                  															; Handle BCD-style decrement correction
	ANDI R20, 0xF0             															; Clear lower nibble
	ORI R20, 0x09              															; Set it to 9 (e.g., 0x20 -> 0x29)
	RETI

; Handle seconds underflow: go to minutes 

sec_0:
	CPI R25, 0                 															; If minutes also 0, we're done
	BREQ min_0                 															; Go to buzzer

	DEC R25                    															; Otherwise, decrement minutes
	LDI R20, 0x59              															; Reset seconds to 59 (BCD)
	LDI R22, 0x0F
	AND R22, R25
	CPI R22, 0x0F
	BREQ overflow_min
	RETI

overflow_min:                 															; BCD fix for minutes too (when lower nibble is F)
	ANDI R25, 0xF0
	ORI R25, 0x09
	RETI

; When both minutes and seconds reach 0: trigger buzzer 

min_0:
	LDI R26, 0x74
	LDI R29, 0				   														 	; without reseting R29 at 00:00 after adding extra 10 R29 has some value so first beep will be shorter after extra 10 sec
	RETI                       														 	; Return from interrupt
		



;reset timer for 1 sec

;From here its just declaretion of numbers

one:
	.db 0x08, 0x08, 0x80, 0x00
	.db 0x00, 0x00, 0x40, 0x00
	.db 0x08, 0x18, 0x40, 0x00
	.db 0x00, 0x00, 0x20, 0x00
	.db 0x08, 0x28, 0x20, 0x00
	.db 0x00, 0x00, 0x10, 0x00
	.db 0x08, 0x48, 0x10, 0x00
	.db 0x08, 0x08, 0x08, 0x00
	.db 0x08, 0x08, 0x04, 0x00
	.db 0x08, 0x08, 0x02, 0x00
	.db 0x00, 0x00, 0x01, 0x00

two:
	.db 0xf8, 0xf8, 0x80, 0x00
	.db 0x80, 0x08, 0x40, 0x00
	.db 0x80, 0x08, 0x20, 0x00
	.db 0x80, 0x08, 0x10, 0x00
	.db 0x80, 0x08, 0x08, 0x00
	.db 0x80, 0x08, 0x04, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0xf8, 0x08, 0x02, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00

three:
	.db 0xf8, 0xf8, 0x80, 0x00
	.db 0x08, 0x08, 0x40, 0x00
	.db 0x08, 0x08, 0x20, 0x00
	.db 0x08, 0x08, 0x10, 0x00
	.db 0x08, 0x08, 0x08, 0x00
	.db 0x08, 0x08, 0x04, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0xf8, 0x08, 0x02, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00

four:
	.db 0x08, 0x88, 0x80, 0x00
	.db 0x08, 0x88, 0x40, 0x00
	.db 0x08, 0x88, 0x20, 0x00
	.db 0x08, 0x88, 0x10, 0x00
	.db 0x08, 0x88, 0x08, 0x00
	.db 0x08, 0x88, 0x04, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x08, 0xf8, 0x02, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00

five:
	.db 0x08, 0xf8, 0x80, 0x00
	.db 0x08, 0x80, 0x40, 0x00
	.db 0x08, 0x80, 0x20, 0x00
	.db 0x08, 0x80, 0x10, 0x00
	.db 0x08, 0x80, 0x08, 0x00
	.db 0x08, 0x80, 0x04, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0xf8, 0xf8, 0x02, 0x00
	.db 0xf8, 0xf8, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00

six:
	.db 0xf8, 0xf8, 0x80, 0x00
	.db 0x88, 0x80, 0x40, 0x00
	.db 0x88, 0x80, 0x20, 0x00
	.db 0x88, 0x80, 0x10, 0x00
	.db 0x88, 0x80, 0x08, 0x00
	.db 0x88, 0x80, 0x04, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0xf8, 0x80, 0x02, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00

seven:
	.db 0x08, 0xf8, 0x80, 0x00
	.db 0x08, 0x08, 0x40, 0x00
	.db 0x08, 0x08, 0x20, 0x00
	.db 0x08, 0x08, 0x10, 0x00
	.db 0x08, 0x08, 0x08, 0x00
	.db 0x08, 0x08, 0x04, 0x00
	.db 0x08, 0x08, 0x02, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00

eight:
	.db 0x88, 0xf8, 0x80, 0x00
	.db 0x88, 0x88, 0x40, 0x00
	.db 0x88, 0x88, 0x20, 0x00
	.db 0x88, 0x88, 0x10, 0x00
	.db 0x88, 0x88, 0x08, 0x00
	.db 0x88, 0x88, 0x04, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0xf8, 0xf8, 0x02, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00

nine:
	.db 0xf8, 0xf8, 0x80, 0x00
	.db 0x08, 0x88, 0x40, 0x00
	.db 0x08, 0x88, 0x20, 0x00
	.db 0x08, 0x88, 0x10, 0x00
	.db 0x08, 0x88, 0x08, 0x00
	.db 0x08, 0x88, 0x04, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0xf8, 0x88, 0x02, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00

zero:
	.db 0x88, 0xf8, 0x80, 0x00
	.db 0x88, 0x88, 0x40, 0x00
	.db 0x88, 0x88, 0x20, 0x00
	.db 0x88, 0x88, 0x10, 0x00
	.db 0x88, 0x88, 0x08, 0x00
	.db 0x88, 0x88, 0x04, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0xf8, 0x88, 0x02, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00

double_dot:
	//.db 0x00, 0x00, 0x80, 0x00
	.db 0x00, 0x00, 0x40, 0x00
	.db 0x00, 0x00, 0x31, 0x00
	.db 0x18, 0x18, 0x21, 0x00
	.db 0x00, 0x00, 0x11, 0x00
	.db 0x3c, 0x3c, 0x10, 0x00
	.db 0x3c, 0x3c, 0x08, 0x00
	.db 0x18, 0x18, 0x04, 0x00
	.db 0x00, 0x00, 0x02, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00
	.db 0x00, 0x00, 0x01, 0x00


; Check and Load Correct Number Pattern 
check_value:
	CPI  R22, 0        																	; Compare digit value with 0
	BREQ detected_0    																	; If equal, go to digit 0 pattern

	CPI R22,1
	BREQ detected_1

	CPI R22,2
	BREQ detected_2

	CPI R22,3
	BREQ detected_3

	CPI R22,4
	BREQ detected_4

	CPI R22,5
	BREQ detected_5

	CPI R22,6
	BREQ detected_6

	CPI R22,7
	BREQ detected_7

	CPI R22,8
	BREQ detected_8

	CPI R22,9
	BREQ detected_9


detected_1:
	LDI ZL, low(one<<1)     															; Load low byte of address of `one` (word address, so shift left by 1)
	LDI ZH, high(one<<1)    															; Load high byte of address
	INC R18                 															; Move to next display element (e.g. from minutes digit 1 to digit 2)
	RET            																		; Jump to display loop to start rendering this digit

detected_2:
	LDI ZL, low(two<<1)
	LDI ZH, high(two<<1)
	INC R18
	RET

detected_3:
	LDI ZL, low(three<<1)
	LDI ZH, high(three<<1)
	INC R18
	RET

detected_4:
	LDI ZL, low(four<<1)
	LDI ZH, high(four<<1)
	INC R18
	RET

detected_5:
	LDI ZL, low(five<<1)
	LDI ZH, high(five<<1)
	INC R18
	RET

detected_6:
	LDI ZL, low(six<<1)
	LDI ZH, high(six<<1)
	INC R18
	RET

detected_7:
	LDI ZL, low(seven<<1)
	LDI ZH, high(seven<<1)
	INC R18
	RET

detected_8:	
	LDI ZL, low(eight<<1)
	LDI ZH, high(eight<<1)
	INC R18
	RET

detected_9:
	LDI ZL, low(nine<<1)
	LDI ZH, high(nine<<1)
	INC R18
	RET

detected_0:
	LDI ZL, low(zero<<1)
	LDI ZH, high(zero<<1)
	INC R18
	RET


