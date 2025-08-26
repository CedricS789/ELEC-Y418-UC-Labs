.include "m328pdef.inc"

.org 0x0000
	rjmp init

init:
	sbi DDRB, 5		;setting PB5 as an output
	cbi PORTB, 5	;setting all the value to 0
	sbi DDRB, 4		;setting PB4 as an output
	cbi PORTB, 4	;setting all the value to 0
	sbi DDRB, 3		;setting PB3 as an output
	cbi PORTB, 3	;setting all the value to 0
	
	


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
	inc R18
	RJMP start             															; Begin display loop for this symbol

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
	
	rjmp main_over
	 

	db_val:
	//.db 0x00, 0x00, 0x80, 0x00
	.db 0x00, 0x80, 0x80, 0x00
	.db 0x00, 0x00, 0x00, 0x00
	.db 0x18, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x00
	.db 0x3c, 0x00, 0x00, 0x00
	.db 0x3c, 0x00, 0x00, 0x00
	.db 0x18, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x00


check_value:
	LDI ZL, low(db_val<<1)
		LDI ZH, high(db_val<<1)
		INC R18
	RET



