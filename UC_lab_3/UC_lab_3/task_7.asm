; ==========================================================
; Lab 3, Task 7: Readout Keyboard and control outputs
; ==========================================================
.include "m328pdef.inc"         ; Include standard definitions for the ATmega328P (register names, bit names, etc.).

; --- Definitions ---
.def temp      = r16             ; Create an alias 'temp' for general-purpose register r16.
.def key_code  = r19             ; Create an alias 'key_code' for r19, which will store the pressed key's ID.

; --- Keypad Mapping Constants ---
.equ KEY_A = 1
.equ KEY_0 = 2
.equ KEY_B = 3
.equ KEY_C = 4

.equ KEY_1 = 5
.equ KEY_2 = 6
.equ KEY_3 = 7
.equ KEY_D = 8

.equ KEY_4 = 9                  ; Assign the name 'KEY_4' to the scan code 9.
.equ KEY_5 = 10
.equ KEY_6 = 11
.equ KEY_E = 12

.equ KEY_7 = 13                 ; Assign the name 'KEY_7' to the scan code 13.
.equ KEY_8 = 14                 ; Assign the name 'KEY_8' to the scan code 14.
.equ KEY_9 = 15                 ; Assign the name 'KEY_9' to the scan code 15.
.equ KEY_F = 16


.org 0x0000                     ; Set the program counter to address 0x0000, the RESET vector.
    rjmp init                   ; Place a jump to the 'init' label. This is the first code to run on power-up.

.org 0x0020                     ; Set the program counter to 0x0020, the Timer0 Overflow interrupt vector.
    rjmp Timer0_Overflow_ISR    ; Place a jump to the interrupt handler. This code runs when Timer0 overflows.

init:
    ; --- Pin Configuration ---
    sbi     DDRC, 2             ; Set bit 2 of Data Direction Register C. Configures pin PC2 as an OUTPUT (for LED1).
    sbi     DDRC, 3             ; Set bit 3 of Data Direction Register C. Configures pin PC3 as an OUTPUT (for LED2).
    sbi     DDRB, 1             ; Set bit 1 of Data Direction Register B. Configures pin PB1 as an OUTPUT (for the buzzer).

    ; --- Keyboard Init ---
    ldi     temp, 0b11110000    ; Load immediate value into temp.
    out     DDRD, temp          ; Set PORTD pins 7-4 as OUTPUTS (keypad rows) and pins 3-0 as INPUTS (keypad columns).
    ldi     temp, 0b11111111    ; Load temp with all 1s.
    out     PORTD, temp         ; Set output rows HIGH (inactive state) and enable internal pull-up resistors on input columns.

    ; ---- Timer0 and Interrupt Configuration ----
    ldi     temp, (1<<TOIE0)    ; Load temp with a mask to enable the Timer0 Overflow Interrupt. (1<<TOIE0) results in 0b00000001.
    sts     TIMSK0, temp        ; Store this value to the Timer Interrupt Mask Register 0 to enable the overflow interrupt.
    sei                         ; Set the Global Interrupt Enable bit in the Status Register (SREG) to allow interrupts to occur.

    rjmp    main                ; Jump to the main program loop.

; --- Main Program Loop ---
main:
    ; --- Stop all outputs from the previous state ---
    clr     temp                ; Clear the temp register (set to 0).
    out     TCCR0B, temp        ; Write 0 to Timer/Counter Control Register B to disconnect the clock and STOP Timer0 (turns off buzzer).
    sbi     PORTC, 2            ; Set PC2 pin HIGH, turning LED1 OFF (assuming common anode wiring).
    sbi     PORTC, 3            ; Set PC3 pin HIGH, turning LED2 OFF.

	wait_for_press:
		rcall   scan_keyboard       ; Call the subroutine to scan the keypad. The result is stored in 'key_code'.
		cpi     key_code, 0         ; Compare the value in 'key_code' with 0. Sets the Zero Flag if they are equal.
		breq    wait_for_press      ; Branch (jump) back to 'wait_for_press' if the Zero Flag is set (i.e., no key was pressed).

		; --- A key has been pressed, decide what to do (IF-ELSEIF-ELSE structure) ---
		cpi     key_code, KEY_7     ; IF the pressed key is KEY_7...
		brne    check_key_8         ; ...if NOT, branch to the next check.
		cbi     PORTC, 2            ; ...if it IS, clear bit PC2 (turn LED1 ON).
		cbi     PORTC, 3            ; ...and clear bit PC3 (turn LED2 ON).
		rcall   wait_for_release    ; Wait for the key to be released to prevent multiple triggers.
		rjmp    main                ; Jump back to the start of the main loop.

	check_key_8:
		cpi     key_code, KEY_8     ; ELSEIF the pressed key is KEY_8...
		brne    check_key_4         ; ...if NOT, branch to the next check.
		cbi     PORTC, 3            ; ...if it IS, clear bit PC3 (turn LED2 ON).
		rcall   wait_for_release    ; Wait for the key to be released.
		rjmp    main                ; Jump back to the start of the main loop.

	check_key_4:
		cpi     key_code, KEY_4     ; ELSEIF the pressed key is KEY_4...
		brne    other_key           ; ...if NOT, branch to the default case.
		cbi     PORTC, 2            ; ...if it IS, clear bit PC2 (turn LED1 ON).
		rcall   wait_for_release    ; Wait for the key to be released.
		rjmp    main                ; Jump back to the start of the main loop.

	other_key:                      ; ELSE (for any other key)...
		ldi     temp, (1<<CS02)     ; Load temp with 0b00000100 to select a clk/256 prescaler for Timer0.
		out     TCCR0B, temp        ; Write the value to the control register, STARTING the timer for the buzzer.
		rcall   wait_for_release    ; Wait for the key to be released.
		rjmp    main                ; Jump back to the start of the main loop.


; --- Wait For Key Release Subroutine ---
; This subroutine polls the keyboard until no keys are pressed. This is a form of debouncing.
wait_for_release:
    rcall   scan_keyboard       ; Scan the keyboard to get the current state.
    cpi     key_code, 0         ; Check if any key is still being held down ('key_code' will be non-zero).
    brne    wait_for_release    ; If NOT equal to 0, a key is still down, so loop back and check again.
    ret                         ; If equal to 0, the key has been released. Return from subroutine.

; --- Keyboard Scanning Subroutine ---
; Scans a 4x4 matrix by grounding one row at a time and reading the column states.
scan_keyboard:
    ldi     key_code, 0         ; Assume no key is pressed initially.

    ; --- Scan Row 1 (PD4) ---
    ldi     temp, 0b11101111    ; Prepare mask to ground Row 1 (PD4) and keep other rows HIGH.
    out     PORTD, temp         ; Apply the mask to activate the row.
    nop                         ; Wait one cycle for the pin voltages to stabilize.
    sbic    PIND, 3             ; Check Column 1 (PD3). If it's LOW (cleared), SKIP the next instruction.
    rjmp    key_1_not_pressed   ; If not pressed, jump to check the next key.
    ldi     key_code, 1         ; If pressed, load the key's code.
    rjmp    key_found           ; Jump to the end since we found the key.
key_1_not_pressed:
    sbic    PIND, 2             ; Check Column 2 (PD2). If LOW, SKIP.
    rjmp    key_2_not_pressed
    ldi     key_code, 2
    rjmp    key_found
key_2_not_pressed:
    sbic    PIND, 1             ; Check Column 3 (PD1). If LOW, SKIP.
    rjmp    key_3_not_pressed
    ldi     key_code, 3
    rjmp    key_found
key_3_not_pressed:
    sbic    PIND, 0             ; Check Column 4 (PD0). If LOW, SKIP.
    rjmp    scan_row_pd5
    ldi     key_code, 4
    rjmp    key_found

    ; --- Scan Row 2 (PD5) ---
scan_row_pd5:
    ldi     temp, 0b11011111    ; Prepare mask to ground Row 2 (PD5).
    out     PORTD, temp
    nop
    sbic    PIND, 3
    rjmp    key_5_not_pressed
    ldi     key_code, 5
    rjmp    key_found
key_5_not_pressed:
    sbic    PIND, 2
    rjmp    key_6_not_pressed
    ldi     key_code, 6
    rjmp    key_found
key_6_not_pressed:
    sbic    PIND, 1
    rjmp    key_7_not_pressed
    ldi     key_code, 7
    rjmp    key_found
key_7_not_pressed:
    sbic    PIND, 0
    rjmp    scan_row_pd6
    ldi     key_code, 8
    rjmp    key_found

    ; --- Scan Row 3 (PD6) ---
scan_row_pd6:
    ldi     temp, 0b10111111    ; Prepare mask to ground Row 3 (PD6).
    out     PORTD, temp
    nop
    sbic    PIND, 3
    rjmp    key_9_not_pressed
    ldi     key_code, 9
    rjmp    key_found
key_9_not_pressed:
    sbic    PIND, 2
    rjmp    key_10_not_pressed
    ldi     key_code, 10
    rjmp    key_found
key_10_not_pressed:
    sbic    PIND, 1
    rjmp    key_11_not_pressed
    ldi     key_code, 11
    rjmp    key_found
key_11_not_pressed:
    sbic    PIND, 0
    rjmp    scan_row_pd7
    ldi     key_code, 12
    rjmp    key_found

    ; --- Scan Row 4 (PD7) ---
scan_row_pd7:
    ldi     temp, 0b01111111    ; Prepare mask to ground Row 4 (PD7).
    out     PORTD, temp
    nop
    sbic    PIND, 3
    rjmp    key_13_not_pressed
    ldi     key_code, 13
    rjmp    key_found
key_13_not_pressed:
    sbic    PIND, 2
    rjmp    key_14_not_pressed
    ldi     key_code, 14
    rjmp    key_found
key_14_not_pressed:
    sbic    PIND, 1
    rjmp    key_15_not_pressed
    ldi     key_code, 15
    rjmp    key_found
key_15_not_pressed:
    sbic    PIND, 0
    rjmp    key_found           ; This actually checks for key 16.
    ldi     key_code, 16        ; Code for key at (Row 4, Column 4)
    rjmp    key_found

key_found:
    ldi     temp, 0b11111111    ; Load temp with all 1s.
    out     PORTD, temp         ; Deactivate all rows and enable all pull-ups, returning the port to its idle state.
    ret                         ; Return from subroutine.

; --- Timer0 Interrupt Service Routine ---
; This code runs automatically when Timer0 overflows. It generates a tone on the buzzer.
Timer0_Overflow_ISR:
    ldi     r16, 185            ; Load the timer reload value.
    out     TCNT0, r16          ; Reload the Timer0 counter to control the tone's frequency (pitch).
    sbi     PINB, 1             ; Toggle the buzzer pin (PB1). Writing a '1' to a PINx register flips the corresponding PORTx bit.
    reti                        ; Return from interrupt and re-enable global interrupts.