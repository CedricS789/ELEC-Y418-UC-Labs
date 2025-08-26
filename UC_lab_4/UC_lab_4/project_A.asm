; ==========================================================
; Project: Piano with Simple Octave Switching (Naturals, 16 keys)
; ==========================================================
.include "m328pdef.inc"         ; Include standard definitions for the ATmega328P.

; --- Definitions ---
.def temp         = r16             ; General-purpose temporary register.
.def note_value   = r17             ; Stores the timer reload value for the current note.
.def key_code     = r19             ; Stores the code of the pressed key.

; --- Hardware Pin Definitions ---
.equ SWITCH_PIN   = PB0            ; Switch for octave selection.

; --- Keypad Mapping Constants (based on scan_keyboard routine) ---
.equ KEY_C     = 13                 ; C4 / C5
.equ KEY_D     = 14                 ; D4 / D5
.equ KEY_E     = 15                 ; E4 / E5
.equ KEY_F     = 16                 ; F4 / F5
.equ KEY_G     = 9                  ; G4 / G5
.equ KEY_A     = 10                 ; A4 / A5
.equ KEY_B     = 11                 ; B4 / B5
.equ KEY_C_H  = 12                 ; C5 / C6

; Add remaining 8 keys as higher naturals (no sharps/flats)
.equ KEY_D5    = 5                  ; D5 / D6
.equ KEY_E5    = 6                  ; E5 / E6
.equ KEY_F5    = 7                  ; F5 / F6
.equ KEY_G5    = 8                  ; G5 / G6
.equ KEY_A5    = 1                  ; A5 / A6
.equ KEY_B5    = 2                  ; B5 / B6
.equ KEY_C6   = 3                  ; C6 / C7
.equ KEY_D6   = 4                  ; D6 / D7

; --- Note Frequency Constants (for prescaler=256) ---
; Base Octave (Switch LOW)
.equ TCNT0_C4 = 137
.equ TCNT0_D4 = 150
.equ TCNT0_E4 = 161
.equ TCNT0_F4 = 166
.equ TCNT0_G4 = 176
.equ TCNT0_A4 = 185
.equ TCNT0_B4 = 193
.equ TCNT0_C5 = 196

; High Octave (Switch HIGH) — your original “_H” set
.equ TCNT0_C5_H = 196
.equ TCNT0_D5_H = 203
.equ TCNT0_E5_H = 209
.equ TCNT0_F5_H = 211              ; Corrected value for F5
.equ TCNT0_G5_H = 216
.equ TCNT0_A5_H = 220
.equ TCNT0_B5_H = 224
.equ TCNT0_C6_H = 226

; Extra higher naturals used by the added 8 keys
.equ TCNT0_D6 = 229
.equ TCNT0_E6 = 232
.equ TCNT0_F6 = 234
.equ TCNT0_G6 = 236
.equ TCNT0_A6 = 238
.equ TCNT0_B6 = 240
.equ TCNT0_C7 = 241
.equ TCNT0_D7 = 243

.org 0x0000                     ; Reset vector.
    rjmp init                   ; Jump to the initialization routine.

.org 0x0020                     ; Timer0 Overflow interrupt vector.
    rjmp Timer0_Overflow_ISR    ; Jump to the interrupt service routine for Timer0.

init:
    ; --- Pin Configuration ---
    sbi     DDRB, 1             ; Configure pin PB1 as an OUTPUT for the buzzer.
    cbi     DDRB, SWITCH_PIN    ; Configure Switch pin PB0 as INPUT.
    sbi     PORTB, SWITCH_PIN   ; Enable pull-up on Switch pin.

    ; --- Keyboard Init ---
    ldi     temp, 0b11110000    ; Set PORTD pins 7-4 as OUTPUTS (rows) and 3-0 as INPUTS (columns).
    out     DDRD, temp
    ldi     temp, 0b11111111    ; Set output rows HIGH and enable pull-ups on input columns.
    out     PORTD, temp

    ; ---- Timer0 and Interrupt Configuration ----
    ldi     temp, (1<<TOIE0)    ; Enable the Timer0 Overflow Interrupt.
    sts     TIMSK0, temp
    sei                         ; Enable global interrupts.

    rjmp    main                ; Jump to the main program loop.

; --- Main Program Loop ---
main:
    ; --- Stop all outputs from the previous state ---
    clr     temp                ; Clear temp register.
    out     TCCR0B, temp        ; Stop Timer0 (disconnects clock, turns off buzzer).

wait_for_press:
    rcall   scan_keyboard       ; Scan the keypad for a pressed key.
    cpi     key_code, 0         ; Check if a key was pressed.
    breq    wait_for_press      ; If no key was pressed, loop back.

    ; --- A key has been pressed, determine which note to play ---
    in      temp, PINB          ; Read switch state
    bst     temp, SWITCH_PIN    ; Store switch state in T-flag

    cpi     key_code, KEY_C
    brne    check_key_d
    brts    play_c5_h           ; If switch is high, play high octave note
    ldi     note_value, TCNT0_C4
    rjmp    play_note
play_c5_h:
    ldi     note_value, TCNT0_C5_H
    rjmp    play_note

check_key_d:
    cpi     key_code, KEY_D
    brne    check_key_e
    brts    play_d5_h
    ldi     note_value, TCNT0_D4
    rjmp    play_note
play_d5_h:
    ldi     note_value, TCNT0_D5_H
    rjmp    play_note

check_key_e:
    cpi     key_code, KEY_E
    brne    check_key_f
    brts    play_e5_h
    ldi     note_value, TCNT0_E4
    rjmp    play_note
play_e5_h:
    ldi     note_value, TCNT0_E5_H
    rjmp    play_note

check_key_f:
    cpi     key_code, KEY_F
    brne    check_key_g
    brts    play_f5_h
    ldi     note_value, TCNT0_F4
    rjmp    play_note
play_f5_h:
    ldi     note_value, TCNT0_F5_H
    rjmp    play_note

check_key_g:
    cpi     key_code, KEY_G
    brne    check_key_a
    brts    play_g5_h
    ldi     note_value, TCNT0_G4
    rjmp    play_note
play_g5_h:
    ldi     note_value, TCNT0_G5_H
    rjmp    play_note

check_key_a:
    cpi     key_code, KEY_A
    brne    check_key_b
    brts    play_a5_h
    ldi     note_value, TCNT0_A4
    rjmp    play_note
play_a5_h:
    ldi     note_value, TCNT0_A5_H
    rjmp    play_note

check_key_b:
    cpi     key_code, KEY_B
    brne    check_KEY_C_H
    brts    play_b5_h
    ldi     note_value, TCNT0_B4
    rjmp    play_note
play_b5_h:
    ldi     note_value, TCNT0_B5_H
    rjmp    play_note

check_KEY_C_H:
    cpi     key_code, KEY_C_H
    brne    check_key_d5        ; continue to higher naturals
    brts    play_c6_h
    ldi     note_value, TCNT0_C5
    rjmp    play_note
play_c6_h:
    ldi     note_value, TCNT0_C6_H
    rjmp    play_note

; --- Remaining 8 keys: keep climbing in naturals ---
check_key_d5:
    cpi     key_code, KEY_D5
    brne    check_key_e5
    brts    play_d6
    ldi     note_value, TCNT0_D5_H     ; LOW ? D5
    rjmp    play_note
play_d6:
    ldi     note_value, TCNT0_D6       ; HIGH ? D6
    rjmp    play_note

check_key_e5:
    cpi     key_code, KEY_E5
    brne    check_key_f5
    brts    play_e6
    ldi     note_value, TCNT0_E5_H     ; LOW ? E5
    rjmp    play_note
play_e6:
    ldi     note_value, TCNT0_E6       ; HIGH ? E6
    rjmp    play_note

check_key_f5:
    cpi     key_code, KEY_F5
    brne    check_key_g5
    brts    play_f6
    ldi     note_value, TCNT0_F5_H     ; LOW ? F5
    rjmp    play_note
play_f6:
    ldi     note_value, TCNT0_F6       ; HIGH ? F6
    rjmp    play_note

check_key_g5:
    cpi     key_code, KEY_G5
    brne    check_key_a5
    brts    play_g6
    ldi     note_value, TCNT0_G5_H     ; LOW ? G5
    rjmp    play_note
play_g6:
    ldi     note_value, TCNT0_G6       ; HIGH ? G6
    rjmp    play_note

check_key_a5:
    cpi     key_code, KEY_A5
    brne    check_key_b5
    brts    play_a6
    ldi     note_value, TCNT0_A5_H     ; LOW ? A5
    rjmp    play_note
play_a6:
    ldi     note_value, TCNT0_A6       ; HIGH ? A6
    rjmp    play_note

check_key_b5:
    cpi     key_code, KEY_B5
    brne    check_KEY_C6
    brts    play_b6
    ldi     note_value, TCNT0_B5_H     ; LOW ? B5
    rjmp    play_note
play_b6:
    ldi     note_value, TCNT0_B6       ; HIGH ? B6
    rjmp    play_note

check_KEY_C6:
    cpi     key_code, KEY_C6
    brne    check_KEY_D6
    brts    play_c7
    ldi     note_value, TCNT0_C6_H     ; LOW ? C6
    rjmp    play_note
play_c7:
    ldi     note_value, TCNT0_C7       ; HIGH ? C7
    rjmp    play_note

check_KEY_D6:
    cpi     key_code, KEY_D6
    brne    other_key_pressed
    brts    play_d7
    ldi     note_value, TCNT0_D6       ; LOW ? D6
    rjmp    play_note
play_d7:
    ldi     note_value, TCNT0_D7       ; HIGH ? D7
    rjmp    play_note

play_note:
    ldi     temp, (1<<CS02)     ; Start Timer0 with a clk/256 prescaler.
    out     TCCR0B, temp
    rcall   wait_for_release    ; Wait for the key to be released.
    rjmp    main                ; Return to the main loop.

other_key_pressed:
    rcall   wait_for_release    ; Key is not a note, just wait for release.
    rjmp    main

; --- Wait For Key Release Subroutine ---
wait_for_release:
    rcall   scan_keyboard       ; Scan the keyboard.
    cpi     key_code, 0         ; Check if a key is still pressed.
    brne    wait_for_release    ; Loop until the key is released.
    ret                         ; Return from subroutine.

; --- Keyboard Scanning Subroutine ---
; Scans a 4x4 matrix keypad.
scan_keyboard:
    ldi     key_code, 0         ; Initialize key_code to 0 (no key pressed).

    ; --- Scan Row 1 (PD4) ---
    ldi     temp, 0b11101111
    out     PORTD, temp
    nop
    sbic    PIND, 3
    rjmp    key_1_not_pressed
    ldi     key_code, 1
    rjmp    key_found
key_1_not_pressed:
    sbic    PIND, 2
    rjmp    key_2_not_pressed
    ldi     key_code, 2
    rjmp    key_found
key_2_not_pressed:
    sbic    PIND, 1
    rjmp    key_3_not_pressed
    ldi     key_code, 3
    rjmp    key_found
key_3_not_pressed:
    sbic    PIND, 0
    rjmp    scan_row_pd5
    ldi     key_code, 4
    rjmp    key_found

    ; --- Scan Row 2 (PD5) ---
scan_row_pd5:
    ldi     temp, 0b11011111
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
    ldi     temp, 0b10111111
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
    ldi     temp, 0b01111111
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
    rjmp    key_found
    ldi     key_code, 16
    rjmp    key_found

key_found:
    ldi     temp, 0b11111111    ; Deactivate all rows and enable pull-ups.
    out     PORTD, temp
    ret                         ; Return from subroutine.

; --- Timer0 Interrupt Service Routine ---
Timer0_Overflow_ISR:
    push    temp                ; Save temp register (r16)
    in      temp, SREG          ; Save SREG
    push    temp

    out     TCNT0, note_value   ; Reload Timer0 counter with the current note's frequency value.
    sbi     PINB, 1             ; Toggle the buzzer pin (PB1).

    pop     temp                ; Restore SREG
    out     SREG, temp
    pop     temp                ; Restore temp register (r16)
    reti                        ; Return from interrupt.
