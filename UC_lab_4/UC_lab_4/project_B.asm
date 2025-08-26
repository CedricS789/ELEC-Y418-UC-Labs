; ============================================================================
; Combined Mini Piano with LED Matrix Display
; Target: ATmega328P (Arduino Uno compatible)
;
; Features:
; - Scans 4x4 keyboard for 16 natural notes (C to D across two octaves per switch position).
; - Plays note on buzzer using Timer0 overflow for tone generation.
; - Octave shift via switch on PB0 (LOW: base octave, HIGH: +1 octave).
; - Displays the note letter (A-G) on the LED matrix in a single 5x7 block.
; - Display refreshes via Timer2 (~3kHz row scan).
; - Note display clears when key is released.
; ============================================================================

.include "m328pdef.inc"

; =============================================================================
; CONFIGURATION CONSTANTS
; =============================================================================

; --- Port B pins: Shift chain for display + Buzzer + Switch ---
.equ SR_DATA_PIN    = 3          ; PB3 ? Serial data
.equ SR_CLOCK_PIN   = 5          ; PB5 ? SR clock
.equ SR_LATCH_PIN   = 4          ; PB4 ? Latch/strobe
.equ BUZZER_PIN     = 1          ; PB1 ? Buzzer output
.equ SWITCH_PIN     = 0          ; PB0 ? Octave switch input

.equ SR_OUTPUT_MASK = (1<<SR_DATA_PIN)|(1<<SR_LATCH_PIN)|(1<<SR_CLOCK_PIN)|(1<<BUZZER_PIN)
.equ SR_PORT_INIT   = (1<<SR_DATA_PIN)|(1<<SR_LATCH_PIN)|(1<<SR_CLOCK_PIN)|(1<<SWITCH_PIN)  ; Pull-up on switch

; --- Timers ---
.equ T2_PRELOAD_4KHZ = 251       ; ~3.125 kHz actual with presc=1024
.equ TIMER2_PRESCALER = (1<<CS22)|(1<<CS21)|(1<<CS20)  ; 1024

; --- Display geometry ---
.equ MATRIX_ROWS      = 7        ; 7 active rows
.equ MATRIX_COLS      = 5        ; 5 cols per character row
.equ COLUMN_CYCLES    = 16       ; 16 cascaded 5x7 blocks
.equ TARGET_BLOCK     = 15       ; Block to display in (0..15)

; --- Keypad Mapping (from scan_keyboard) ---
.equ KEY_C     = 13              ; C
.equ KEY_D     = 14              ; D
.equ KEY_E     = 15              ; E
.equ KEY_F     = 16              ; F
.equ KEY_G     = 9               ; G
.equ KEY_A     = 10              ; A
.equ KEY_B     = 11              ; B
.equ KEY_C_H   = 12              ; C (higher)
.equ KEY_D5    = 5               ; D (higher)
.equ KEY_E5    = 6               ; E (higher)
.equ KEY_F5    = 7               ; F (higher)
.equ KEY_G5    = 8               ; G (higher)
.equ KEY_A5    = 1               ; A (higher)
.equ KEY_B5    = 2               ; B (higher)
.equ KEY_C6    = 3               ; C (higher)
.equ KEY_D6    = 4               ; D (higher)

; --- Note Frequency Preloads (clk/256 prescaler) ---
.equ TCNT0_C4 = 137
.equ TCNT0_D4 = 150
.equ TCNT0_E4 = 161
.equ TCNT0_F4 = 166
.equ TCNT0_G4 = 176
.equ TCNT0_A4 = 185
.equ TCNT0_B4 = 193
.equ TCNT0_C5 = 196
.equ TCNT0_D5_H = 203
.equ TCNT0_E5_H = 209
.equ TCNT0_F5_H = 211
.equ TCNT0_G5_H = 216
.equ TCNT0_A5_H = 220
.equ TCNT0_B5_H = 224
.equ TCNT0_C6_H = 226
.equ TCNT0_D6 = 229
.equ TCNT0_E6 = 232
.equ TCNT0_F6 = 234
.equ TCNT0_G6 = 236
.equ TCNT0_A6 = 238
.equ TCNT0_B6 = 240
.equ TCNT0_C7 = 241
.equ TCNT0_D7 = 243

; --- Note Indices (0=C, 1=D, 2=E, 3=F, 4=G, 5=A, 6=B) ---
.equ NOTE_IDX_C = 0
.equ NOTE_IDX_D = 1
.equ NOTE_IDX_E = 2
.equ NOTE_IDX_F = 3
.equ NOTE_IDX_G = 4
.equ NOTE_IDX_A = 5
.equ NOTE_IDX_B = 6
.equ NOTE_IDX_NONE = 255        ; No note (blank display)

; --- Register Definitions ---
.def temp           = r16       ; General temp
.def note_value     = r17       ; Timer0 preload for note
.def key_code       = r19       ; Pressed key code
.def current_note_idx = r23     ; Current note index for display (0-6 or 255)

; =============================================================================
; MACROS
; =============================================================================

; Shift one bit from reg @0, bit @1
.MACRO SHIFT_BIT_OUT
    sbi   PORTB, SR_DATA_PIN
    sbrs  @0, @1
    cbi   PORTB, SR_DATA_PIN
    sbi   PINB, SR_CLOCK_PIN
    sbi   PINB, SR_CLOCK_PIN
.ENDMACRO

; =============================================================================
; VECTORS
; =============================================================================
.cseg
.org 0x0000
    rjmp  init                  ; Reset

.org 0x0012
    rjmp  timer2_refresh_isr    ; Timer2 OVF (display refresh)

.org 0x0020
    rjmp  Timer0_Overflow_ISR   ; Timer0 OVF (buzzer tone)

; =============================================================================
; INIT
; =============================================================================
init:
    ; Stack
    ldi   temp, high(RAMEND)
    out   SPH, temp
    ldi   temp, low(RAMEND)
    out   SPL, temp

    ; Zero reg (ABI)
    clr   r1

    ; Port B: Display shift + Buzzer out + Switch in/pull-up
    ldi   temp, SR_OUTPUT_MASK
    out   DDRB, temp
    ldi   temp, SR_PORT_INIT
    out   PORTB, temp

    ; Keyboard: PD7-4 out, PD3-0 in/pull-up
    ldi   temp, 0b11110000
    out   DDRD, temp
    ldi   temp, 0b11111111
    out   PORTD, temp

    ; Timers
    rcall setup_timer2_refresh
    ldi   temp, (1<<TOIE0)      ; Enable Timer0 OVF interrupt (for buzzer)
    sts   TIMSK0, temp

    ; Display state
    rcall init_display_state

    ; No initial note
    ldi   current_note_idx, NOTE_IDX_NONE

    sei                         ; Global interrupts
    rjmp  main                  ; To main loop

setup_timer2_refresh:
    lds   temp, TCCR2A
    andi  temp, ~((1<<WGM21)|(1<<WGM20))
    sts   TCCR2A, temp

    lds   temp, TCCR2B
    andi  temp, ~(1<<WGM22)
    ori   temp, TIMER2_PRESCALER
    sts   TCCR2B, temp

    lds   temp, TIMSK2
    ori   temp, (1<<TOIE2)
    sts   TIMSK2, temp

    ldi   temp, T2_PRELOAD_4KHZ
    sts   TCNT2, temp
    ret

init_display_state:
    ldi   r22, MATRIX_ROWS - 1  ; Row index (6..0)
    ldi   r21, 0b01000000       ; One-hot row select (MSB=row6)
    ret

; =============================================================================
; FONT (5x7, bits 4:0; bit4=left LED?)
; =============================================================================
characters:
char_A:  .db 0b00110, 0b01001, 0b01001, 0b01001, 0b01111, 0b01001, 0b01001, 0
char_B:  .db 0b01110, 0b01001, 0b01001, 0b01110, 0b01001, 0b01001, 0b01110, 0
char_C:  .db 0b00111, 0b01000, 0b01000, 0b01000, 0b01000, 0b01000, 0b00111, 0
char_D:  .db 0b01110, 0b01001, 0b01001, 0b01001, 0b01001, 0b01001, 0b01110, 0
char_E:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01111, 0
char_F:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01000, 0
char_G:  .db 0b00111, 0b01000, 0b01000, 0b01011, 0b01001, 0b01001, 0b00110, 0  ; Added G
char_space: .db 0, 0, 0, 0, 0, 0, 0, 0  ; Blank

; Note index to font pointer table (word addresses *2 for LPM)
note_table:
    .dw char_C*2, char_D*2, char_E*2, char_F*2, char_G*2, char_A*2, char_B*2

; =============================================================================
; MAIN LOOP (Piano Logic)
; =============================================================================
main:
    clr   temp
    out   TCCR0B, temp          ; Stop Timer0 (buzzer off)

wait_for_press:
    rcall scan_keyboard         ; Scan for key
    cpi   key_code, 0
    breq  wait_for_press        ; Loop if none

    ; Key pressed: Read switch for octave
    in    temp, PINB
    bst   temp, SWITCH_PIN      ; T-flag = switch state

    ; Map key to note preload and index
    cpi   key_code, KEY_C
    brne  check_key_d
    ldi   current_note_idx, NOTE_IDX_C
    brts  play_c5_h
    ldi   note_value, TCNT0_C4
    rjmp  play_note
play_c5_h:
    ldi   note_value, TCNT0_C5
    rjmp  play_note

check_key_d:
    cpi   key_code, KEY_D
    brne  check_key_e
    ldi   current_note_idx, NOTE_IDX_D
    brts  play_d5_h
    ldi   note_value, TCNT0_D4
    rjmp  play_note
play_d5_h:
    ldi   note_value, TCNT0_D5_H
    rjmp  play_note

check_key_e:
    cpi   key_code, KEY_E
    brne  check_key_f
    ldi   current_note_idx, NOTE_IDX_E
    brts  play_e5_h
    ldi   note_value, TCNT0_E4
    rjmp  play_note
play_e5_h:
    ldi   note_value, TCNT0_E5_H
    rjmp  play_note

check_key_f:
    cpi   key_code, KEY_F
    brne  check_key_g
    ldi   current_note_idx, NOTE_IDX_F
    brts  play_f5_h
    ldi   note_value, TCNT0_F4
    rjmp  play_note
play_f5_h:
    ldi   note_value, TCNT0_F5_H
    rjmp  play_note

check_key_g:
    cpi   key_code, KEY_G
    brne  check_key_a
    ldi   current_note_idx, NOTE_IDX_G
    brts  play_g5_h
    ldi   note_value, TCNT0_G4
    rjmp  play_note
play_g5_h:
    ldi   note_value, TCNT0_G5_H
    rjmp  play_note

check_key_a:
    cpi   key_code, KEY_A
    brne  check_key_b
    ldi   current_note_idx, NOTE_IDX_A
    brts  play_a5_h
    ldi   note_value, TCNT0_A4
    rjmp  play_note
play_a5_h:
    ldi   note_value, TCNT0_A5_H
    rjmp  play_note

check_key_b:
    cpi   key_code, KEY_B
    brne  check_key_c_h
    ldi   current_note_idx, NOTE_IDX_B
    brts  play_b5_h
    ldi   note_value, TCNT0_B4
    rjmp  play_note
play_b5_h:
    ldi   note_value, TCNT0_B5_H
    rjmp  play_note

check_key_c_h:
    cpi   key_code, KEY_C_H
    brne  check_key_d5
    ldi   current_note_idx, NOTE_IDX_C
    brts  play_c6_h
    ldi   note_value, TCNT0_C5
    rjmp  play_note
play_c6_h:
    ldi   note_value, TCNT0_C6_H
    rjmp  play_note

check_key_d5:
    cpi   key_code, KEY_D5
    brne  check_key_e5
    ldi   current_note_idx, NOTE_IDX_D
    brts  play_d6
    ldi   note_value, TCNT0_D5_H
    rjmp  play_note
play_d6:
    ldi   note_value, TCNT0_D6
    rjmp  play_note

check_key_e5:
    cpi   key_code, KEY_E5
    brne  check_key_f5
    ldi   current_note_idx, NOTE_IDX_E
    brts  play_e6
    ldi   note_value, TCNT0_E5_H
    rjmp  play_note
play_e6:
    ldi   note_value, TCNT0_E6
    rjmp  play_note

check_key_f5:
    cpi   key_code, KEY_F5
    brne  check_key_g5
    ldi   current_note_idx, NOTE_IDX_F
    brts  play_f6
    ldi   note_value, TCNT0_F5_H
    rjmp  play_note
play_f6:
    ldi   note_value, TCNT0_F6
    rjmp  play_note

check_key_g5:
    cpi   key_code, KEY_G5
    brne  check_key_a5
    ldi   current_note_idx, NOTE_IDX_G
    brts  play_g6
    ldi   note_value, TCNT0_G5_H
    rjmp  play_note
play_g6:
    ldi   note_value, TCNT0_G6
    rjmp  play_note

check_key_a5:
    cpi   key_code, KEY_A5
    brne  check_key_b5
    ldi   current_note_idx, NOTE_IDX_A
    brts  play_a6
    ldi   note_value, TCNT0_A5_H
    rjmp  play_note
play_a6:
    ldi   note_value, TCNT0_A6
    rjmp  play_note

check_key_b5:
    cpi   key_code, KEY_B5
    brne  check_key_c6
    ldi   current_note_idx, NOTE_IDX_B
    brts  play_b6
    ldi   note_value, TCNT0_B5_H
    rjmp  play_note
play_b6:
    ldi   note_value, TCNT0_B6
    rjmp  play_note

check_key_c6:
    cpi   key_code, KEY_C6
    brne  check_key_d6
    ldi   current_note_idx, NOTE_IDX_C
    brts  play_c7
    ldi   note_value, TCNT0_C6_H
    rjmp  play_note
play_c7:
    ldi   note_value, TCNT0_C7
    rjmp  play_note

check_key_d6:
    cpi   key_code, KEY_D6
    brne  other_key_pressed
    ldi   current_note_idx, NOTE_IDX_D
    brts  play_d7
    ldi   note_value, TCNT0_D6
    rjmp  play_note
play_d7:
    ldi   note_value, TCNT0_D7
    rjmp  play_note

play_note:
    ldi   temp, (1<<CS02)       ; Start Timer0 (clk/256)
    out   TCCR0B, temp
    rcall wait_for_release      ; Hold until released
    ldi   current_note_idx, NOTE_IDX_NONE  ; Clear display
    rjmp  main

other_key_pressed:
    rcall wait_for_release
    rjmp  main

; =============================================================================
; SUBROUTINES
; =============================================================================

wait_for_release:
    rcall scan_keyboard
    cpi   key_code, 0
    brne  wait_for_release
    ret

scan_keyboard:
    clr   key_code              ; Default: no key

    ; Row 1 (PD4 low)
    ldi   temp, 0b11101111
    out   PORTD, temp
    nop
    sbic  PIND, 3
    rjmp  k1_np
    ldi   key_code, 1
    rjmp  key_found
k1_np:
    sbic  PIND, 2
    rjmp  k2_np
    ldi   key_code, 2
    rjmp  key_found
k2_np:
    sbic  PIND, 1
    rjmp  k3_np
    ldi   key_code, 3
    rjmp  key_found
k3_np:
    sbic  PIND, 0
    rjmp  scan_row2
    ldi   key_code, 4
    rjmp  key_found

scan_row2:  ; Row 2 (PD5 low)
    ldi   temp, 0b11011111
    out   PORTD, temp
    nop
    sbic  PIND, 3
    rjmp  k5_np
    ldi   key_code, 5
    rjmp  key_found
k5_np:
    sbic  PIND, 2
    rjmp  k6_np
    ldi   key_code, 6
    rjmp  key_found
k6_np:
    sbic  PIND, 1
    rjmp  k7_np
    ldi   key_code, 7
    rjmp  key_found
k7_np:
    sbic  PIND, 0
    rjmp  scan_row3
    ldi   key_code, 8
    rjmp  key_found

scan_row3:  ; Row 3 (PD6 low)
    ldi   temp, 0b10111111
    out   PORTD, temp
    nop
    sbic  PIND, 3
    rjmp  k9_np
    ldi   key_code, 9
    rjmp  key_found
k9_np:
    sbic  PIND, 2
    rjmp  k10_np
    ldi   key_code, 10
    rjmp  key_found
k10_np:
    sbic  PIND, 1
    rjmp  k11_np
    ldi   key_code, 11
    rjmp  key_found
k11_np:
    sbic  PIND, 0
    rjmp  scan_row4
    ldi   key_code, 12
    rjmp  key_found

scan_row4:  ; Row 4 (PD7 low)
    ldi   temp, 0b01111111
    out   PORTD, temp
    nop
    sbic  PIND, 3
    rjmp  k13_np
    ldi   key_code, 13
    rjmp  key_found
k13_np:
    sbic  PIND, 2
    rjmp  k14_np
    ldi   key_code, 14
    rjmp  key_found
k14_np:
    sbic  PIND, 1
    rjmp  k15_np
    ldi   key_code, 15
    rjmp  key_found
k15_np:
    sbic  PIND, 0
    rjmp  key_found
    ldi   key_code, 16

key_found:
    ldi   temp, 0b11111111      ; Reset rows/pull-ups
    out   PORTD, temp
    ret

; =============================================================================
; ISRs
; =============================================================================

Timer0_Overflow_ISR:  ; Buzzer tone
    push  temp
    in    temp, SREG
    push  temp

    out   TCNT0, note_value     ; Reload
    sbi   PINB, BUZZER_PIN      ; Toggle buzzer

    pop   temp
    out   SREG, temp
    pop   temp
    reti

timer2_refresh_isr:  ; Display refresh (one row)
    push  r16
    in    r16, SREG
    push  r16
    push  r17
    push  r18
    push  r19
    push  r20

    ; Steady refresh
    ldi   r16, T2_PRELOAD_4KHZ
    sts   TCNT2, r16

    ; --- Fetch row bits based on current note index ---
    cpi   current_note_idx, 7
    brsh  blank_row

    mov   r18, current_note_idx
    lsl   r18                   ; *2 for table offset
    ldi   ZL, low(note_table*2)
    ldi   ZH, high(note_table*2)
    add   ZL, r18
    adc   ZH, r1
    lpm   r16, Z+
    lpm   r17, Z                ; r16/17 = char_* *2
    mov   ZL, r16
    mov   ZH, r17

    cpi   r22, MATRIX_ROWS
    brlo  row_ok
    ldi   r22, MATRIX_ROWS - 1
row_ok:
    add   ZL, r22
    adc   ZH, r1
    lpm   r20, Z                ; r20 = row pattern
    rjmp  row_fetched

blank_row:
    clr   r20                   ; All LEDs off

row_fetched:
    ; --- Send columns (only TARGET_BLOCK gets pattern) ---
    ldi   r18, COLUMN_CYCLES    ; Blocks loop
    clr   r19                   ; Block index

column_shift_loop:
    cpi   r19, TARGET_BLOCK
    brne  send_zeros

send_pattern:
    SHIFT_BIT_OUT r20, 0
    SHIFT_BIT_OUT r20, 1
    SHIFT_BIT_OUT r20, 2
    SHIFT_BIT_OUT r20, 3
    SHIFT_BIT_OUT r20, 4
    rjmp  after_bits

send_zeros:
    SHIFT_BIT_OUT r1, 0         ; r1=0
    SHIFT_BIT_OUT r1, 1
    SHIFT_BIT_OUT r1, 2
    SHIFT_BIT_OUT r1, 3
    SHIFT_BIT_OUT r1, 4

after_bits:
    inc   r19
    dec   r18
    brne  column_shift_loop

    ; --- Row select (7 bits, one-hot) + separator ---
    cbi   PORTB, SR_DATA_PIN    ; Separator 0
    sbi   PINB, SR_CLOCK_PIN
    sbi   PINB, SR_CLOCK_PIN

    SHIFT_BIT_OUT r21, 6
    SHIFT_BIT_OUT r21, 5
    SHIFT_BIT_OUT r21, 4
    SHIFT_BIT_OUT r21, 3
    SHIFT_BIT_OUT r21, 2
    SHIFT_BIT_OUT r21, 1
    SHIFT_BIT_OUT r21, 0

    ; --- Advance row ---
    dec   r22
    lsr   r21
    brne  trigger_strobe

    ldi   r22, MATRIX_ROWS - 1
    ldi   r21, 0b01000000

trigger_strobe:
    sbi   PINB, SR_LATCH_PIN    ; Toggle latch (start pulse)
    nop
    nop                         ; Short delay (~125ns @16MHz)
    sbi   PINB, SR_LATCH_PIN    ; Toggle back (end pulse)

    ; Restore
    pop   r20
    pop   r19
    pop   r18
    pop   r17
    pop   r16
    out   SREG, r16
    pop   r16
    reti

; =============================================================================
; END
; =============================================================================