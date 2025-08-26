; ============================================================================
; Enhanced LED Matrix Display Driver (single-block output)
; Target: ATmega328P (Arduino Uno compatible)
;
; Shift-register matrix with row scanning.
; Shows the character in ONLY ONE 5×7 block (upper-left / first-out).
; Change TARGET_BLOCK to pick another block (0..15).
; ============================================================================

.include "m328pdef.inc"

; =============================================================================
; CONFIGURATION CONSTANTS
; =============================================================================

; --- Port B pins driving the shift chain ---
.equ SR_DDR         = DDRB
.equ SR_PORT        = PORTB
.equ SR_PINREG      = PINB
.equ SR_DATA_PIN    = 3          ; PB3 ? Serial data
.equ SR_CLOCK_PIN   = 5          ; PB5 ? SR clock
.equ SR_LATCH_PIN   = 4          ; PB4 ? Latch/strobe

; --- Timers ---
.equ T2_PRELOAD_4KHZ = 251       ; ~3.125 kHz actual with presc=1024 (close enough)
.equ T0_STROBE_DELAY = 240       ; short latch pulse

; --- Display geometry ---
.equ MATRIX_ROWS      = 7        ; 7 active rows
.equ MATRIX_COLS      = 5        ; 5 cols per character row
.equ char_BYTES       = 8        ; 7 rows + 1 pad
.equ COLUMN_CYCLES    = 16       ; 16 cascaded 5×7 blocks

; --- Pick which block to light (0..15). Use 15 if your chain is reversed. ---
.equ TARGET_BLOCK     = 15

; --- Bit masks / prescalers ---
.equ SR_OUTPUT_MASK   = (1<<SR_DATA_PIN)|(1<<SR_LATCH_PIN)|(1<<SR_CLOCK_PIN)
.equ TIMER2_PRESCALER = (1<<CS22)|(1<<CS21)|(1<<CS20)  ; 1024
.equ TIMER0_PRESCALER = (1<<CS02)|(1<<CS01)|(1<<CS00)  ; 1024

; =============================================================================
; MACROS
; =============================================================================

; Emit one bit from register @0, bit @1 into the shift chain
.MACRO SHIFT_BIT_OUT
    sbi   SR_PORT, SR_DATA_PIN          ; assume '1'
    sbrs  @0, @1                        ; if bit is 0...
    cbi   SR_PORT, SR_DATA_PIN          ; ...drive '0'
    sbi   SR_PINREG, SR_CLOCK_PIN       ; clock ? (PIN write toggles)
    sbi   SR_PINREG, SR_CLOCK_PIN       ; clock ?
.ENDMACRO

; =============================================================================
; VECTORS
; =============================================================================
.cseg
.org 0x0000
    rjmp  system_init                   ; Reset

.org 0x0010                              ; (kept same positions as user’s code)
    rjmp  timer0_strobe_isr             ; Timer0 overflow

.org 0x0012
    rjmp  timer2_refresh_isr            ; Timer2 overflow

; =============================================================================
; INIT
; =============================================================================
system_init:
    ; Stack
    ldi   r16, high(RAMEND)
    out   SPH, r16
    ldi   r16, low(RAMEND)
    out   SPL, r16

    ; Keep r1 zero per ABI (also used for quick zero bits)
    clr   r1

    ; Shift-register pins as outputs, idle high
    ldi   r17, SR_OUTPUT_MASK
    out   SR_DDR,  r17
    out   SR_PORT, r17

    ; Timers
    rcall setup_timer2_refresh
    rcall setup_timer0_strobe

    ; Display state (start at top row)
    rcall init_display_state

    sei
main_loop:
    nop
    rjmp  main_loop

; --- Timer2: refresh ---
setup_timer2_refresh:
    lds   r16, TCCR2A
    andi  r16, ~((1<<WGM21)|(1<<WGM20))
    sts   TCCR2A, r16

    lds   r16, TCCR2B
    andi  r16, ~(1<<WGM22)
    ori   r16, TIMER2_PRESCALER
    sts   TCCR2B, r16

    lds   r16, TIMSK2
    ori   r16, (1<<TOIE2)
    sts   TIMSK2, r16

    ldi   r16, T2_PRELOAD_4KHZ
    sts   TCNT2, r16
    ret

; --- Timer0: strobe ---
setup_timer0_strobe:
    lds   r16, TCCR0A
    andi  r16, ~((1<<WGM01)|(1<<WGM00))
    sts   TCCR0A, r16

    lds   r16, TCCR0B
    andi  r16, ~((1<<WGM02)|(1<<CS02)|(1<<CS01)|(1<<CS00)) ; stopped
    sts   TCCR0B, r16

    lds   r16, TIMSK0
    ori   r16, (1<<TOIE0)
    sts   TIMSK0, r16
    ret

; --- Display state ---
init_display_state:
    ldi   r22, MATRIX_ROWS - 1          ; row index (0..6) counting downward
    ldi   r21, 0b01000000               ; one-hot row select, MSB = row 6
    ret

; =============================================================================
; FONT (5×7, bits 0..4 used)
; =============================================================================
characters:
char_A:  .db 0b00110, 0b01001, 0b01001, 0b01001, 0b01111, 0b01001, 0b01001, 0
char_B:  .db 0b01110, 0b01001, 0b01001, 0b01110, 0b01001, 0b01001, 0b01110, 0
char_C:  .db 0b00111, 0b01000, 0b01000, 0b01000, 0b01000, 0b01000, 0b00111, 0
char_D:  .db 0b01110, 0b01001, 0b01001, 0b01001, 0b01001, 0b01001, 0b01110, 0
char_E:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01111, 0
char_F:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01000, 0

char_1:  .db 0b00010, 0b00110, 0b01010, 0b00010, 0b00010, 0b00010, 0b00010, 0
char_2:  .db 0b01111, 0b00001, 0b00001, 0b01111, 0b01000, 0b01000, 0b01111, 0
char_3:  .db 0b01111, 0b00001, 0b00001, 0b00111, 0b00001, 0b00001, 0b01111, 0
char_4:  .db 0b01001, 0b01001, 0b01001, 0b01111, 0b00001, 0b00001, 0b00001, 0
char_5:  .db 0b01111, 0b01000, 0b01000, 0b01111, 0b00001, 0b00001, 0b01111, 0
char_6:  .db 0b01111, 0b01000, 0b01000, 0b01111, 0b01001, 0b01001, 0b01111, 0
char_7:  .db 0b01111, 0b00001, 0b00001, 0b00010, 0b00100, 0b00100, 0b00100, 0
char_8:  .db 0b01111, 0b01001, 0b01001, 0b01111, 0b01001, 0b01001, 0b01111, 0
char_9:  .db 0b01111, 0b01001, 0b01001, 0b01111, 0b00001, 0b00001, 0b01111, 0
char_0:  .db 0b01111, 0b01001, 0b01001, 0b01001, 0b01001, 0b01001, 0b01111, 0

; =============================================================================
; ISRs
; =============================================================================

; --- Timer0 ISR: completes the latch pulse and stops T0 ---
timer0_strobe_isr:
    ldi   r16, T0_STROBE_DELAY
    sts   TCNT0, r16
    sbi   SR_PINREG, SR_LATCH_PIN       ; toggle latch (end pulse)

    lds   r16, TCCR0B
    andi  r16, ~((1<<CS02)|(1<<CS01)|(1<<CS00)) ; stop timer
    sts   TCCR0B, r16
    reti

; --- Timer2 ISR: push one row to the chain (only one block gets data) ---
timer2_refresh_isr:
    ; Save context
    push  r16
    in    r16, SREG
    push  r16
    push  r17
    push  r18
    push  r19
    push  r20

    ; Keep steady refresh
    ldi   r16, T2_PRELOAD_4KHZ
    sts   TCNT2, r16

    ; -------- Fetch row bits for chosen glyph (example: '7') --------
    ldi   ZL, low(char_7 * 2)           ; flash byte address
    ldi   ZH, high(char_7 * 2)

    cpi   r22, MATRIX_ROWS
    brlo  row_ok
    ldi   r22, MATRIX_ROWS - 1
row_ok:
    add   ZL, r22
    adc   ZH, r1                        ; r1 = 0
    lpm   r20, Z                        ; r20 = 5-bit pattern for this row

    ; -------- Send columns for 16 blocks; only TARGET_BLOCK gets r20 --------
    ldi   r18, COLUMN_CYCLES            ; remaining blocks
    clr   r19                           ; current block index 0..15

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
    ; r1 is 0 ? emits 0 five times
    SHIFT_BIT_OUT r1, 0
    SHIFT_BIT_OUT r1, 1
    SHIFT_BIT_OUT r1, 2
    SHIFT_BIT_OUT r1, 3
    SHIFT_BIT_OUT r1, 4

after_bits:
    inc   r19
    dec   r18
    brne  column_shift_loop

    ; -------- Row select shift (7 bits, one-hot) --------
    cbi   SR_PORT, SR_DATA_PIN          ; separator bit
    sbi   SR_PINREG, SR_CLOCK_PIN
    sbi   SR_PINREG, SR_CLOCK_PIN

    SHIFT_BIT_OUT r21, 6
    SHIFT_BIT_OUT r21, 5
    SHIFT_BIT_OUT r21, 4
    SHIFT_BIT_OUT r21, 3
    SHIFT_BIT_OUT r21, 2
    SHIFT_BIT_OUT r21, 1
    SHIFT_BIT_OUT r21, 0

    ; -------- Advance to next row --------
    dec   r22
    lsr   r21
    brne  trigger_strobe

    ldi   r22, MATRIX_ROWS - 1
    ldi   r21, 0b01000000

trigger_strobe:
    sbi   SR_PINREG, SR_LATCH_PIN       ; begin latch pulse
    lds   r16, TCCR0B
    ori   r16, TIMER0_PRESCALER         ; start Timer0 to end pulse
    sts   TCCR0B, r16

    ; Restore context
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
