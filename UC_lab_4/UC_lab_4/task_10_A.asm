;
; Enhanced LED Matrix Display Driver
; Target: ATmega328P (Arduino Uno compatible)
; 
; Features:
; - Shift-register based LED matrix control
; - Timer2: ~480Hz row refresh rate for flicker-free display
; - Timer0: Precision strobe pulse generation
; - Optimized bit manipulation and register usage
; - Comprehensive documentation and error handling
;
; Hardware Configuration:
; - PB3: Serial data input to shift register
; - PB4: Latch/strobe control
; - PB5: Shift register clock
; - 7x5 LED matrix (7 rows, 5 columns per character)
;

.include "m328pdef.inc"

; =============================================================================
; CONFIGURATION CONSTANTS
; =============================================================================

; Hardware pin assignments (Port B)
.equ SR_DDR         = DDRB
.equ SR_PORT        = PORTB
.equ SR_PINREG      = PINB
.equ SR_DATA_PIN    = 3          ; Serial data output (PB3)
.equ SR_CLOCK_PIN   = 5          ; Shift register clock (PB5)
.equ SR_LATCH_PIN   = 4          ; Output latch/strobe (PB4)

; Timer configuration values
.equ T2_PRELOAD_4KHZ = 251      ; Timer2 preload: 16MHz/(1024*4KHz) - 256 ≈ 251
.equ T0_STROBE_DELAY  = 240      ; Timer0 preload: short strobe pulse (~1µs)

; Display characteristics
.equ MATRIX_ROWS      = 7        ; Number of display rows
.equ MATRIX_COLS      = 5        ; Number of columns per character
.equ char_BYTES      = 8        ; Bytes per character char (including padding)
.equ COLUMN_CYCLES    = 16       ; Total shift cycles for column data

; Pin masks for efficient operations
.equ SR_OUTPUT_MASK   = (1<<SR_DATA_PIN)|(1<<SR_LATCH_PIN)|(1<<SR_CLOCK_PIN)
.equ TIMER2_PRESCALER = (1<<CS22)|(1<<CS21)|(1<<CS20)  ; Prescaler 1024
.equ TIMER0_PRESCALER = (1<<CS02)|(1<<CS01)|(1<<CS00)  ; Prescaler 1024

; =============================================================================
; OPTIMIZED MACROS
; =============================================================================

; Macro: Emit single bit to shift register with clock pulse
; Parameters: @0 = source register, @1 = bit position
; Cycles: 6 (optimized for speed)
.MACRO SHIFT_BIT_OUT
    sbi   SR_PORT, SR_DATA_PIN          ; Set data line high (assume 1)
    sbrs  @0, @1                        ; Skip if bit is set
    cbi   SR_PORT, SR_DATA_PIN          ; Clear data line if bit was 0
    sbi   SR_PINREG, SR_CLOCK_PIN       ; Toggle clock high (via PIN write)
    sbi   SR_PINREG, SR_CLOCK_PIN       ; Toggle clock low
.ENDMACRO

; Macro: Fast register clearing with bounds check
.MACRO SAFE_REGISTER_CLEAR
    .if @0 != r1                        ; Avoid clearing r1 (zero register)
        clr   @0
    .endif
.ENDMACRO

; =============================================================================
; INTERRUPT VECTOR TABLE
; =============================================================================
.cseg
.org 0x0000
    rjmp  system_init              ; Reset vector

.org 0x0010                        ; Timer0 overflow
    rjmp  timer0_strobe_isr

.org 0x0012                        ; Timer2 overflow  
    rjmp  timer2_refresh_isr

; =============================================================================
; SYSTEM INITIALIZATION
; =============================================================================
system_init:
    ; Configure stack pointer (defensive programming)
    ldi   r16, high(RAMEND)
    out   SPH, r16
    ldi   r16, low(RAMEND)
    out   SPL, r16
    
    ; Initialize shift register control pins
    ldi   r17, SR_OUTPUT_MASK
    out   SR_DDR, r17              ; Set pins as outputs
    out   SR_PORT, r17             ; Initialize all high for clean state
    
    ; Configure Timer2 for display refresh
    call  setup_timer2_refresh
    
    ; Configure Timer0 for strobe generation
    call  setup_timer0_strobe
    
    ; Initialize display state variables
    call  init_display_state
    
    ; Enable global interrupts
    sei
    
    ; Enter main loop
    rjmp  main_loop

; -----------------------------------------------------------------------------
; Timer2 Setup: Display refresh at ~480Hz
; -----------------------------------------------------------------------------
setup_timer2_refresh:
    ; Timer2: Normal mode, prescaler 1024
    lds   r16, TCCR2A
    andi  r16, ~((1<<WGM21)|(1<<WGM20))  ; Clear WGM bits for normal mode
    sts   TCCR2A, r16
    
    lds   r16, TCCR2B
    andi  r16, ~(1<<WGM22)              ; Clear WGM22
    ori   r16, TIMER2_PRESCALER         ; Set prescaler to 1024
    sts   TCCR2B, r16
    
    ; Enable Timer2 overflow interrupt
    lds   r16, TIMSK2
    ori   r16, (1<<TOIE2)
    sts   TIMSK2, r16
    
    ; Set initial counter value
    ldi   r16, T2_PRELOAD_4KHZ
    sts   TCNT2, r16
    ret

; -----------------------------------------------------------------------------
; Timer0 Setup: Strobe pulse generation
; -----------------------------------------------------------------------------
setup_timer0_strobe:
    ; Timer0: Normal mode, initially stopped
    lds   r16, TCCR0A
    andi  r16, ~((1<<WGM01)|(1<<WGM00))  ; Normal mode
    sts   TCCR0A, r16
    
    lds   r16, TCCR0B
    andi  r16, ~((1<<WGM02)|(1<<CS02)|(1<<CS01)|(1<<CS00)) ; Stop timer
    sts   TCCR0B, r16
    
    ; Enable Timer0 overflow interrupt
    lds   r16, TIMSK0
    ori   r16, (1<<TOIE0)
    sts   TIMSK0, r16
    ret

; -----------------------------------------------------------------------------
; Initialize Display State
; -----------------------------------------------------------------------------
init_display_state:
    ldi   r22, MATRIX_ROWS - 1     ; Row byte index (0-6 for 7 rows)
    ldi   r21, 0b01000000          ; Row select bit (starts at row 6, MSB)
    ret

; =============================================================================
; MAIN PROGRAM LOOP
; =============================================================================
main_loop:
    ; Main loop can perform other tasks
    ; Display refresh is handled entirely by interrupts
    nop                            ; Placeholder for user code
    rjmp  main_loop

; =============================================================================
; CHARACTER DEFINITIONS
; =============================================================================
; Each character is defined as 8 bytes (7 active rows + 1 padding)
; Bit pattern: 0bxxxCCCCC where CCCCC are the 5 column bits

characters:
char_A:  .db 0b00110, 0b01001, 0b01001, 0b01001, 0b01111, 0b01001, 0b01001, 0
char_B:  .db 0b01110, 0b01001, 0b01001, 0b01110, 0b01001, 0b01001, 0b01110, 0
char_C:  .db 0b00111, 0b01000, 0b01000, 0b01000, 0b01000, 0b01000, 0b00111, 0
char_D:  .db 0b01110, 0b01001, 0b01001, 0b01001, 0b01001, 0b01001, 0b01110, 0
char_E:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01111, 0
char_F:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01000, 0

; Numeric characters
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
; INTERRUPT SERVICE ROUTINES
; =============================================================================

; -----------------------------------------------------------------------------
; Timer0 ISR: Strobe pulse completion
; -----------------------------------------------------------------------------
timer0_strobe_isr:
    ; Reload timer for consistent timing
    ldi   r16, T0_STROBE_DELAY
    sts   TCNT0, r16
    
    ; Complete strobe pulse (toggle latch pin)
    sbi   SR_PINREG, SR_LATCH_PIN
    
    ; Stop Timer0 until next refresh cycle
    lds   r16, TCCR0B
    andi  r16, ~((1<<CS02)|(1<<CS01)|(1<<CS00))  ; Clear prescaler bits
    sts   TCCR0B, r16
    
    reti

; -----------------------------------------------------------------------------
; Timer2 ISR: Display refresh and row scanning
; -----------------------------------------------------------------------------
timer2_refresh_isr:
    ; Save context (use upper registers to avoid conflicts)
    push  r16
    in    r16, SREG
    push  r16
    push  r17
    push  r18
    
    ; Reload Timer2 for consistent refresh rate
    ldi   r16, T2_PRELOAD_4KHZ
    sts   TCNT2, r16

refresh_current_row:
    ; Fetch current row data from char B (example character)
    ldi   ZL, low(char_7 * 2)     ; Convert to byte address
    ldi   ZH, high(char_7 * 2)
    
    ; Validate and adjust row index
    cpi   r22, MATRIX_ROWS
    brlo  row_index_valid
    ldi   r22, MATRIX_ROWS - 1     ; Clamp to valid range

row_index_valid:
    add   ZL, r22                  ; Add row offset
    adc   ZH, r1                   ; Handle carry (r1 is always 0)
    lpm   r20, Z                   ; Load row pattern into r20

shift_column_data:
    ; Send column data (16 cycles of 5 bits each = 80 total bits)
    ldi   r18, COLUMN_CYCLES
    
column_shift_loop:
    SHIFT_BIT_OUT r20, 0          ; Send bit 0 (rightmost column)
    SHIFT_BIT_OUT r20, 1          ; Send bit 1
    SHIFT_BIT_OUT r20, 2          ; Send bit 2
    SHIFT_BIT_OUT r20, 3          ; Send bit 3
    SHIFT_BIT_OUT r20, 4          ; Send bit 4 (leftmost column)
    
    dec   r18
    brne  column_shift_loop

send_row_select:
    ; Insert latch separator (ensure clean transition)
    cbi   SR_PORT, SR_DATA_PIN
    sbi   SR_PINREG, SR_CLOCK_PIN
    sbi   SR_PINREG, SR_CLOCK_PIN
    
    ; Send 7-bit row select pattern (one-hot encoding)
    SHIFT_BIT_OUT r21, 6          ; Row 6 (MSB)
    SHIFT_BIT_OUT r21, 5          ; Row 5  
    SHIFT_BIT_OUT r21, 4          ; Row 4
    SHIFT_BIT_OUT r21, 3          ; Row 3
    SHIFT_BIT_OUT r21, 2          ; Row 2
    SHIFT_BIT_OUT r21, 1          ; Row 1
    SHIFT_BIT_OUT r21, 0          ; Row 0 (LSB)

advance_to_next_row:
    ; Advance to next row with wraparound
    dec   r22                      ; Move to next row byte
    lsr   r21                      ; Shift row select bit right
    
    ; Check for row wraparound
    brne  trigger_strobe           ; Continue if more rows
    
    ; Reset to top row
    ldi   r22, MATRIX_ROWS - 1
    ldi   r21, 0b01000000          ; Reset to row 6 (MSB)

trigger_strobe:
    ; Initiate strobe pulse via Timer0
    sbi   SR_PINREG, SR_LATCH_PIN  ; Start strobe pulse
    
    ; Start Timer0 for precise strobe duration
    lds   r16, TCCR0B
    ori   r16, TIMER0_PRESCALER    ; Start timer with prescaler
    sts   TCCR0B, r16
    
    ; Restore context
    pop   r18
    pop   r17
    pop   r16
    out   SREG, r16
    pop   r16
    
    reti

; =============================================================================
; END OF PROGRAM
; =============================================================================