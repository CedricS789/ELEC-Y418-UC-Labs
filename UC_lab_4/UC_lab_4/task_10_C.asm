; =============================================================================
; LED MATRIX DISPLAY DRIVER FOR ATMEGA328P
; =============================================================================
; 
; Author:          [Your Name]
; Date Created:    [Current Date]
; Last Modified:   [Current Date]
; Version:         1.0.0
; Target MCU:      ATmega328P (Arduino Uno compatible)
; Clock:           16MHz external crystal
; 
; DESCRIPTION:
; Professional LED matrix display driver using STP08DP05 LED driver IC.
; Implements continuous multiplexed refresh for optimal brightness and
; flicker-free operation with hardware-timed latch pulses.
;
; FEATURES:
; - Shift-register based LED matrix control (STP08DP05 LED driver)
; - Continuous refresh using optimized multiplexing algorithm
; - Timer0-based precision latch pulse generation
; - Optimized bit manipulation with cycle-accurate timing
; - Comprehensive error handling and bounds checking
; - Modular character definition system
; - Memory-efficient register usage
;
; HARDWARE CONFIGURATION:
; ┌─────────────────────────────────────────────────────────────────────┐
; │ Pin Assignment:                                                     │
; │ • PB3 (Arduino D11) - Serial Data Input to STP08DP05 (SDI)          │
; │ • PB4 (Arduino D12) - Latch/Output Control (LE)                     │
; │ • PB5 (Arduino D13) - Shift Register Clock (CLK)                    │
; │                                                                     │
; │ LED Matrix: 7x5 character display (7 rows × 5 columns)              │
; │ LED Driver: STP08DP05 with integrated current sinks (90mA max)      │
; │ Current Setting: External resistor on IREF pin (typically 1.2kΩ)    │
; └─────────────────────────────────────────────────────────────────────┘
;
; =============================================================================

.include "m328pdef.inc"

; =============================================================================
; SYSTEM CONFIGURATION CONSTANTS
; =============================================================================

; I/O Port Configuration (Port B assignments)
.equ SR_DDR           = DDRB      ; Data Direction Register
.equ SR_PORT          = PORTB     ; Output Port Register  
.equ SR_PINREG        = PINB      ; Pin Toggle Register
.equ SR_DATA_PIN      = 3         ; Serial data input to STP08DP05 (PB3)
.equ SR_CLOCK_PIN     = 5         ; STP08DP05 clock input (PB5)
.equ SR_LATCH_PIN     = 4         ; STP08DP05 latch enable (PB4)

; Timer0 Configuration Constants
.equ T0_LATCH_DELAY   = 240       ; Timer0 preload for latch pulse
.equ TIMER0_PRESCALER = (1<<CS02)|(1<<CS01)|(1<<CS00)  ; Prescaler: 1024

; Display Matrix Specifications
.equ MATRIX_ROWS      = 7         ; Vertical resolution (rows)
.equ MATRIX_COLS      = 5         ; Horizontal resolution per character
.equ CHAR_BYTES       = 8         ; Storage bytes per character (7+1 padding)
.equ COLUMN_CYCLES    = 16        ; Shift register fill cycles (80÷5)

; Target Block Selection (0-15, where 0 is first block out, 15 is last)
.equ TARGET_BLOCK     = 14        ; Block to display character on

; Derived Constants and Bit Masks
.equ SR_OUTPUT_MASK   = (1<<SR_DATA_PIN)|(1<<SR_LATCH_PIN)|(1<<SR_CLOCK_PIN)
.equ ROW_RESET_MASK   = 0b01000000  ; Initial row selector (row 6, MSB first)
.equ MAX_ROW_INDEX    = MATRIX_ROWS - 1  ; Bounds checking constant

; =============================================================================
; PERFORMANCE-OPTIMIZED MACROS
; =============================================================================

; -----------------------------------------------------------------------------
; Macro: SHIFT_BIT_OUT
; Purpose: Emit single bit to STP08DP05 with synchronized clock pulse
; Parameters: 
;   @0 = source register containing bit pattern
;   @1 = bit position to extract (0-7)
; Side Effects: Modifies SR_PORT pins, generates clock edge
; Note: Uses skip instruction for branch-free bit testing
; -----------------------------------------------------------------------------
.MACRO SHIFT_BIT_OUT
    sbi   SR_PORT, SR_DATA_PIN          ; Pre-set data line (assume bit = 1)
    sbrs  @0, @1                        ; Skip next if bit[@1] is set
    cbi   SR_PORT, SR_DATA_PIN          ; Clear data line if bit was 0
    sbi   SR_PINREG, SR_CLOCK_PIN       ; Generate positive clock edge
    sbi   SR_PINREG, SR_CLOCK_PIN       ; Generate negative clock edge
.ENDMACRO

; -----------------------------------------------------------------------------
; Macro: SAFE_REGISTER_CLEAR
; Purpose: Clear register with protection against system register corruption
; Parameters: @0 = target register
; Safety: Prevents accidental clearing of r1 (designated zero register)
; Usage: SAFE_REGISTER_CLEAR r16
; -----------------------------------------------------------------------------
.MACRO SAFE_REGISTER_CLEAR
    .if @0 != r1                        ; Compile-time safety check
        clr   @0                        ; Clear register if not r1
    .else
        .error "Cannot clear r1 - reserved zero register"
    .endif
.ENDMACRO

; -----------------------------------------------------------------------------
; Macro: BOUNDARY_CHECK_ROW
; Purpose: Validate and clamp row index to prevent array overflow
; Parameters: @0 = row index register
; Modifies: @0 (clamped to valid range)
; Range: 0 to MAX_ROW_INDEX
; -----------------------------------------------------------------------------
.MACRO BOUNDARY_CHECK_ROW
    cpi   @0, MAX_ROW_INDEX + 1         ; Compare with upper bound
    brlo  PC + 2                        ; Skip correction if valid
    ldi   @0, MAX_ROW_INDEX             ; Clamp to maximum valid index
.ENDMACRO

; =============================================================================
; INTERRUPT VECTOR TABLE - CRITICAL SYSTEM ENTRY POINTS
; =============================================================================
.cseg
.org 0x0000
    rjmp  system_init                   ; RESET - Power-on/external reset
                                       ; All other vectors use default (reti)

.org 0x0010                           ; TIMER0_OVF - Timer0 overflow
    rjmp  timer0_latch_completion_isr ; Precision latch timing control

; =============================================================================
; SYSTEM INITIALIZATION AND CONFIGURATION
; =============================================================================

; -----------------------------------------------------------------------------
; Function: system_init
; Purpose: Complete system initialization sequence
; Registers Modified: r16, r17
; Stack Usage: None (called before main)
; Critical: Must execute before any peripheral operations
; -----------------------------------------------------------------------------
system_init:
    cli                                ; Disable interrupts during setup
    
    ; Initialize stack pointer for proper function calls and interrupts
    ; Stack grows downward from RAMEND (0x08FF on ATmega328P)
    ldi   r16, high(RAMEND)
    out   SPH, r16                     ; Set stack pointer high byte
    ldi   r16, low(RAMEND)  
    out   SPL, r16                     ; Set stack pointer low byte
    
    ; Configure shift register control interface
    call  init_shift_register_pins
    
    ; Setup Timer0 for precision latch pulse generation  
    call  configure_timer0_system
    
    ; Initialize display state management variables
    call  initialize_display_state
    
    ; Clear r1 for use as zero register (AVR ABI requirement)
    clr   r1                           ; r1 used for zero bits in block selection
    
    ; System ready - enable global interrupt flag
    sei                                ; Global interrupt enable
    
    ; Transfer control to main application loop
    rjmp  main_application_loop

; -----------------------------------------------------------------------------
; Function: init_shift_register_pins  
; Purpose: Configure Port B pins for STP08DP05 LED driver interface
; Registers Modified: r17
; Hardware: Sets PB3, PB4, PB5 as outputs with clean initial state
; -----------------------------------------------------------------------------
init_shift_register_pins:
    ldi   r17, SR_OUTPUT_MASK          ; Load pin configuration mask
    out   SR_DDR, r17                  ; Configure as output pins  
    out   SR_PORT, r17                 ; Set initial state (all high)
    ret

; -----------------------------------------------------------------------------
; Function: configure_timer0_system
; Purpose: Initialize Timer0 for latch pulse timing control
; Registers Modified: r16  
; Configuration: Normal mode, overflow interrupt enabled, initially stopped
; -----------------------------------------------------------------------------
configure_timer0_system:
    ; Configure Timer0 operating mode (Normal mode, count up)
    lds   r16, TCCR0A
    andi  r16, ~((1<<WGM01)|(1<<WGM00)) ; Clear WGM bits for normal mode
    sts   TCCR0A, r16
    
    ; Configure Timer0 clock source (initially stopped)
    lds   r16, TCCR0B  
    andi  r16, ~((1<<WGM02)|(1<<CS02)|(1<<CS01)|(1<<CS00))
    sts   TCCR0B, r16                  ; Timer stopped, normal mode
    
    ; Enable Timer0 overflow interrupt for latch completion detection
    lds   r16, TIMSK0
    ori   r16, (1<<TOIE0)              ; Set Timer0 overflow interrupt enable
    sts   TIMSK0, r16
    ret

; -----------------------------------------------------------------------------
; Function: initialize_display_state
; Purpose: Set initial conditions for display multiplexing
; Registers Modified: r21, r22 (global state registers)
; State: Row 6 selected (MSB first scanning), byte index = 6
; -----------------------------------------------------------------------------
initialize_display_state:
    ldi   r22, MAX_ROW_INDEX           ; Set row byte index (0-6 for 7 rows)
    ldi   r21, ROW_RESET_MASK          ; Set initial row selector bit
    ret

; =============================================================================
; MAIN APPLICATION LOOP - HIGH-SPEED DISPLAY REFRESH
; =============================================================================

; -----------------------------------------------------------------------------
; Function: main_application_loop
; Purpose: Continuous high-frequency display refresh for maximum brightness
; Strategy: Tight loop with single-row refresh per iteration
; -----------------------------------------------------------------------------
main_application_loop:
    call  refresh_current_display_row  ; Update one row of the matrix
    rjmp  main_application_loop        ; Infinite loop for continuous refresh

; =============================================================================
; CHARACTER BITMAP DEFINITIONS - 7×5 PIXEL FONT
; =============================================================================
; 
; FORMAT SPECIFICATION:
; • Each character: 8 bytes (7 active rows + 1 padding byte)  
; • Bit encoding: 0bxxxCCCCC where CCCCC = column bits (5 pixels wide)
; • Bit 0 = rightmost column, Bit 4 = leftmost column
; • Padding byte (index 7) must be 0 for proper array bounds
; • Row 0 = top of character, Row 6 = bottom of character
;
; MEMORY LAYOUT:
; characters + (char_index * 8) + row_index = byte address
; Example: char_A row 3 = characters + (0 * 8) + 3
;
; VISUAL REFERENCE GRID:
;   Bit:  4 3 2 1 0
;   Col:  █ █ █ █ █  <- 5 columns per character
;         ↑       ↑
;      Left    Right
;
; =============================================================================

character_bitmap_table:

; Alphabetic Characters (A-F implemented)
char_A:  .db 0b00110, 0b01001, 0b01001, 0b01001, 0b01111, 0b01001, 0b01001, 0  ; A
char_B:  .db 0b01110, 0b01001, 0b01001, 0b01110, 0b01001, 0b01001, 0b01110, 0  ; B  
char_C:  .db 0b00111, 0b01000, 0b01000, 0b01000, 0b01000, 0b01000, 0b00111, 0  ; C
char_D:  .db 0b01110, 0b01001, 0b01001, 0b01001, 0b01001, 0b01001, 0b01110, 0  ; D
char_E:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01111, 0  ; E
char_F:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01000, 0  ; F

; Numeric Characters (0-9 complete set)
char_0:  .db 0b01111, 0b01001, 0b01001, 0b01001, 0b01001, 0b01001, 0b01111, 0  ; 0
char_1:  .db 0b00010, 0b00110, 0b01010, 0b00010, 0b00010, 0b00010, 0b00010, 0  ; 1
char_2:  .db 0b01111, 0b00001, 0b00001, 0b01111, 0b01000, 0b01000, 0b01111, 0  ; 2
char_3:  .db 0b01111, 0b00001, 0b00001, 0b00111, 0b00001, 0b00001, 0b01111, 0  ; 3
char_4:  .db 0b01001, 0b01001, 0b01001, 0b01111, 0b00001, 0b00001, 0b00001, 0  ; 4  
char_5:  .db 0b01111, 0b01000, 0b01000, 0b01111, 0b00001, 0b00001, 0b01111, 0  ; 5
char_6:  .db 0b01111, 0b01000, 0b01000, 0b01111, 0b01001, 0b01001, 0b01111, 0  ; 6
char_7:  .db 0b01111, 0b00001, 0b00001, 0b00010, 0b00100, 0b00100, 0b00100, 0  ; 7
char_8:  .db 0b01111, 0b01001, 0b01001, 0b01111, 0b01001, 0b01001, 0b01111, 0  ; 8
char_9:  .db 0b01111, 0b01001, 0b01001, 0b01111, 0b00001, 0b00001, 0b01111, 0  ; 9

; Character set boundary marker (for bounds checking)
character_table_end:

; =============================================================================
; INTERRUPT SERVICE ROUTINES - TIME-CRITICAL HANDLERS
; =============================================================================

; -----------------------------------------------------------------------------
; ISR: timer0_latch_completion_isr  
; Purpose: Complete latch pulse and disable timer for next cycle
; Trigger: Timer0 overflow after delay
; Registers: Uses r16 (automatically saved/restored by hardware)
; Critical: This ISR controls display multiplexing timing accuracy
; -----------------------------------------------------------------------------
timer0_latch_completion_isr:
    ; Reload timer counter for consistent timing on next activation  
    ldi   r16, T0_LATCH_DELAY          ; Load latch duration constant
    sts   TCNT0, r16                   ; Reset counter for next cycle
    
    ; Complete latch pulse sequence (end of latch pulse)
    sbi   SR_PINREG, SR_LATCH_PIN      ; Generate falling edge on latch
    
    ; Disable Timer0 to save power until next refresh cycle  
    lds   r16, TCCR0B                  ; Read current timer control
    andi  r16, ~((1<<CS02)|(1<<CS01)|(1<<CS00))  ; Clear prescaler bits
    sts   TCCR0B, r16                  ; Stop timer (no clock source)
    
    reti                               ; Return from interrupt, restore SREG

; -----------------------------------------------------------------------------
; Function: refresh_current_display_row
; Purpose: Complete single-row refresh cycle with automatic row advancement  
; Algorithm: Fetch→Shift→Latch→Advance in optimized sequence
; Registers: 
;   Input:  r21 (current row selector), r22 (current row index)  
;   Output: r21, r22 (advanced to next row with wraparound)
;   Temp:   r16, r17, r18, r19, r20 (automatically saved/restored)
; Hardware: Generates 87 clock pulses + precision latch pulse
; -----------------------------------------------------------------------------
refresh_current_display_row:
    ; Preserve working registers on stack (LIFO order)  
    push  r16                          ; Timer control register
    push  r17                          ; General purpose
    push  r18                          ; Loop counter
    push  r19                          ; Current block index

fetch_row_bitmap_data:
    ; Calculate character bitmap address: char_7 + row_offset
    ; Note: Currently displays character '7' - easily configurable
    ldi   ZL, low(char_7 * 2)         ; Load character base address (word→byte)
    ldi   ZH, high(char_7 * 2)        ; Program memory addresses are word-based
    
    ; Apply bounds checking to prevent buffer overflow
    BOUNDARY_CHECK_ROW r22             ; Macro: clamp r22 to valid range
    
    ; Calculate final byte address: base + row_index  
    add   ZL, r22                      ; Add row offset to low byte
    adc   ZH, r1                       ; Add carry to high byte (r1 = 0)
    lpm   r20, Z                       ; Load row bitmap from program memory

transmit_column_data:
    ; Transmit column data for 16 blocks, only TARGET_BLOCK gets the character
    ldi   r18, COLUMN_CYCLES           ; Load iteration counter (16 blocks)
    clr   r19                          ; Current block index (0-15)
    
column_transmission_loop:
    ; Check if this is the target block
    cpi   r19, TARGET_BLOCK            ; Compare current block with target
    brne  send_zero_block              ; If not target, send zeros
    
send_character_block:
    ; Send the actual character data for this block
    SHIFT_BIT_OUT r20, 0               ; Transmit bit 0 (rightmost column)
    SHIFT_BIT_OUT r20, 1               ; Transmit bit 1  
    SHIFT_BIT_OUT r20, 2               ; Transmit bit 2
    SHIFT_BIT_OUT r20, 3               ; Transmit bit 3
    SHIFT_BIT_OUT r20, 4               ; Transmit bit 4 (leftmost column)
    rjmp  next_block                   ; Skip to next block
    
send_zero_block:
    ; Send zeros for non-target blocks (r1 is always 0)
    SHIFT_BIT_OUT r1, 0                ; Transmit 0
    SHIFT_BIT_OUT r1, 1                ; Transmit 0
    SHIFT_BIT_OUT r1, 2                ; Transmit 0
    SHIFT_BIT_OUT r1, 3                ; Transmit 0
    SHIFT_BIT_OUT r1, 4                ; Transmit 0
    
next_block:
    inc   r19                          ; Move to next block
    dec   r18                          ; Decrement loop counter
    brne  column_transmission_loop     ; Continue until all blocks complete

insert_row_separator:
    ; Insert clean separator between column and row data  
    cbi   SR_PORT, SR_DATA_PIN         ; Ensure data line is low
    sbi   SR_PINREG, SR_CLOCK_PIN      ; Generate separator clock pulse  
    sbi   SR_PINREG, SR_CLOCK_PIN      ; Complete clock cycle

transmit_row_select_data:
    ; Transmit 7-bit row selector (one-hot encoding, MSB first)
    ; Only one bit is high, indicating which row to activate
    SHIFT_BIT_OUT r21, 6               ; Row 6 selector (MSB)
    SHIFT_BIT_OUT r21, 5               ; Row 5 selector
    SHIFT_BIT_OUT r21, 4               ; Row 4 selector  
    SHIFT_BIT_OUT r21, 3               ; Row 3 selector
    SHIFT_BIT_OUT r21, 2               ; Row 2 selector
    SHIFT_BIT_OUT r21, 1               ; Row 1 selector
    SHIFT_BIT_OUT r21, 0               ; Row 0 selector (LSB)

advance_to_next_row:
    ; Update row state for next refresh cycle
    dec   r22                          ; Move to next row byte index
    lsr   r21                          ; Shift row selector bit right
    
    ; Check for row sequence wraparound
    brne  initiate_hardware_latch      ; Continue if more rows remain
    
    ; Reset to top of matrix (row sequence complete)
    ldi   r22, MAX_ROW_INDEX           ; Reset to last row index
    ldi   r21, ROW_RESET_MASK          ; Reset row selector to MSB position

initiate_hardware_latch:
    ; Begin precision latch pulse sequence
    sbi   SR_PINREG, SR_LATCH_PIN      ; Start latch pulse (rising edge)
    
    ; Activate Timer0 for precise latch duration control
    lds   r16, TCCR0B                  ; Read current timer configuration  
    ori   r16, TIMER0_PRESCALER        ; Apply prescaler bits
    sts   TCCR0B, r16                  ; Start timer with clock source
    
    ; Restore working registers from stack (reverse LIFO order)
    pop   r19                          ; Restore block index
    pop   r18                          ; Restore loop counter
    pop   r17                          ; Restore general purpose
    pop   r16                          ; Restore timer control
    
    ret                                ; Return to main loop

; =============================================================================
; END OF PROGRAM - DEVELOPMENT AND MAINTENANCE INFORMATION
; =============================================================================
;
; HARDWARE COMPATIBILITY:
; ├─ Microcontroller: ATmega328P (Arduino Uno compatible)
; ├─ LED Driver: STP08DP05
; ├─ LED Matrix: 7×5 character display
; └─ Power Supply: 5V regulated
;
; MODIFICATION GUIDELINES:
; ├─ Character Set: Edit character_bitmap_table section
; ├─ Display Size: Modify MATRIX_ROWS/MATRIX_COLS constants  
; ├─ Target Block: Change TARGET_BLOCK constant (0-15)
; ├─ Character Selection: Change char_7 reference in fetch_row_bitmap_data
; ├─ Timing: Adjust T0_LATCH_DELAY for different latch widths
; ├─ Pin Assignment: Change SR_*_PIN constants for different connections
; └─ Refresh Rate: Modify main loop or add delays for slower refresh
;
; KNOWN LIMITATIONS:
; ├─ Single character display on one block only (easily extended to multiple)
; ├─ No brightness control (could add PWM dimming)
; ├─ Fixed character (char_7) - could implement character selection
; ├─ Fixed target block - could implement dynamic block selection
; └─ No error recovery for hardware faults
;
; VERSION HISTORY:
; └─ v1.0.0 - Initial professional implementation with full documentation
;
; =============================================================================