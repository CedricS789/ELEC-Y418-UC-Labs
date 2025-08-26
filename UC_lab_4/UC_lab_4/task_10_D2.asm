; LED Matrix Display Driver with 4x4 Keypad for ATmega328P
; Pin Assignment: PB3=Data, PB4=Latch, PB5=Clock, PD7-4=Keypad Rows, PD3-0=Keypad Cols

.include "m328pdef.inc"

; Constants
.equ SR_DDR           = DDRB
.equ SR_PORT          = PORTB
.equ SR_PINREG        = PINB
.equ SR_DATA_PIN      = 3
.equ SR_CLOCK_PIN     = 5
.equ SR_LATCH_PIN     = 4

; Timer0 Configuration
.equ T0_LATCH_DELAY   = 240
.equ TIMER0_PRESCALER = (1<<CS02)|(1<<CS01)|(1<<CS00)

; Keypad Configuration
.equ KEYPAD_DDR       = DDRD
.equ KEYPAD_PORT      = PORTD
.equ KEYPAD_PIN       = PIND
.equ KEYPAD_ROW_MASK  = 0b11110000
.equ KEYPAD_COL_MASK  = 0b00001111
.equ DEBOUNCE_CYCLES  = 5

; Keypad pins
.equ ROW_1            = 7
.equ ROW_2            = 6
.equ ROW_3            = 5
.equ ROW_4            = 4
.equ COL_0            = 3
.equ COL_1            = 2
.equ COL_2            = 1
.equ COL_3            = 0

; Keypad scan codes
.equ KEY_7 = 1
.equ KEY_8 = 2
.equ KEY_9 = 3
.equ KEY_F = 4
.equ KEY_4 = 5
.equ KEY_5 = 6
.equ KEY_6 = 7
.equ KEY_E = 8
.equ KEY_1 = 9
.equ KEY_2 = 10
.equ KEY_3 = 11
.equ KEY_D = 12
.equ KEY_A = 13
.equ KEY_0 = 14
.equ KEY_B = 15
.equ KEY_C = 16

; Display Matrix
.equ MATRIX_ROWS      = 7
.equ MATRIX_COLS      = 5
.equ CHAR_BYTES       = 8
.equ COLUMN_CYCLES    = 16
.equ TARGET_BLOCK     = 14

.def key_state        = r24
.def debounce_counter = r23

; Derived Constants
.equ SR_OUTPUT_MASK   = (1<<SR_DATA_PIN)|(1<<SR_LATCH_PIN)|(1<<SR_CLOCK_PIN)
.equ ROW_RESET_MASK   = 0b01000000
.equ MAX_ROW_INDEX    = MATRIX_ROWS - 1

; Memory allocation
.dseg
.org 0x0100
current_char:     .byte 1
.cseg

; Macros
.MACRO SHIFT_BIT_OUT
    sbi   SR_PORT, SR_DATA_PIN
    sbrs  @0, @1
    cbi   SR_PORT, SR_DATA_PIN
    sbi   SR_PINREG, SR_CLOCK_PIN
    sbi   SR_PINREG, SR_CLOCK_PIN
.ENDMACRO

.MACRO SAFE_REGISTER_CLEAR
    .if @0 != r1
        clr   @0
    .else
        .error "Cannot clear r1 - reserved zero register"
    .endif
.ENDMACRO

.MACRO BOUNDARY_CHECK_ROW
    cpi   @0, MAX_ROW_INDEX + 1
    brlo  PC + 2
    ldi   @0, MAX_ROW_INDEX
.ENDMACRO

; Interrupt vectors
.cseg
.org 0x0000
    rjmp  system_init

.org 0x0010
    rjmp  timer0_latch_completion_isr

; System initialization
system_init:
    cli
    
    ; Setup stack pointer
    ldi   r16, high(RAMEND)
    out   SPH, r16
    ldi   r16, low(RAMEND)
    out   SPL, r16
    
    ; Initialize hardware
    call  init_shift_register_pins
    call  init_keypad_pins
    call  configure_timer0_system
    
    ; Initialize software state
    call  initialize_display_state
    call  initialize_keypad_state
    
    clr   r1
    sei
    rjmp  main_application_loop

init_shift_register_pins:
    ldi   r17, SR_OUTPUT_MASK
    out   SR_DDR, r17
    out   SR_PORT, r17
    ret

init_keypad_pins:
    ldi   r16, KEYPAD_ROW_MASK
    out   KEYPAD_DDR, r16
    ldi   r16, 0xFF
    out   KEYPAD_PORT, r16
    ret

configure_timer0_system:
    ; Configure normal mode
    lds   r16, TCCR0A
    andi  r16, ~((1<<WGM01)|(1<<WGM00))
    sts   TCCR0A, r16
    
    ; Stop timer initially
    lds   r16, TCCR0B
    andi  r16, ~((1<<WGM02)|(1<<CS02)|(1<<CS01)|(1<<CS00))
    sts   TCCR0B, r16
    
    ; Enable overflow interrupt
    lds   r16, TIMSK0
    ori   r16, (1<<TOIE0)
    sts   TIMSK0, r16
    ret

initialize_display_state:
    ldi   r22, MAX_ROW_INDEX
    ldi   r21, ROW_RESET_MASK
    ret

initialize_keypad_state:
    ldi   r16, '0'
    sts   current_char, r16
    ldi   key_state, 0xFF
    clr   debounce_counter
    ret

; Keypad Interface Functions
scan_keypad_nonblocking:
    push  r16
    push  r17
    
    ; Check debounce period
    tst   debounce_counter
    brne  skip_keypad_scan
    
    ; Perform keypad scan
    call  scan_keyboard_matrix
    
    ; Check if key detected
    cpi   r16, 0xFF
    breq  no_key_pressed
    
    ; Check if different from current state
    cp    r16, key_state
    breq  scan_keypad_exit
    
    ; New key detected
    mov   key_state, r16
    call  map_key_to_character
    sts   current_char, r16
    ldi   debounce_counter, DEBOUNCE_CYCLES
    rjmp  scan_keypad_exit
    
no_key_pressed:
    ldi   key_state, 0xFF
    
skip_keypad_scan:
    tst   debounce_counter
    breq  scan_keypad_exit
    dec   debounce_counter
    
scan_keypad_exit:
    pop   r17
    pop   r16
    ret

scan_keyboard_matrix:
    push  r17
    push  r18
    push  r19
    push  r20
    
    ldi   r19, 0xFF    ; Default: no key found
    ldi   r20, 1       ; Row counter (1-4)
    
scan_next_row:
    ; Set all rows HIGH
    sbi   PORTD, ROW_1
    sbi   PORTD, ROW_2
    sbi   PORTD, ROW_3
    sbi   PORTD, ROW_4
    
    ; Activate current row (set LOW)
    cpi   r20, 1
    breq  activate_row_1
    cpi   r20, 2
    breq  activate_row_2
    cpi   r20, 3
    breq  activate_row_3
    cpi   r20, 4
    breq  activate_row_4
    rjmp  scan_complete
    
activate_row_1:
    cbi   PORTD, ROW_1
    rjmp  read_columns
activate_row_2:
    cbi   PORTD, ROW_2
    rjmp  read_columns
activate_row_3:
    cbi   PORTD, ROW_3
    rjmp  read_columns
activate_row_4:
    cbi   PORTD, ROW_4
    
read_columns:
    nop    ; Wait for signals to settle
    nop
    nop
    nop
    
    in    r17, PIND    ; Read column states
    
    ; Check each column
    sbrs  r17, COL_0
    call  calculate_key_code_col0
    sbrs  r17, COL_1
    call  calculate_key_code_col1
    sbrs  r17, COL_2
    call  calculate_key_code_col2
    sbrs  r17, COL_3
    call  calculate_key_code_col3
    
    ; Anti-ghosting check
    mov   r18, r17
    com   r18
    andi  r18, 0x0F
    call  count_set_bits
    cpi   r16, 2
    brsh  ghost_detected
    
    inc   r20
    cpi   r20, 5
    brlo  scan_next_row
    rjmp  scan_complete
    
ghost_detected:
    ldi   r19, 0xFF
    
scan_complete:
    mov   r16, r19
    pop   r20
    pop   r19
    pop   r18
    pop   r17
    ret

; Key code calculation helpers
calculate_key_code_col0:
    ldi   r18, 0
    rjmp  store_key_with_row

calculate_key_code_col1:
    ldi   r18, 1
    rjmp  store_key_with_row

calculate_key_code_col2:
    ldi   r18, 2
    rjmp  store_key_with_row

calculate_key_code_col3:
    ldi   r18, 3
    
store_key_with_row:
    mov   r19, r20
    dec   r19
    lsl   r19
    lsl   r19
    add   r19, r18
    inc   r19
    ret

count_set_bits:
    push  r17
    ldi   r16, 0
    mov   r17, r18
    
count_loop:
    tst   r17
    breq  count_done
    sbrc  r17, 0
    inc   r16
    lsr   r17
    rjmp  count_loop
    
count_done:
    pop   r17
    ret

map_key_to_character:
    push  ZH
    push  ZL
    
    ; Bounds checking
    cpi   r16, 1
    brlo  map_default_char
    cpi   r16, 17
    brsh  map_default_char
    
    ; Convert to table index
    dec   r16
    
    ; Calculate address
    ldi   ZH, high(keypad_char_map * 2)
    ldi   ZL, low(keypad_char_map * 2)
    add   ZL, r16
    brcc  no_carry_map
    inc   ZH
no_carry_map:
    lpm   r16, Z
    rjmp  map_exit
    
map_default_char:
    ldi   r16, '?'
    
map_exit:
    pop   ZL
    pop   ZH
    ret

; Keypad character mapping table
keypad_char_map:
    .db   '7', '8', '9', 'F'    ; Row 1
    .db   '4', '5', '6', 'E'    ; Row 2
    .db   '1', '2', '3', 'D'    ; Row 3
    .db   'A', '0', 'B', 'C'    ; Row 4

; Main application loop
main_application_loop:
    call  refresh_current_display_row
    call  scan_keypad_nonblocking
    rjmp  main_application_loop

; =============================================================================
; CHARACTER BITMAP LOOKUP FUNCTIONS
; Purpose: Map ASCII characters to bitmap addresses in program memory
; =============================================================================
;
; HOW CHARACTER DISPLAY WORKS:
; Each character (like 'A', '5', 'F') is represented as a bitmap - a pattern of
; dots that forms the visual shape of the character. Our display is 7 rows by 
; 5 columns, so each character is stored as 7 bytes (one per row) plus 1 padding byte.
;
; EXAMPLE: The character 'A' might look like:
; Row 0: 01110  (bits: 0b01110 = 0x0E)
; Character bitmap lookup
get_character_bitmap_address:
    push  ZH
    push  ZL
    
    ; Check supported characters
    cpi   r16, '0'
    breq  map_char_0
    cpi   r16, '1'
    breq  map_char_1
    cpi   r16, '2'
    breq  map_char_2
    cpi   r16, '3'
    breq  map_char_3
    cpi   r16, '4'
    breq  map_char_4
    cpi   r16, '5'
    breq  map_char_5
    cpi   r16, '6'
    breq  map_char_6
    cpi   r16, '7'
    breq  map_char_7
    cpi   r16, '8'
    breq  map_char_8
    cpi   r16, '9'
    breq  map_char_9
    cpi   r16, 'A'
    breq  map_char_A
    cpi   r16, 'B'
    breq  map_char_B
    cpi   r16, 'C'
    breq  map_char_C
    cpi   r16, 'D'
    breq  map_char_D
    cpi   r16, 'E'
    breq  map_char_E
    cpi   r16, 'F'
    breq  map_char_F
    
    ; Default case
    ldi   r16, low(char_7 * 2)
    ldi   r17, high(char_7 * 2)
    rjmp  bitmap_address_exit

map_char_0:
    ldi   r16, low(char_0 * 2)
    ldi   r17, high(char_0 * 2)
    rjmp  bitmap_address_exit
map_char_1:
    ldi   r16, low(char_1 * 2)
    ldi   r17, high(char_1 * 2)
    rjmp  bitmap_address_exit
map_char_2:
    ldi   r16, low(char_2 * 2)
    ldi   r17, high(char_2 * 2)
    rjmp  bitmap_address_exit
map_char_3:
    ldi   r16, low(char_3 * 2)
    ldi   r17, high(char_3 * 2)
    rjmp  bitmap_address_exit
map_char_4:
    ldi   r16, low(char_4 * 2)
    ldi   r17, high(char_4 * 2)
    rjmp  bitmap_address_exit
map_char_5:
    ldi   r16, low(char_5 * 2)
    ldi   r17, high(char_5 * 2)
    rjmp  bitmap_address_exit
map_char_6:
    ldi   r16, low(char_6 * 2)
    ldi   r17, high(char_6 * 2)
    rjmp  bitmap_address_exit
map_char_7:
    ldi   r16, low(char_7 * 2)
    ldi   r17, high(char_7 * 2)
    rjmp  bitmap_address_exit
map_char_8:
    ldi   r16, low(char_8 * 2)
    ldi   r17, high(char_8 * 2)
    rjmp  bitmap_address_exit
map_char_9:
    ldi   r16, low(char_9 * 2)
    ldi   r17, high(char_9 * 2)
    rjmp  bitmap_address_exit
map_char_A:
    ldi   r16, low(char_A * 2)
    ldi   r17, high(char_A * 2)
    rjmp  bitmap_address_exit
map_char_B:
    ldi   r16, low(char_B * 2)
    ldi   r17, high(char_B * 2)
    rjmp  bitmap_address_exit
map_char_C:
    ldi   r16, low(char_C * 2)
    ldi   r17, high(char_C * 2)
    rjmp  bitmap_address_exit
map_char_D:
    ldi   r16, low(char_D * 2)
    ldi   r17, high(char_D * 2)
    rjmp  bitmap_address_exit
map_char_E:
    ldi   r16, low(char_E * 2)
    ldi   r17, high(char_E * 2)
    rjmp  bitmap_address_exit
map_char_F:
    ldi   r16, low(char_F * 2)
    ldi   r17, high(char_F * 2)
    
bitmap_address_exit:
    pop   ZL
    pop   ZH
    ret

; Character bitmap data (7x5 pixel font)
character_bitmap_table:

char_A:  .db 0b00110, 0b01001, 0b01001, 0b01001, 0b01111, 0b01001, 0b01001, 0
char_B:  .db 0b01110, 0b01001, 0b01001, 0b01110, 0b01001, 0b01001, 0b01110, 0
char_C:  .db 0b00111, 0b01000, 0b01000, 0b01000, 0b01000, 0b01000, 0b00111, 0
char_D:  .db 0b01110, 0b01001, 0b01001, 0b01001, 0b01001, 0b01001, 0b01110, 0
char_E:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01111, 0
char_F:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01000, 0

char_0:  .db 0b01111, 0b01001, 0b01001, 0b01001, 0b01001, 0b01001, 0b01111, 0
char_1:  .db 0b00010, 0b00110, 0b01010, 0b00010, 0b00010, 0b00010, 0b00010, 0
char_2:  .db 0b01111, 0b00001, 0b00001, 0b01111, 0b01000, 0b01000, 0b01111, 0
char_3:  .db 0b01111, 0b00001, 0b00001, 0b00111, 0b00001, 0b00001, 0b01111, 0
char_4:  .db 0b01001, 0b01001, 0b01001, 0b01111, 0b00001, 0b00001, 0b00001, 0
char_5:  .db 0b01111, 0b01000, 0b01000, 0b01111, 0b00001, 0b00001, 0b01111, 0
char_6:  .db 0b01111, 0b01000, 0b01000, 0b01111, 0b01001, 0b01001, 0b01111, 0
char_7:  .db 0b01111, 0b00001, 0b00001, 0b00010, 0b00100, 0b00100, 0b00100, 0
char_8:  .db 0b01111, 0b01001, 0b01001, 0b01111, 0b01001, 0b01001, 0b01111, 0
char_9:  .db 0b01111, 0b01001, 0b01001, 0b01111, 0b00001, 0b00001, 0b01111, 0

character_table_end:

; Timer0 overflow ISR - completes latch pulse
timer0_latch_completion_isr:
    ldi   r16, T0_LATCH_DELAY
    sts   TCNT0, r16
    sbi   SR_PINREG, SR_LATCH_PIN
    lds   r16, TCCR0B
    andi  r16, ~((1<<CS02)|(1<<CS01)|(1<<CS00))
    sts   TCCR0B, r16
    reti

; -----------------------------------------------------------------------------
; Function: refresh_current_display_row
; Purpose: Complete single-row refresh cycle with automatic row advancement  
; 
; THIS IS THE MOST COMPLEX FUNCTION - LET'S BREAK IT DOWN:
; 
; WHAT LED MATRIX MULTIPLEXING MEANS:
; Our LED matrix has 7 rows and many columns, but we can only control one row
; at a time. To display a complete character, we rapidly cycle through rows:
; Row 0 → Row 1 → Row 2 → ... → Row 6 → Row 0 → ...
; Each time, we send the correct column data for that row, then move to the next row.
; If we do this fast enough (>50 times per second), human eyes see a stable image.
; 
; SHIFT REGISTER OPERATION:
; We use a shift register (STP08DP05) to control many LEDs with just 3 wires:
; 1. Send data bits one by one (87 bits total)
; 2. Generate a latch pulse to transfer data to LED outputs
; 3. Only the target block shows our character, others show nothing
; 
; ALGORITHM STEPS:
; 1. Fetch→Shift→Latch→Advance in optimized sequence
; 2. Get character bitmap data for current row
; 3. Send 80 column bits (16 blocks × 5 bits each)
; 4. Send 7 row selector bits (only 1 bit high = active row)
; 5. Generate precise latch pulse using Timer0
; 6. Advance to next row for following iteration
; 
; Registers: 
;   Input:  r21 (current row selector), r22 (current row index 0-6)  
;   Output: r21, r22 (advanced to next row with wraparound)
;   Temp:   r16, r17, r18, r19, r20 (automatically saved/restored)
; Hardware: Generates 87 clock pulses + precision latch pulse
; -----------------------------------------------------------------------------
refresh_current_display_row:
    ; PRESERVE WORKING REGISTERS ON STACK (LIFO order)  
    ; We need several registers for calculations. Save them so we don't
    ; corrupt the main program's data.
    push  r16                          ; Timer control register
    push  r17                          ; General purpose calculations
    push  r18                          ; Loop counter for shift register
    push  r19                          ; Current block index (0-15)

fetch_row_bitmap_data:
    ; GET CURRENT CHARACTER TO DISPLAY FROM KEYPAD INPUT
    lds   r16, current_char            ; Load ASCII character from SRAM
                                       ; This was set by keypad scanning
    call  get_character_bitmap_address ; Convert ASCII to bitmap memory address
                                       ; Returns r17:r16 = program memory address
    mov   ZL, r16                      ; Z register low byte = address low byte
    mov   ZH, r17                      ; Z register high byte = address high byte
                                       ; Z register now points to character bitmap
    
    ; APPLY BOUNDS CHECKING TO PREVENT BUFFER OVERFLOW
    BOUNDARY_CHECK_ROW r22             ; Macro: clamp r22 to valid range (0-6)
                                       ; Prevents reading beyond character data
    
    ; CALCULATE FINAL BYTE ADDRESS FOR CURRENT ROW
    ; Each character has 8 bytes (rows 0-7), we want byte [r22]
    add   ZL, r22                      ; Add row offset to address low byte
    adc   ZH, r1                       ; Add carry to address high byte (r1=0)
                                       ; Z now points to specific row bitmap
    lpm   r20, Z                       ; lpm = "Load Program Memory"
                                       ; Load 5-bit row pattern into r20

transmit_column_data:
    ; TRANSMIT COLUMN DATA FOR ALL 16 BLOCKS
    ; The LED driver controls 80 outputs in 16 blocks of 5 outputs each.
    ; Only TARGET_BLOCK gets our character data, others get zeros.
    ldi   r18, COLUMN_CYCLES           ; COLUMN_CYCLES = 16 (number of blocks)
    clr   r19                          ; Current block index (0-15)
    
column_transmission_loop:
    ; CHECK IF THIS IS THE TARGET BLOCK
    ; Only one block shows our character, others are dark
    cpi   r19, TARGET_BLOCK            ; Compare current block with TARGET_BLOCK (14)
    brne  send_zero_block              ; If not target block, send zeros
    
send_character_block:
    ; SEND THE ACTUAL CHARACTER DATA FOR THIS BLOCK
    ; Use our SHIFT_BIT_OUT macro to send 5 bits with proper timing
    SHIFT_BIT_OUT r20, 0               ; Transmit bit 0 (rightmost column)
    SHIFT_BIT_OUT r20, 1               ; Transmit bit 1  
    SHIFT_BIT_OUT r20, 2               ; Transmit bit 2 (middle column)
    SHIFT_BIT_OUT r20, 3               ; Transmit bit 3
    SHIFT_BIT_OUT r20, 4               ; Transmit bit 4 (leftmost column)
    rjmp  next_block                   ; Jump to block counter update
    
send_zero_block:
    ; SEND ZEROS FOR NON-TARGET BLOCKS
    ; These blocks will be dark (r1 is always 0)
    SHIFT_BIT_OUT r1, 0                ; Transmit 0 (r1 bit 0 = 0)
    SHIFT_BIT_OUT r1, 1                ; Transmit 0 (r1 bit 1 = 0)
    SHIFT_BIT_OUT r1, 2                ; Transmit 0 (r1 bit 2 = 0)
    SHIFT_BIT_OUT r1, 3                ; Transmit 0 (r1 bit 3 = 0)
    SHIFT_BIT_OUT r1, 4                ; Transmit 0 (r1 bit 4 = 0)
    
next_block:
    inc   r19                          ; Move to next block (0→1→2→...→15)
    dec   r18                          ; Decrement remaining blocks counter
    brne  column_transmission_loop     ; Continue until all 16 blocks sent

insert_row_separator:
    ; INSERT CLEAN SEPARATOR BETWEEN COLUMN AND ROW DATA  
    ; This ensures clean timing between different data types
    cbi   SR_PORT, SR_DATA_PIN         ; Ensure data line is LOW (0)
    sbi   SR_PINREG, SR_CLOCK_PIN      ; Generate separator clock pulse (toggle)
    sbi   SR_PINREG, SR_CLOCK_PIN      ; Complete clock cycle (toggle back)

transmit_row_select_data:
    ; TRANSMIT 7-BIT ROW SELECTOR (one-hot encoding, MSB first)
    ; Only one bit is HIGH at a time, indicating which physical row to activate
    ; We shift MSB first because that's how the shift register works
    SHIFT_BIT_OUT r21, 6               ; Row 6 selector bit (MSB)
    SHIFT_BIT_OUT r21, 5               ; Row 5 selector bit
    SHIFT_BIT_OUT r21, 4               ; Row 4 selector bit  
    SHIFT_BIT_OUT r21, 3               ; Row 3 selector bit
    SHIFT_BIT_OUT r21, 2               ; Row 2 selector bit
    SHIFT_BIT_OUT r21, 1               ; Row 1 selector bit
    SHIFT_BIT_OUT r21, 0               ; Row 0 selector bit (LSB)

advance_to_next_row:
    dec   r22
    lsr   r21
    brne  initiate_hardware_latch
    ldi   r22, MAX_ROW_INDEX
    ldi   r21, ROW_RESET_MASK

initiate_hardware_latch:
    sbi   SR_PINREG, SR_LATCH_PIN
    lds   r16, TCCR0B
    ori   r16, TIMER0_PRESCALER
    sts   TCCR0B, r16
    
    pop   r19
    pop   r18
    pop   r17
    pop   r16
    ret
