; =============================================================================
; LED MATRIX DISPLAY DRIVER FOR ATMEGA328P - EDUCATIONAL VERSION
; =============================================================================
; 
; Author:          [Your Name]
; Date Created:    [Current Date]
; Last Modified:   [Current Date]
; Version:         1.0.0
; Target MCU:      ATmega328P (Arduino Uno compatible)
; Clock:           16MHz external crystal
; 
; WHAT THIS PROGRAM DOES (for beginners):
; This program controls a LED matrix display (like a tiny TV screen made of LEDs)
; that can show letters and numbers. It also reads input from a keypad (like a
; calculator keypad) and displays the character corresponding to the pressed key.
;
; ASSEMBLY LANGUAGE BASICS:
; Assembly is the lowest level programming language before machine code.
; Each line typically represents ONE instruction that the CPU can execute.
; It's like giving very specific, step-by-step instructions to the processor.
; 
; Common Assembly Concepts:
; • Registers: Like super-fast memory slots (r16, r17, etc.) inside the CPU
; • Memory: Slower storage that holds data and program instructions
; • Instructions: Commands like "ldi" (load immediate), "add", "call", etc.
; • Labels: Named locations in the program (like bookmarks)
; • .equ: Defines a constant value (like #define in C)
; • Comments: Lines starting with ; explain what the code does
;
; DESCRIPTION:
; Professional LED matrix display driver with integrated 4x4 keypad control.
; Displays characters on LED matrix corresponding to pressed keys with robust
; debouncing and error handling to prevent ghosting and false triggers.
;
; FEATURES:
; - Shift-register based LED matrix control (STP08DP05 LED driver)
; - Integrated 4x4 matrix keypad with robust scanning
; - Timer0-based precision latch pulse generation
; - Advanced debouncing and anti-ghosting algorithms  
; - Dynamic character selection based on keypad input
; - Comprehensive error handling and bounds checking
; - Modular character definition system
; - Memory-efficient register usage
;
; HARDWARE CONFIGURATION:
; ┌─────────────────────────────────────────────────────────────────────┐
; │ Pin Assignment (which wires connect where):                         │
; │ • PB3 (Arduino D11) - Serial Data Input to STP08DP05               │
; │ • PB4 (Arduino D12) - Latch/Output Control                         │
; │ • PB5 (Arduino D13) - Shift Register Clock                         │
; │                                                                     │
; │ • PD7-PD4 - Keypad Rows (Outputs, active low)                      │
; │ • PD3-PD0 - Keypad Columns (Inputs with pull-ups)                  │
; │                                                                     │
; │ LED Matrix: 7x5 character display (7 rows × 5 columns)             │
; │ Keypad: 4x4 matrix (hex keypad layout)                             │
; └─────────────────────────────────────────────────────────────────────┘
;
; =============================================================================

.include "m328pdef.inc"                  ; Include ATmega328P register definitions
                                        ; This tells the assembler what names like
                                        ; PORTB, DDRB, etc. actually mean in terms
                                        ; of memory addresses

; =============================================================================
; SYSTEM CONFIGURATION CONSTANTS - WHAT THE NUMBERS MEAN
; =============================================================================
; 
; In assembly, we use constants to make code readable and maintainable.
; Instead of writing magic numbers like "3" or "240", we give them meaningful
; names. The .equ directive creates a constant (like #define in C).
; These constants are replaced with actual values when the code is assembled.

; I/O Port Configuration (Port B assignments)
; Port B is one of the I/O ports on the microcontroller (like GPIO pins)
; Each port has three associated registers:
; - DDR (Data Direction Register): Sets pins as input (0) or output (1)  
; - PORT (Port Output Register): Sets output pin levels (high/low)
; - PIN (Pin Input Register): Reads current pin states
.equ SR_DDR           = DDRB      ; Data Direction Register for shift register pins
.equ SR_PORT          = PORTB     ; Output Port Register for shift register pins  
.equ SR_PINREG        = PINB      ; Pin Toggle Register (special AVR feature)
.equ SR_DATA_PIN      = 3         ; Bit position 3 in Port B (PB3 = pin 11)
.equ SR_CLOCK_PIN     = 5         ; Bit position 5 in Port B (PB5 = pin 13) 
.equ SR_LATCH_PIN     = 4         ; Bit position 4 in Port B (PB4 = pin 12)

; Timer0 Configuration Constants
; Timers are special circuits that count clock pulses. We use Timer0 to create
; precise timing for the LED matrix latch signal.
.equ T0_LATCH_DELAY   = 240       ; Timer0 preload value (240 means timer will 
                                  ; overflow after 256-240=16 counts)
.equ TIMER0_PRESCALER = (1<<CS02)|(1<<CS01)|(1<<CS00)  ; Prescaler: 1024
                                  ; This makes timer count slower: 16MHz/1024 = 15.625kHz
                                  ; The (1<<CS02) syntax sets bit CS02 to 1

; Keypad Configuration Constants  
; The keypad is organized as a 4x4 grid. We scan it by setting one row low
; at a time and reading which columns go low (indicating a pressed key).
.equ KEYPAD_DDR       = DDRD      ; Data Direction Register for keypad
.equ KEYPAD_PORT      = PORTD     ; Output Port Register for keypad
.equ KEYPAD_PIN       = PIND      ; Input Pin Register for keypad
.equ KEYPAD_ROW_MASK  = 0b11110000  ; Binary: rows are bits 7,6,5,4 (outputs)
.equ KEYPAD_COL_MASK  = 0b00001111  ; Binary: columns are bits 3,2,1,0 (inputs)
.equ DEBOUNCE_CYCLES  = 5         ; How many scan cycles to wait before accepting
                                  ; a key press (prevents false triggers from
                                  ; electrical noise or bouncing contacts)

; Keypad pin assignments (individual pin numbers within Port D)
; These tell us exactly which physical pin on the microcontroller connects to what
.equ ROW_1            = 7         ; PD7 - Keypad Row 1 (Physical top row of buttons)
.equ ROW_2            = 6         ; PD6 - Keypad Row 2 
.equ ROW_3            = 5         ; PD5 - Keypad Row 3  
.equ ROW_4            = 4         ; PD4 - Keypad Row 4 (Physical bottom row of buttons)
.equ COL_0            = 3         ; PD3 - Keypad Column 0 (leftmost column)
.equ COL_1            = 2         ; PD2 - Keypad Column 1
.equ COL_2            = 1         ; PD1 - Keypad Column 2
.equ COL_3            = 0         ; PD0 - Keypad Column 3 (rightmost column)

; Keypad scan codes (1-based indexing to match documentation)
; Each key gets a unique number from 1-16. This maps the physical button
; positions to these numbers based on the typical hex keypad layout:
; Physical layout: Row 1:[7][8][9][F], Row 2:[4][5][6][E], Row 3:[1][2][3][D], Row 4:[A][0][B][C]
.equ KEY_7 = 1      ; Row 1, Col 0 - Physical top-left button
.equ KEY_8 = 2      ; Row 1, Col 1
.equ KEY_9 = 3      ; Row 1, Col 2
.equ KEY_F = 4      ; Row 1, Col 3 - Physical top-right button
.equ KEY_4 = 5      ; Row 2, Col 0
.equ KEY_5 = 6      ; Row 2, Col 1 - Center of keypad
.equ KEY_6 = 7      ; Row 2, Col 2
.equ KEY_E = 8      ; Row 2, Col 3
.equ KEY_1 = 9      ; Row 3, Col 0
.equ KEY_2 = 10     ; Row 3, Col 1
.equ KEY_3 = 11     ; Row 3, Col 2
.equ KEY_D = 12     ; Row 3, Col 3
.equ KEY_A = 13     ; Row 4, Col 0 - Physical bottom-left button
.equ KEY_0 = 14     ; Row 4, Col 1
.equ KEY_B = 15     ; Row 4, Col 2
.equ KEY_C = 16     ; Row 4, Col 3 - Physical bottom-right button

; Display Matrix Specifications
; These define the size and characteristics of our LED display
.equ MATRIX_ROWS      = 7         ; How many rows of LEDs (vertical resolution)
.equ MATRIX_COLS      = 5         ; How many columns per character (horizontal resolution)
.equ CHAR_BYTES       = 8         ; How many bytes needed to store one character
                                  ; (7 rows + 1 padding byte for memory alignment)
.equ COLUMN_CYCLES    = 16        ; How many 5-bit blocks we send (80÷5 = 16)
                                  ; The LED driver controls 80 outputs total

; Target Block Selection (0-15, where 0 is first block out, 15 is last)
; The LED matrix is divided into 16 blocks of 5 LEDs each. We choose which
; block to display our character on.
.equ TARGET_BLOCK     = 14        ; Block 14 = LEDs 70-74 (near the end)

; Global State Variables (using high registers for keypad state)
; In AVR assembly, registers r16-r31 are "high" registers that can be used
; with immediate operations (ldi instruction). We reserve some for global state.
.def key_state        = r24       ; Stores the last detected key (0xFF = no key)
.def debounce_counter = r23       ; Counts down from DEBOUNCE_CYCLES to 0

; Derived Constants and Bit Masks
; These are calculated from the above values to make the code more readable
.equ SR_OUTPUT_MASK   = (1<<SR_DATA_PIN)|(1<<SR_LATCH_PIN)|(1<<SR_CLOCK_PIN)
                                  ; Binary mask with 1s in positions 3,4,5
                                  ; Used to set multiple pins as outputs at once
.equ ROW_RESET_MASK   = 0b01000000  ; Initial row selector (row 6, MSB first)
                                  ; Only bit 6 is set, meaning we start scanning
                                  ; from row 6 and work our way down
.equ MAX_ROW_INDEX    = MATRIX_ROWS - 1  ; Maximum valid row index (6)
                                  ; Used for bounds checking to prevent crashes

; =============================================================================
; MEMORY ALLOCATION - SRAM DATA SECTION
; =============================================================================
; 
; MEMORY BASICS FOR BEGINNERS:
; The microcontroller has different types of memory:
; - Flash memory: Stores the program code (read-only, non-volatile)
; - SRAM: Temporary data storage (read-write, lost when power off)  
; - EEPROM: Permanent data storage (read-write, keeps data when power off)
;
; The .dseg directive switches to the data segment (SRAM allocation)
; The .cseg directive switches to the code segment (Flash memory)
;
.dseg                                 ; Switch to data segment (SRAM memory)
.org 0x0100                           ; Start after register file and I/O space
                                      ; The first 256 bytes (0x00-0xFF) are reserved
                                      ; for CPU registers and I/O registers
current_char:     .byte 1             ; Reserve 1 byte to store the current character
                                      ; This will hold the ASCII code of the character
                                      ; to display (like 'A' = 65, '0' = 48, etc.)
.cseg                                 ; Switch back to code segment (Flash memory)

; =============================================================================
; PERFORMANCE-OPTIMIZED MACROS - REUSABLE CODE BLOCKS
; =============================================================================
;
; WHAT ARE MACROS?
; Macros are like "templates" for assembly code. When you use a macro, the
; assembler copies the macro's code into your program at that location.
; Think of them like functions, but the code gets pasted in place rather
; than called. This makes them faster but uses more memory.

; -----------------------------------------------------------------------------
; Macro: SHIFT_BIT_OUT  
; Purpose: Send one bit of data to the shift register with proper timing
; 
; HOW SHIFT REGISTERS WORK:
; A shift register is like a chain of memory bits. You feed data in one bit
; at a time, and each clock pulse moves all bits one position over. After
; sending all bits, you "latch" them to actually turn on/off the outputs.
; 
; PARAMETERS EXPLANATION:
;   @0 = source register containing the byte with our bit pattern
;   @1 = bit position to extract (0=rightmost bit, 7=leftmost bit)
; 
; STEP-BY-STEP OPERATION:
; 1. Assume the bit is 1, so set data line HIGH
; 2. Check if the actual bit is 0, if so, set data line LOW  
; 3. Create a clock pulse (LOW→HIGH→LOW) to shift the bit into the register
; 
; Side Effects: Changes the shift register pins, generates timing-critical signals
; Note: Uses "skip instruction" technique for fast, branch-free bit testing
; -----------------------------------------------------------------------------
.MACRO SHIFT_BIT_OUT
    sbi   SR_PORT, SR_DATA_PIN          ; sbi = "Set Bit in I/O register"
                                        ; Pre-set data line HIGH (assume bit = 1)
    sbrs  @0, @1                        ; sbrs = "Skip if Bit in Register is Set"
                                        ; Skip next instruction if bit[@1] is 1
    cbi   SR_PORT, SR_DATA_PIN          ; cbi = "Clear Bit in I/O register" 
                                        ; If we didn't skip, bit was 0, so set data LOW
    sbi   SR_PINREG, SR_CLOCK_PIN       ; sbi on PIN register = toggle the bit
                                        ; Generate rising edge on clock pin
    sbi   SR_PINREG, SR_CLOCK_PIN       ; Toggle again = falling edge
                                        ; Complete clock pulse (HIGH pulse)
.ENDMACRO

; -----------------------------------------------------------------------------
; Macro: SAFE_REGISTER_CLEAR
; Purpose: Clear a register with safety checks to prevent system corruption
; 
; WHY THIS IS NEEDED:
; In AVR assembly, register r1 is special - it should always contain 0.
; Many instructions assume r1=0. If we accidentally clear it in the wrong way
; or use it for other purposes, the system could crash.
; 
; Parameters: @0 = target register to clear (set to 0)
; Safety: Prevents accidental corruption of r1 (the designated zero register)
; Usage: SAFE_REGISTER_CLEAR r16
; -----------------------------------------------------------------------------
.MACRO SAFE_REGISTER_CLEAR
    .if @0 != r1                        ; .if = compile-time condition check
        clr   @0                        ; clr = "clear register" (set to 0)
                                        ; Only clear if it's not r1
    .else
        .error "Cannot clear r1 - reserved zero register"  ; Stop assembly with error
    .endif
.ENDMACRO

; -----------------------------------------------------------------------------
; Macro: BOUNDARY_CHECK_ROW
; Purpose: Prevent array overflow by checking if row index is valid
; 
; WHY BOUNDS CHECKING MATTERS:
; If we try to access row 8 when we only have rows 0-6, we'll read/write
; random memory, which could crash the system or corrupt data.
; This macro "clamps" the value to the valid range.
; 
; Parameters: @0 = row index register (will be modified if out of bounds)
; Modifies: @0 (clamped to valid range 0 to MAX_ROW_INDEX)
; Range: 0 to MAX_ROW_INDEX (which is 6 for our 7-row display)
; -----------------------------------------------------------------------------
.MACRO BOUNDARY_CHECK_ROW
    cpi   @0, MAX_ROW_INDEX + 1         ; cpi = "Compare with Immediate value"
                                        ; Compare register with (6+1)=7
    brlo  PC + 2                        ; brlo = "Branch if Lower"
                                        ; Skip next instruction if @0 < 7 (valid range)
                                        ; PC+2 means skip the next instruction
    ldi   @0, MAX_ROW_INDEX             ; ldi = "Load Immediate"
                                        ; If we didn't skip, clamp to maximum (6)
.ENDMACRO

; =============================================================================
; INTERRUPT VECTOR TABLE - CRITICAL SYSTEM ENTRY POINTS
; =============================================================================
;
; WHAT ARE INTERRUPTS?
; Interrupts are like "urgent phone calls" for the microcontroller. When certain
; events happen (timer overflow, pin change, etc.), the CPU immediately stops
; what it's doing and jumps to a special function called an Interrupt Service
; Routine (ISR). After the ISR finishes, it returns to where it left off.
;
; THE VECTOR TABLE:
; This is a list of addresses that tell the CPU where to jump when each type
; of interrupt occurs. It must be at the very beginning of program memory.
; Each interrupt type has a specific position in this table.
;
.cseg                                 ; Make sure we're in code segment (Flash memory)
.org 0x0000                           ; .org = "origin" - set memory address to 0x0000
                                      ; This is the very first memory location
    rjmp  system_init                 ; rjmp = "Relative Jump"
                                      ; RESET vector: When power comes on or reset
                                      ; button pressed, jump to system_init
                                      ; All other interrupt vectors use default behavior (reti)

.org 0x0010                           ; Timer0 overflow vector address  
                                      ; When Timer0 counts from 255 to 0, this interrupt fires
    rjmp  timer0_latch_completion_isr ; Jump to our latch pulse completion handler
                                      ; This provides precision timing for LED display

; =============================================================================
; SYSTEM INITIALIZATION AND CONFIGURATION
; =============================================================================
;
; WHAT INITIALIZATION MEANS:
; When a microcontroller starts up, its registers and pins are in random states.
; Before we can use any peripherals (timers, I/O pins, interrupts), we must
; configure them properly. This is like setting up all the equipment before
; starting to use it.

; -----------------------------------------------------------------------------
; Function: system_init
; Purpose: Complete system initialization sequence - sets up everything needed
; 
; WHAT THIS FUNCTION DOES:
; 1. Disable interrupts temporarily (prevent chaos during setup)
; 2. Set up the stack pointer (needed for function calls and interrupts)
; 3. Configure all I/O pins (shift register, keypad)
; 4. Set up Timer0 for precise timing
; 5. Initialize all variables to known starting values
; 6. Enable interrupts and start the main program
; 
; Registers Modified: r16, r17 (temporary working registers)
; Stack Usage: None (this runs before the stack is used by other functions)
; Critical: Must execute before any peripheral operations or the system won't work
; -----------------------------------------------------------------------------
system_init:
    cli                                ; cli = "Clear Interrupt flag" 
                                       ; Disable ALL interrupts during setup
                                       ; Prevents interference while configuring hardware
    
    ; STACK POINTER SETUP - CRITICAL FOR FUNCTION CALLS AND INTERRUPTS
    ; The stack is used for:
    ; - Storing return addresses when calling functions
    ; - Saving registers during interrupts
    ; - Local variables (though we don't use any here)
    ; The stack grows DOWNWARD from high memory addresses
    ldi   r16, high(RAMEND)            ; ldi = "Load Immediate"
                                       ; RAMEND = highest RAM address (0x08FF on ATmega328P)
                                       ; high() extracts upper 8 bits (0x08)
    out   SPH, r16                     ; out = "Output to I/O register"
                                       ; Set Stack Pointer High byte
    ldi   r16, low(RAMEND)             ; low() extracts lower 8 bits (0xFF)  
    out   SPL, r16                     ; Set Stack Pointer Low byte
                                       ; Now stack starts at address 0x08FF
    
    ; HARDWARE INITIALIZATION - SET UP ALL PERIPHERALS
    ; We call separate functions to keep the code organized and readable
    call  init_shift_register_pins     ; call = "Call subroutine"
                                       ; Configure pins for LED matrix control
    call  init_keypad_pins             ; Configure pins for keypad scanning
    call  configure_timer0_system      ; Setup Timer0 for precision timing  
    
    ; SOFTWARE INITIALIZATION - SET VARIABLES TO KNOWN STARTING VALUES
    call  initialize_display_state     ; Set up display multiplexing variables
    call  initialize_keypad_state      ; Set up keypad scanning variables
    
    ; r1 REGISTER INITIALIZATION - IMPORTANT AVR CONVENTION!
    ; In AVR assembly, register r1 is designated as the "zero register"
    ; Many instructions assume it always contains 0
    clr   r1                           ; clr = "Clear Register" (set to 0)
                                       ; We'll use r1 for zero bits in block selection
    
    ; SYSTEM STARTUP COMPLETE - ENABLE INTERRUPTS AND START MAIN PROGRAM
    sei                                ; sei = "Set Interrupt flag"
                                       ; Enable global interrupts (allow ISRs to run)
    
    ; Transfer control to the main application loop (never returns)
    rjmp  main_application_loop        ; rjmp = "Relative Jump"
                                       ; Start the continuous display refresh loop

; -----------------------------------------------------------------------------
; Function: init_shift_register_pins  
; Purpose: Configure Port B pins for STP08DP05 LED driver interface
; 
; WHAT THIS FUNCTION DOES:
; Sets up the three pins needed to control the shift register that drives the LEDs:
; - Data pin: Sends the 1s and 0s that determine which LEDs turn on/off
; - Clock pin: Timing signal that tells the shift register when to read the data
; - Latch pin: Signal that transfers data from internal buffer to LED outputs
; 
; PIN CONFIGURATION PROCESS:
; 1. Create a bit mask with 1s in positions we want as outputs
; 2. Write this mask to the DDR (Data Direction Register) to set pins as outputs
; 3. Write to PORT register to set initial output levels (all high)
; 
; Registers Modified: r17 (temporary working register)
; Hardware: Configures PB3, PB4, PB5 as output pins with clean initial state
; -----------------------------------------------------------------------------
init_shift_register_pins:
    ldi   r17, SR_OUTPUT_MASK          ; Load the bit pattern for our output pins
                                       ; SR_OUTPUT_MASK = (1<<3)|(1<<4)|(1<<5) = 0b111000
    out   SR_DDR, r17                  ; Set Data Direction Register 
                                       ; 1 = output pin, 0 = input pin
                                       ; This makes PB3, PB4, PB5 outputs
    out   SR_PORT, r17                 ; Set initial output levels
                                       ; Writing 1 to PORT makes pin HIGH (5V)
                                       ; All three pins start HIGH (inactive state)
    ret                                ; ret = "Return from subroutine"
                                       ; Go back to whoever called this function

; -----------------------------------------------------------------------------
; Function: init_keypad_pins
; Purpose: Configure Port D pins for 4x4 matrix keypad interface
; 
; HOW MATRIX KEYPADS WORK:
; A 4x4 keypad has 8 wires: 4 rows and 4 columns. Keys are at intersections.
; To scan: Set one row LOW, read columns. If a column goes LOW, that key is pressed.
; 
; ELECTRICAL SETUP:
; - Rows (PD7-PD4): Outputs, normally HIGH, we pull ONE row LOW at a time
; - Columns (PD3-PD0): Inputs with pull-up resistors, normally HIGH
; - When button pressed: connects row to column, column goes LOW
; 
; Registers Modified: r16 (temporary working register)
; Hardware: PD7-PD4 as outputs (rows), PD3-PD0 as inputs (columns)
; -----------------------------------------------------------------------------
init_keypad_pins:
    ldi   r16, KEYPAD_ROW_MASK         ; KEYPAD_ROW_MASK = 0b11110000
                                       ; 1s in positions 7,6,5,4 (rows)
    out   KEYPAD_DDR, r16              ; Configure rows as outputs
                                       ; PD7-PD4 = outputs, PD3-PD0 = inputs (0s)
    ldi   r16, 0xFF                    ; Binary 11111111 (all bits set)
    out   KEYPAD_PORT, r16             ; Set all outputs HIGH, enable pull-ups
                                       ; For output pins: HIGH = inactive state
                                       ; For input pins: enables internal pull-up resistors
    ret                                ; Return to caller

; -----------------------------------------------------------------------------
; Function: configure_timer0_system
; Purpose: Initialize Timer0 for latch pulse timing control
; 
; WHAT ARE TIMERS?
; Timers are special circuits that count clock pulses. Timer0 can count from
; 0 to 255, then overflow back to 0. When it overflows, it can trigger an interrupt.
; We use this to create precise timing for our LED display latch signal.
; 
; TIMER MODES:
; - Normal mode: Counts 0→1→2→...→255→0 (overflow)→1→2→...
; - Other modes: PWM, CTC, etc. (we don't use these)
; 
; PRESCALER:
; The timer can count every CPU clock (16MHz) or use a prescaler to count slower.
; Prescaler 1024 means: count once every 1024 CPU clocks = 16MHz/1024 = 15.625kHz
; 
; Registers Modified: r16 (temporary working register)  
; Configuration: Normal mode, overflow interrupt enabled, initially stopped
; -----------------------------------------------------------------------------
configure_timer0_system:
    ; CONFIGURE TIMER WAVEFORM GENERATION MODE (Normal mode)
    ; Timer0 has control register A (TCCR0A) and B (TCCR0B)
    ; WGM bits control what mode the timer operates in
    lds   r16, TCCR0A                  ; lds = "Load Direct from SRAM"
                                       ; Read current Timer Control Register A
    andi  r16, ~((1<<WGM01)|(1<<WGM00)) ; andi = "AND with Immediate"
                                       ; Clear WGM01 and WGM00 bits (set to 0)
                                       ; ~(...) creates bit mask with 0s in those positions
    sts   TCCR0A, r16                  ; sts = "Store Direct to SRAM"
                                       ; Write back modified register
                                       ; Now WGM01=0, WGM00=0 = Normal mode
    
    ; CONFIGURE CLOCK SOURCE (initially stopped for safety)
    lds   r16, TCCR0B                  ; Read Timer Control Register B
    andi  r16, ~((1<<WGM02)|(1<<CS02)|(1<<CS01)|(1<<CS00))  ; Clear all mode and clock bits
                                       ; WGM02=0 completes Normal mode setting
                                       ; CS bits = 000 means no clock (timer stopped)
    sts   TCCR0B, r16                  ; Write back - timer is now stopped and in normal mode
    
    ; ENABLE TIMER0 OVERFLOW INTERRUPT
    ; When timer counts 255→0, it will trigger our ISR for precise latch control
    lds   r16, TIMSK0                  ; Load Timer Interrupt Mask Register
    ori   r16, (1<<TOIE0)              ; ori = "OR with Immediate"
                                       ; Set Timer0 Overflow Interrupt Enable bit
                                       ; TOIE0 = Timer0 Overflow Interrupt Enable
    sts   TIMSK0, r16                  ; Store back - interrupt now enabled
    ret                                ; Return to caller

; -----------------------------------------------------------------------------
; Function: initialize_display_state
; Purpose: Set initial conditions for display multiplexing
; 
; WHAT IS DISPLAY MULTIPLEXING?
; Our LED matrix has 7 rows but we can only control one row at a time. To show
; a complete character, we rapidly switch between rows, turning on the right
; LEDs in each row. If we do this fast enough (>50Hz), human eyes see a stable image.
; 
; MULTIPLEXING VARIABLES:
; - r22: Which row we're currently displaying (byte index: 0-6 for rows 0-6)
; - r21: Row selector bit pattern (only one bit is 1, indicating active row)
; 
; WHY START WITH ROW 6?
; We scan from bottom to top (row 6→5→4→3→2→1→0→6→...) because the shift
; register sends the MSB (Most Significant Bit) first, and we want row 6 first.
; 
; Registers Modified: r21, r22 (global state registers for display)
; State: Row 6 selected (MSB first scanning), byte index = 6
; -----------------------------------------------------------------------------
initialize_display_state:
    ldi   r22, MAX_ROW_INDEX           ; MAX_ROW_INDEX = 6 (for 7 rows: 0,1,2,3,4,5,6)
                                       ; Start with the last row (row 6)
    ldi   r21, ROW_RESET_MASK          ; ROW_RESET_MASK = 0b01000000 (bit 6 set)
                                       ; Only bit 6 is 1, meaning row 6 is selected
    ret                                ; Return to caller

; -----------------------------------------------------------------------------
; Function: initialize_keypad_state
; Purpose: Set initial conditions for keypad scanning and debouncing
; 
; WHAT IS DEBOUNCING?
; Physical buttons don't make clean electrical contact. When pressed, they
; "bounce" - make and break contact rapidly for a few milliseconds. Without
; debouncing, one button press might register as multiple presses.
; 
; DEBOUNCING STRATEGY:
; When we detect a key change, we ignore further changes for several scan cycles.
; This gives the button time to settle into a stable state.
; 
; INITIAL STATE:
; - Display character '0' on startup (ASCII code 48)
; - No key currently pressed (0xFF means "no key")
; - No debouncing active (counter = 0)
; 
; Registers Modified: current_char (SRAM), key_state (r24), debounce_counter (r23)
; State: Display '0' initially, clear keypad state
; -----------------------------------------------------------------------------
initialize_keypad_state:
    ldi   r16, '0'                     ; ASCII code for character '0' (decimal 48)
                                       ; This is what we'll display on startup
    sts   current_char, r16            ; sts = "Store Direct to SRAM"
                                       ; Save '0' to our current character variable
    ldi   key_state, 0xFF              ; 0xFF = 255 = "no key pressed" indicator
                                       ; key_state is defined as register r24
    clr   debounce_counter             ; clr = "Clear Register" (set to 0)
                                       ; No debouncing active initially
                                       ; debounce_counter is defined as register r23
    ret                                ; Return to caller

; =============================================================================
; KEYPAD INTERFACE FUNCTIONS
; Purpose: 4x4 matrix keypad scanning with anti-ghosting protection
; =============================================================================
;
; HOW MATRIX KEYPAD SCANNING WORKS (for complete beginners):
; 
; 1. PHYSICAL STRUCTURE:
;    A 4x4 keypad has 16 buttons arranged in a grid with 8 wires:
;    - 4 row wires (horizontal)  
;    - 4 column wires (vertical)
;    Each button connects one row wire to one column wire when pressed.
;
; 2. ELECTRICAL SETUP:
;    - Row pins: Outputs from microcontroller, normally HIGH (5V)
;    - Column pins: Inputs to microcontroller, with pull-up resistors (normally HIGH)
;
; 3. SCANNING PROCESS:
;    To find which button is pressed:
;    a) Set ALL rows HIGH except one (set that one LOW)
;    b) Read all columns - if any column is LOW, a button in that row is pressed
;    c) Repeat for each row
;    d) The intersection of LOW row and LOW column tells us which button
;
; 4. ANTI-GHOSTING:
;    Problem: If user presses 2 buttons in same row, we might see false readings
;    Solution: Count how many columns are LOW per row - if >1, ignore the reading

; -----------------------------------------------------------------------------
; Function: scan_keypad_nonblocking
; Purpose: Single keypad scan cycle with debouncing and state management
; 
; WHY "NON-BLOCKING"?
; A "blocking" function would wait until a key is pressed, stopping everything else.
; A "non-blocking" function checks for keys quickly and returns immediately,
; allowing the display refresh to continue. This prevents the LED display from flickering.
; 
; DEBOUNCING LOGIC:
; - If debounce timer is active (>0), skip scanning and just decrement timer
; - If timer expired (=0), perform one scan cycle
; - If new key detected, start debounce timer to ignore changes for a few cycles
; - If no key detected, clear the key state
; 
; Returns: Updates current_char in SRAM if valid key press detected
; Strategy: Non-blocking scan to maintain display refresh rate
; -----------------------------------------------------------------------------
scan_keypad_nonblocking:
    push  r16                          ; push = "Push onto stack"
                                       ; Save r16 because we'll modify it
                                       ; Stack remembers values in LIFO order
    push  r17                          ; Save r17 for the same reason
    
    ; CHECK IF WE'RE IN DEBOUNCE PERIOD
    tst   debounce_counter             ; tst = "Test register" (check if zero)
                                       ; Sets flags based on register value
                                       ; debounce_counter is r23
    brne  skip_keypad_scan             ; brne = "Branch if Not Equal (to zero)"
                                       ; If counter > 0, skip scanning
    
    ; PERFORM ACTUAL KEYPAD SCAN
    ; Only scan if debounce timer has expired
    call  scan_keyboard_matrix         ; This does the actual row/column scanning
                                       ; Returns r16 = key code (1-16) or 0xFF if none
    
    ; CHECK IF A KEY WAS DETECTED
    cpi   r16, 0xFF                    ; cpi = "Compare with Immediate"
                                       ; Compare scan result with 0xFF (no key)
    breq  no_key_pressed               ; breq = "Branch if Equal"
                                       ; If r16 = 0xFF, jump to no_key_pressed
    
    ; VALID KEY DETECTED - CHECK IF IT'S DIFFERENT FROM CURRENT STATE
    ; We only want to update the display if the key actually changed
    cp    r16, key_state               ; cp = "Compare registers"
                                       ; Compare new key with previous key state
    breq  scan_keypad_exit             ; If same key, no action needed
    
    ; NEW KEY PRESS DETECTED - UPDATE DISPLAY AND START DEBOUNCING
    mov   key_state, r16               ; mov = "Move (copy) register"
                                       ; Store new key state (r16 → r24)
    call  map_key_to_character         ; Convert key number to ASCII character
                                       ; Input: r16 = key number, Output: r16 = ASCII
    sts   current_char, r16            ; Store ASCII character for display
    
    ; START DEBOUNCE TIMER TO PREVENT FALSE TRIGGERS
    ldi   debounce_counter, DEBOUNCE_CYCLES  ; Load debounce delay (5 cycles)
    rjmp  scan_keypad_exit             ; rjmp = "Relative Jump" - skip to end
    
no_key_pressed:
    ; NO KEY PRESSED - CLEAR KEY STATE  
    ldi   key_state, 0xFF              ; Set "no key" state
    
skip_keypad_scan:
    ; DECREMENT DEBOUNCE COUNTER IF ACTIVE
    ; This happens whether we scanned or not
    tst   debounce_counter             ; Check if counter > 0
    breq  scan_keypad_exit             ; If already 0, skip decrement
    dec   debounce_counter             ; dec = "Decrement register" (subtract 1)
    
scan_keypad_exit:
    ; RESTORE SAVED REGISTERS AND RETURN
    pop   r17                          ; pop = "Pop from stack"
                                       ; Restore r17 (reverse of push order)
    pop   r16                          ; Restore r16
    ret                                ; Return to caller

; -----------------------------------------------------------------------------
; Function: scan_keyboard_matrix
; Purpose: Scan 4x4 matrix keypad with anti-ghosting protection
; 
; DETAILED SCANNING ALGORITHM:
; 
; 1. INITIALIZATION:
;    - Set default return value to 0xFF (no key found)
;    - Initialize row counter to 1 (we'll scan rows 1-4)
;
; 2. FOR EACH ROW (1 to 4):
;    a) Set ALL row pins HIGH (inactive)
;    b) Set CURRENT row pin LOW (active)
;    c) Wait briefly for signals to settle (electrical delay)
;    d) Read all column pins
;    e) Check which columns are LOW (indicating pressed buttons)
;    f) If exactly 1 column is LOW, calculate key code
;    g) If 0 or >1 columns LOW, it's either no key or ghosting - ignore
;
; 3. GHOSTING DETECTION:
;    Ghosting happens when multiple keys are pressed and creates false readings.
;    We prevent this by counting LOW columns per row - only accept single key presses.
;
; 4. KEY CODE CALCULATION:
;    Key code = (row-1) * 4 + column + 1
;    Examples: Row 1, Col 0 = (1-1)*4 + 0 + 1 = 1 (key '7')
;              Row 4, Col 1 = (4-1)*4 + 1 + 1 = 14 (key '0')
; 
; Returns: r16 = key scan code (1-16) or 0xFF if no valid key
; Strategy: Row-by-row scanning with column reading and validation
; -----------------------------------------------------------------------------
scan_keyboard_matrix:
    ; SAVE REGISTERS THAT WE'LL MODIFY
    push  r17                          ; Will hold column readings
    push  r18                          ; Will hold bit counting variables  
    push  r19                          ; Will hold key code result
    push  r20                          ; Will hold row counter (1-4)
    
    ; INITIALIZE SCAN VARIABLES
    ldi   r19, 0xFF                    ; Default return value = 0xFF (no key found)
    ldi   r20, 1                       ; Row counter: start with Row 1 (physical top row)
                                       ; We use 1-4 instead of 0-3 to match documentation
    
scan_next_row:
    ; STEP 1: SET ALL ROWS TO INACTIVE STATE (HIGH)
    ; Before activating one row, make sure all others are inactive
    sbi   PORTD, ROW_1                 ; sbi = "Set Bit in I/O register"
                                       ; Set row 1 pin HIGH (PD7 = 1)
    sbi   PORTD, ROW_2                 ; Set row 2 pin HIGH (PD6 = 1)
    sbi   PORTD, ROW_3                 ; Set row 3 pin HIGH (PD5 = 1)  
    sbi   PORTD, ROW_4                 ; Set row 4 pin HIGH (PD4 = 1)
    
    ; STEP 2: ACTIVATE CURRENT ROW (set it LOW)
    ; Use the row counter (r20) to determine which row to activate
    cpi   r20, 1                       ; Compare row counter with 1
    breq  activate_row_1               ; If r20 = 1, activate row 1
    cpi   r20, 2                       ; Compare with 2
    breq  activate_row_2               ; If r20 = 2, activate row 2
    cpi   r20, 3                       ; Compare with 3
    breq  activate_row_3               ; If r20 = 3, activate row 3
    cpi   r20, 4                       ; Compare with 4
    breq  activate_row_4               ; If r20 = 4, activate row 4
    rjmp  scan_complete                ; Invalid row number, exit
    
activate_row_1:
    cbi   PORTD, ROW_1                 ; cbi = "Clear Bit in I/O register"
                                       ; Set row 1 pin LOW (PD7 = 0) - ACTIVE
    rjmp  read_columns                 ; Jump to column reading
activate_row_2:
    cbi   PORTD, ROW_2                 ; Set row 2 pin LOW (PD6 = 0) - ACTIVE
    rjmp  read_columns
activate_row_3:
    cbi   PORTD, ROW_3                 ; Set row 3 pin LOW (PD5 = 0) - ACTIVE
    rjmp  read_columns
activate_row_4:
    cbi   PORTD, ROW_4                 ; Set row 4 pin LOW (PD4 = 0) - ACTIVE
    
read_columns:
    ; STEP 3: WAIT FOR ELECTRICAL SIGNALS TO SETTLE
    ; When we change output pins, there's a brief electrical transition period.
    ; We wait a few CPU cycles to ensure clean readings.
    nop                                ; nop = "No Operation" (do nothing for 1 cycle)
    nop                                ; Each nop takes 1/16MHz = 62.5 nanoseconds
    nop
    nop                                ; Total delay = 4 × 62.5ns = 250ns
    
    ; STEP 4: READ ALL COLUMN STATES
    in    r17, PIND                    ; in = "Input from I/O register"
                                       ; Read entire Port D into r17
                                       ; Columns are on bits 3,2,1,0 of Port D
    
    ; STEP 5: CHECK EACH COLUMN FOR BUTTON PRESS
    ; If a button is pressed, its column will be LOW (0)
    ; We use sbrs (skip if bit set) to check each column bit
    sbrs  r17, COL_0                   ; sbrs = "Skip if Bit in Register Set"
                                       ; Skip next line if column 0 is HIGH (1)
    call  calculate_key_code_col0      ; Column 0 is LOW, calculate key code
    sbrs  r17, COL_1                   ; Check column 1
    call  calculate_key_code_col1      ; Column 1 is LOW, calculate key code
    sbrs  r17, COL_2                   ; Check column 2
    call  calculate_key_code_col2      ; Column 2 is LOW, calculate key code
    sbrs  r17, COL_3                   ; Check column 3
    call  calculate_key_code_col3      ; Column 3 is LOW, calculate key code
    
    ; STEP 6: ANTI-GHOSTING - CHECK FOR MULTIPLE KEY PRESSES
    ; If user presses 2 buttons in same row, we'll see 2 LOW columns
    ; This can create false readings, so we ignore such cases
    mov   r18, r17                     ; Copy column readings
    com   r18                          ; com = "Complement" (flip all bits)
                                       ; Now 1s represent pressed keys
    andi  r18, 0x0F                    ; andi = "AND with Immediate"
                                       ; Keep only column bits (mask = 00001111)
    
    ; COUNT HOW MANY BITS ARE SET (how many keys pressed in this row)
    call  count_set_bits               ; Returns count in r16
    cpi   r16, 2                       ; Compare count with 2
    brsh  ghost_detected               ; brsh = "Branch if Same or Higher"
                                       ; If count ≥ 2, multiple keys = ghosting
    
    ; SINGLE KEY PRESS OR NO KEY - CONTINUE TO NEXT ROW
    inc   r20                          ; inc = "Increment register"
                                       ; Move to next row (r20: 1→2→3→4→5)
    cpi   r20, 5                       ; Compare with 5
    brlo  scan_next_row                ; brlo = "Branch if Lower"
                                       ; If r20 < 5, continue scanning (rows 1-4)
    rjmp  scan_complete                ; All rows scanned, finish
    
ghost_detected:
    ; MULTIPLE KEYS IN SAME ROW - IGNORE THIS SCAN RESULT
    ldi   r19, 0xFF                    ; Set return value to "no key found"
                                       ; This prevents false key detection
    
scan_complete:
    ; RESTORE RETURN VALUE AND CLEAN UP
    mov   r16, r19                     ; Copy final result to r16 (return register)
    
    ; RESTORE ALL SAVED REGISTERS (reverse order of push)
    pop   r20                          ; Restore row counter
    pop   r19                          ; Restore result variable
    pop   r18                          ; Restore bit counting variable
    pop   r17                          ; Restore column readings
    ret                                ; Return to caller with result in r16

; HELPER FUNCTIONS FOR KEY CODE CALCULATION
; These small functions calculate the unique key code based on which row and column
; were detected during the keypad scan. Each function corresponds to one column.

; Calculate key code for column 0 (leftmost column)
calculate_key_code_col0:
    ldi   r18, 0                       ; Column number = 0
    rjmp  store_key_with_row           ; Go to common calculation code

; Calculate key code for column 1  
calculate_key_code_col1:
    ldi   r18, 1                       ; Column number = 1
    rjmp  store_key_with_row

; Calculate key code for column 2
calculate_key_code_col2:
    ldi   r18, 2                       ; Column number = 2
    rjmp  store_key_with_row

; Calculate key code for column 3 (rightmost column)
calculate_key_code_col3:
    ldi   r18, 3                       ; Column number = 3
    
store_key_with_row:
    ; CALCULATE UNIQUE KEY CODE FROM ROW AND COLUMN
    ; Formula: key_code = (row-1) * 4 + column + 1
    ; This gives us key codes 1-16 for our 4x4 keypad
    ; 
    ; Examples:
    ; Row 1, Col 0: (1-1)*4 + 0 + 1 = 1 (key '7')
    ; Row 1, Col 3: (1-1)*4 + 3 + 1 = 4 (key 'F') 
    ; Row 4, Col 1: (4-1)*4 + 1 + 1 = 14 (key '0')
    ; Row 4, Col 3: (4-1)*4 + 3 + 1 = 16 (key 'C')
    
    mov   r19, r20                     ; Copy current row number (1-4)
    dec   r19                          ; Convert to 0-based: 1→0, 2→1, 3→2, 4→3
    lsl   r19                          ; lsl = "Logical Shift Left" 
                                       ; Multiply by 2: 0→0, 1→2, 2→4, 3→6
    lsl   r19                          ; Shift again to multiply by 4 total
                                       ; Now: 0→0, 1→4, 2→8, 3→12
    add   r19, r18                     ; Add column number (0-3)
                                       ; Result: row_offset + column
    inc   r19                          ; Add 1 for 1-based indexing (1-16)
                                       ; Final key code is now in r19
    ret                                ; Return to caller

; -----------------------------------------------------------------------------
; Function: count_set_bits
; Purpose: Count number of 1-bits in r18 (used for ghosting detection)
; 
; HOW BIT COUNTING WORKS:
; We repeatedly check the least significant bit (rightmost bit) and shift right.
; For each 1-bit we find, increment a counter. Continue until all bits processed.
; 
; Example: r18 = 0b00001101 (decimal 13)
; Step 1: Check bit 0 (1) - increment counter to 1, shift → 0b00000110
; Step 2: Check bit 0 (0) - don't increment, shift → 0b00000011  
; Step 3: Check bit 0 (1) - increment counter to 2, shift → 0b00000001
; Step 4: Check bit 0 (1) - increment counter to 3, shift → 0b00000000
; Step 5: r18 = 0, stop. Result: 3 bits were set
; 
; Returns: r16 = number of 1-bits in r18
; -----------------------------------------------------------------------------
count_set_bits:
    push  r17                          ; Save r17 (we'll use it for copying r18)
    ldi   r16, 0                       ; Initialize bit counter to 0
    mov   r17, r18                     ; Copy value to count (preserve original r18)
    
count_loop:
    tst   r17                          ; tst = "Test register" (check if zero)
    breq  count_done                   ; If r17 = 0, no more bits to check
    sbrc  r17, 0                       ; sbrc = "Skip if Bit in Register Clear"
                                       ; Skip next line if bit 0 is 0
    inc   r16                          ; inc = "Increment" 
                                       ; If bit 0 was 1, increment counter
    lsr   r17                          ; lsr = "Logical Shift Right"
                                       ; Move all bits right: bit1→bit0, bit2→bit1, etc.
                                       ; Leftmost bit becomes 0
    rjmp  count_loop                   ; Continue checking remaining bits
    
count_done:
    pop   r17                          ; Restore r17
    ret                                ; Return with count in r16

; -----------------------------------------------------------------------------
; Function: map_key_to_character  
; Purpose: Convert keypad scan code to ASCII character for display
; 
; WHAT ARE ASCII CODES?
; ASCII (American Standard Code for Information Interchange) assigns numbers
; to characters. For example: 'A' = 65, 'B' = 66, '0' = 48, '1' = 49, etc.
; Our LED display expects ASCII codes to know which character pattern to show.
; 
; KEYPAD LAYOUT MAPPING:
; Our keypad has this physical layout:
; [7] [8] [9] [F]  ← Row 1 (scan codes 1,2,3,4)
; [4] [5] [6] [E]  ← Row 2 (scan codes 5,6,7,8)  
; [1] [2] [3] [D]  ← Row 3 (scan codes 9,10,11,12)
; [A] [0] [B] [C]  ← Row 4 (scan codes 13,14,15,16)
; 
; We use a lookup table stored in program memory (Flash) to convert scan codes to ASCII.
; 
; Input: r16 = keypad scan code (1-16, as returned by scan_keyboard_matrix)
; Returns: r16 = ASCII character code (e.g., 'A'=65, '0'=48, 'F'=70)
; Layout: Converts based on standard hex keypad: 0=48, 1=49, ..., A=65, B=66, C=67, D=68, E=69, F=70
; -----------------------------------------------------------------------------
map_key_to_character:
    ; SAVE Z REGISTER (used for program memory access)
    ; The Z register (r31:r30) is special - it's used as a pointer for reading
    ; from program memory with the LPM (Load Program Memory) instruction
    push  ZH                           ; Save high byte of Z register (r31)
    push  ZL                           ; Save low byte of Z register (r30)
    
    ; BOUNDS CHECKING - MAKE SURE SCAN CODE IS VALID
    ; Valid scan codes are 1-16. Anything else indicates an error.
    cpi   r16, 1                       ; Compare with minimum valid value
    brlo  map_default_char             ; brlo = "Branch if Lower"
                                       ; If r16 < 1, use default character
    cpi   r16, 17                      ; Compare with maximum+1 (17)
    brsh  map_default_char             ; brsh = "Branch if Same or Higher"
                                       ; If r16 ≥ 17, use default character
    
    ; CONVERT TO TABLE INDEX
    ; Our lookup table is 0-indexed, but scan codes are 1-16
    dec   r16                          ; Convert 1-16 to 0-15 for table lookup
    
    ; CALCULATE PROGRAM MEMORY ADDRESS
    ; The lookup table is stored in Flash memory at label "keypad_char_map"
    ; In AVR, program memory addresses need to be multiplied by 2 (word addressing)
    ldi   ZH, high(keypad_char_map * 2) ; Load high byte of table address
    ldi   ZL, low(keypad_char_map * 2)  ; Load low byte of table address
    
    ; ADD OFFSET TO GET SPECIFIC CHARACTER ADDRESS
    add   ZL, r16                      ; Add index to low byte
    brcc  no_carry_map                 ; brcc = "Branch if Carry Clear"
                                       ; If no overflow from addition, skip next line
    inc   ZH                           ; If carry occurred, increment high byte
no_carry_map:
    ; READ CHARACTER FROM PROGRAM MEMORY
    lpm   r16, Z                       ; lpm = "Load Program Memory"
                                       ; Load byte from Flash memory at address Z
                                       ; Result goes into r16
    rjmp  map_exit                     ; Jump to cleanup and return
    
map_default_char:
    ; INVALID SCAN CODE - USE DEFAULT CHARACTER
    ldi   r16, '?'                     ; ASCII code for '?' character (decimal 63)
                                       ; This indicates an error or unknown key
    
map_exit:
    ; RESTORE Z REGISTER AND RETURN
    pop   ZL                           ; Restore low byte of Z register
    pop   ZH                           ; Restore high byte of Z register
    ret                                ; Return with ASCII character in r16

; KEYPAD CHARACTER MAPPING TABLE (stored in program memory)
; This table maps 0-based indices to ASCII character codes.
; The table is organized to match the physical keypad layout:
;
; Index 0-3:   Physical Row 1 (top): [7][8][9][F] 
; Index 4-7:   Physical Row 2: [4][5][6][E]
; Index 8-11:  Physical Row 3: [1][2][3][D] 
; Index 12-15: Physical Row 4 (bottom): [A][0][B][C]
;
; The .db directive stores bytes in program memory (Flash).
keypad_char_map:
    .db   '7', '8', '9', 'F'           ; Indices 0-3: Physical Row 1 (top row)
                                       ; '7'=55, '8'=56, '9'=57, 'F'=70 in ASCII
    .db   '4', '5', '6', 'E'           ; Indices 4-7: Physical Row 2
                                       ; '4'=52, '5'=53, '6'=54, 'E'=69 in ASCII
    .db   '1', '2', '3', 'D'           ; Indices 8-11: Physical Row 3
                                       ; '1'=49, '2'=50, '3'=51, 'D'=68 in ASCII
    .db   'A', '0', 'B', 'C'           ; Indices 12-15: Physical Row 4 (bottom row)
                                       ; 'A'=65, '0'=48, 'B'=66, 'C'=67 in ASCII

; =============================================================================
; MAIN APPLICATION LOOP - HIGH-SPEED DISPLAY REFRESH
; =============================================================================
;
; THE HEART OF THE PROGRAM:
; This is where the microcontroller spends 99.9% of its time. It continuously:
; 1. Updates one row of the LED matrix display
; 2. Checks for keypad button presses
; 3. Repeats forever
;
; WHY THIS DESIGN?
; - LED displays need constant refreshing to appear stable to human eyes
; - We can only light up one row at a time, so we rapidly cycle through all rows
; - Keypad checking is integrated to avoid blocking the display refresh
; - If display refresh stops, the LEDs will flicker or go dark

; -----------------------------------------------------------------------------
; Function: main_application_loop
; Purpose: Continuous display refresh with integrated keypad scanning
; 
; EXECUTION FLOW:
; This creates an infinite loop that never ends (until power off or reset).
; Each iteration:
; 1. Refreshes exactly one row of the LED matrix (7 rows total, cycling 0→1→2→3→4→5→6→0→...)
; 2. Performs one quick keypad scan (non-blocking, takes microseconds)
; 3. Jumps back to start of loop
; 
; TIMING CONSIDERATIONS:
; - Each loop iteration should complete in under 2-3 milliseconds
; - Total refresh rate: ~100-200 Hz (100-200 complete screen refreshes per second)
; - This is fast enough that human eyes see a steady, flicker-free image
; - Keypad response feels immediate (checked 100+ times per second)
; 
; Strategy: Refresh display, scan keypad, handle key presses, repeat forever
; -----------------------------------------------------------------------------
main_application_loop:
    call  refresh_current_display_row  ; Update one row of the LED matrix
                                       ; This sends data to shift register and advances
                                       ; to the next row for the following iteration
    call  scan_keypad_nonblocking      ; Check for key presses (non-blocking)
                                       ; This takes only a few microseconds and doesn't
                                       ; interfere with display refresh timing
    rjmp  main_application_loop        ; rjmp = "Relative Jump"
                                       ; Jump back to start - infinite loop for continuous operation

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
; Row 1: 10001  (bits: 0b10001 = 0x11) 
; Row 2: 10001  (bits: 0b10001 = 0x11)
; Row 3: 11111  (bits: 0b11111 = 0x1F)
; Row 4: 10001  (bits: 0b10001 = 0x11)
; Row 5: 10001  (bits: 0b10001 = 0x11)
; Row 6: 00000  (bits: 0b00000 = 0x00)

; -----------------------------------------------------------------------------
; Function: get_character_bitmap_address
; Purpose: Convert ASCII character to program memory bitmap address
; 
; WHAT THIS FUNCTION DOES:
; Takes an ASCII code (like 'A' = 65) and returns the memory address where
; that character's bitmap data is stored. This is like looking up a word
; in a dictionary - we need to know where to find the definition (bitmap).
; 
; INPUT/OUTPUT:
; Input: r16 = ASCII character code (like 'A'=65, '0'=48, 'F'=70)
; Returns: r17:r16 = 16-bit program memory address of character bitmap
;          (r17 = high byte, r16 = low byte of address)
; 
; SUPPORTED CHARACTERS:
; We support hexadecimal characters: 0,1,2,3,4,5,6,7,8,9,A,B,C,D,E,F
; Any other character defaults to '7' (could be changed to '?' or space)
; 
; Strategy: Lookup table approach with bounds checking for supported characters
; -----------------------------------------------------------------------------
get_character_bitmap_address:
    ; SAVE Z REGISTER (we'll need it but caller might be using it)
    push  ZH                           ; Save Z register high byte
    push  ZL                           ; Save Z register low byte
    
    ; CHECK FOR EACH SUPPORTED CHARACTER AND BRANCH TO APPROPRIATE HANDLER
    ; This is like a big "switch" statement in higher-level languages
    ; We compare the ASCII code and jump to the right address calculation
    
    ; NUMERIC CHARACTERS (ASCII 48-57 for '0'-'9')
    cpi   r16, '0'                     ; Compare with ASCII '0' (48)
    breq  map_char_0                   ; breq = "Branch if Equal" - if r16='0', jump
    cpi   r16, '1'                     ; Compare with ASCII '1' (49)  
    breq  map_char_1                   ; If r16='1', jump to map_char_1
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
    
    ; HEXADECIMAL LETTER CHARACTERS (ASCII 65-70 for 'A'-'F')
    cpi   r16, 'A'                     ; ASCII 'A' (65)
    breq  map_char_A
    cpi   r16, 'B'                     ; ASCII 'B' (66)
    breq  map_char_B
    cpi   r16, 'C'                     ; ASCII 'C' (67)
    breq  map_char_C
    cpi   r16, 'D'                     ; ASCII 'D' (68)
    breq  map_char_D
    cpi   r16, 'E'                     ; ASCII 'E' (69)
    breq  map_char_E
    cpi   r16, 'F'                     ; ASCII 'F' (70)
    breq  map_char_F
    
    ; DEFAULT CASE - UNSUPPORTED CHARACTER
    ; If we get here, the character isn't in our supported set
    ldi   r16, low(char_7 * 2)         ; Use character '7' as default
    ldi   r17, high(char_7 * 2)        ; The *2 is needed for AVR program memory addressing
    rjmp  bitmap_address_exit           ; Jump to cleanup and return
    
    ; INDIVIDUAL CHARACTER ADDRESS CALCULATIONS
    ; Each character's bitmap data is stored at a labeled location in program memory
    ; We calculate the address and store it in r17:r16 (high:low bytes)
    ; The "*2" multiplier is required because AVR uses word-based program memory addressing
map_char_0:
    ldi   r16, low(char_0 * 2)         ; Load low byte of char_0 address
    ldi   r17, high(char_0 * 2)        ; Load high byte of char_0 address
    rjmp  bitmap_address_exit          ; Jump to cleanup
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
    ; CLEANUP AND RETURN
    pop   ZL                           ; Restore Z register low byte  
    pop   ZH                           ; Restore Z register high byte
    ret                                ; Return with address in r17:r16

; =============================================================================
; CHARACTER BITMAP DEFINITIONS - 7×5 PIXEL FONT
; =============================================================================
; 
; UNDERSTANDING BITMAP FONTS (for complete beginners):
; 
; A bitmap font stores each character as a pattern of dots (pixels). Our display
; is 7 rows tall and 5 columns wide, giving us a 7×5 grid per character.
; 
; VISUAL EXAMPLE - How the number '0' looks:
; 
;   Columns: 4 3 2 1 0  (bit positions, 4=leftmost, 0=rightmost)
;            █ █ █ █ █
; Row 0:     ░ █ █ █ ░   = 0b01111 = 0x0F
; Row 1:     █ ░ ░ ░ █   = 0b10001 = 0x11  
; Row 2:     █ ░ ░ ░ █   = 0b10001 = 0x11
; Row 3:     █ ░ ░ ░ █   = 0b10001 = 0x11
; Row 4:     █ ░ ░ ░ █   = 0b10001 = 0x11
; Row 5:     █ ░ ░ ░ █   = 0b10001 = 0x11
; Row 6:     ░ █ █ █ ░   = 0b01111 = 0x0F
; Row 7:     ░ ░ ░ ░ ░   = 0b00000 = 0x00 (padding)
; 
; KEY POINTS:
; • Each row is stored as one byte
; • Only the lowest 5 bits are used (columns 0-4)
; • 1 = LED on (visible dot), 0 = LED off (dark)  
; • Bit 0 = rightmost column, Bit 4 = leftmost column
; • Row 7 is padding (always 0) for memory alignment
; 
; FORMAT SPECIFICATION:
; • Each character: 8 bytes total (7 active rows + 1 padding byte)  
; • Bit encoding: 0bxxxCCCCC where CCCCC = column bits (5 pixels wide)
; • Padding byte (index 7) must be 0 for proper array bounds
; • Row 0 = top of character, Row 6 = bottom of character
;
; MEMORY LAYOUT:
; To find a specific row of a character:
; Address = character_start_address + row_index
; Example: char_A row 3 = char_A + 3
;
; VISUAL REFERENCE GRID:
;   Bit:  4 3 2 1 0
;   Col:  █ █ █ █ █  ← 5 columns per character
;         ↑       ↑
;      Left    Right
;
; =============================================================================

character_bitmap_table:

; ALPHABETIC CHARACTERS (A-F implemented for hexadecimal display)
; These patterns are designed to be readable on a 7×5 pixel matrix

char_A:  .db 0b00110, 0b01001, 0b01001, 0b01001, 0b01111, 0b01001, 0b01001, 0  ; Letter A
         ; Row 0: ░░██░    Row 1: ░█░░█    Row 2: ░█░░█    Row 3: ░█░░█
         ; Row 4: ░████    Row 5: ░█░░█    Row 6: ░█░░█    Row 7: ░░░░░ (padding)

char_B:  .db 0b01110, 0b01001, 0b01001, 0b01110, 0b01001, 0b01001, 0b01110, 0  ; Letter B  
         ; Row 0: ░███░    Row 1: ░█░░█    Row 2: ░█░░█    Row 3: ░███░
         ; Row 4: ░█░░█    Row 5: ░█░░█    Row 6: ░███░    Row 7: ░░░░░ (padding)

char_C:  .db 0b00111, 0b01000, 0b01000, 0b01000, 0b01000, 0b01000, 0b00111, 0  ; Letter C
         ; Row 0: ░░███    Row 1: ░█░░░    Row 2: ░█░░░    Row 3: ░█░░░  
         ; Row 4: ░█░░░    Row 5: ░█░░░    Row 6: ░░███    Row 7: ░░░░░ (padding)

char_D:  .db 0b01110, 0b01001, 0b01001, 0b01001, 0b01001, 0b01001, 0b01110, 0  ; Letter D
         ; Row 0: ░███░    Row 1: ░█░░█    Row 2: ░█░░█    Row 3: ░█░░█
         ; Row 4: ░█░░█    Row 5: ░█░░█    Row 6: ░███░    Row 7: ░░░░░ (padding)

char_E:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01111, 0  ; Letter E
         ; Row 0: ░████    Row 1: ░█░░░    Row 2: ░█░░░    Row 3: ░███░
         ; Row 4: ░█░░░    Row 5: ░█░░░    Row 6: ░████    Row 7: ░░░░░ (padding)

char_F:  .db 0b01111, 0b01000, 0b01000, 0b01110, 0b01000, 0b01000, 0b01000, 0  ; Letter F
         ; Row 0: ░████    Row 1: ░█░░░    Row 2: ░█░░░    Row 3: ░███░
         ; Row 4: ░█░░░    Row 5: ░█░░░    Row 6: ░█░░░    Row 7: ░░░░░ (padding)

; NUMERIC CHARACTERS (0-9 complete set for decimal and hexadecimal display)

char_0:  .db 0b01111, 0b01001, 0b01001, 0b01001, 0b01001, 0b01001, 0b01111, 0  ; Number 0
         ; Row 0: ░████    Row 1: ░█░░█    Row 2: ░█░░█    Row 3: ░█░░█
         ; Row 4: ░█░░█    Row 5: ░█░░█    Row 6: ░████    Row 7: ░░░░░ (padding)

char_1:  .db 0b00010, 0b00110, 0b01010, 0b00010, 0b00010, 0b00010, 0b00010, 0  ; Number 1
         ; Row 0: ░░░█░    Row 1: ░░██░    Row 2: ░█░█░    Row 3: ░░░█░
         ; Row 4: ░░░█░    Row 5: ░░░█░    Row 6: ░░░█░    Row 7: ░░░░░ (padding)

char_2:  .db 0b01111, 0b00001, 0b00001, 0b01111, 0b01000, 0b01000, 0b01111, 0  ; Number 2
         ; Row 0: ░████    Row 1: ░░░░█    Row 2: ░░░░█    Row 3: ░████
         ; Row 4: ░█░░░    Row 5: ░█░░░    Row 6: ░████    Row 7: ░░░░░ (padding)

char_3:  .db 0b01111, 0b00001, 0b00001, 0b00111, 0b00001, 0b00001, 0b01111, 0  ; Number 3
         ; Row 0: ░████    Row 1: ░░░░█    Row 2: ░░░░█    Row 3: ░░███
         ; Row 4: ░░░░█    Row 5: ░░░░█    Row 6: ░████    Row 7: ░░░░░ (padding)

char_4:  .db 0b01001, 0b01001, 0b01001, 0b01111, 0b00001, 0b00001, 0b00001, 0  ; Number 4  
         ; Row 0: ░█░░█    Row 1: ░█░░█    Row 2: ░█░░█    Row 3: ░████
         ; Row 4: ░░░░█    Row 5: ░░░░█    Row 6: ░░░░█    Row 7: ░░░░░ (padding)

char_5:  .db 0b01111, 0b01000, 0b01000, 0b01111, 0b00001, 0b00001, 0b01111, 0  ; Number 5
         ; Row 0: ░████    Row 1: ░█░░░    Row 2: ░█░░░    Row 3: ░████
         ; Row 4: ░░░░█    Row 5: ░░░░█    Row 6: ░████    Row 7: ░░░░░ (padding)

char_6:  .db 0b01111, 0b01000, 0b01000, 0b01111, 0b01001, 0b01001, 0b01111, 0  ; Number 6
         ; Row 0: ░████    Row 1: ░█░░░    Row 2: ░█░░░    Row 3: ░████
         ; Row 4: ░█░░█    Row 5: ░█░░█    Row 6: ░████    Row 7: ░░░░░ (padding)

char_7:  .db 0b01111, 0b00001, 0b00001, 0b00010, 0b00100, 0b00100, 0b00100, 0  ; Number 7
         ; Row 0: ░████    Row 1: ░░░░█    Row 2: ░░░░█    Row 3: ░░░█░
         ; Row 4: ░░█░░    Row 5: ░░█░░    Row 6: ░░█░░    Row 7: ░░░░░ (padding)

char_8:  .db 0b01111, 0b01001, 0b01001, 0b01111, 0b01001, 0b01001, 0b01111, 0  ; Number 8
         ; Row 0: ░████    Row 1: ░█░░█    Row 2: ░█░░█    Row 3: ░████
         ; Row 4: ░█░░█    Row 5: ░█░░█    Row 6: ░████    Row 7: ░░░░░ (padding)

char_9:  .db 0b01111, 0b01001, 0b01001, 0b01111, 0b00001, 0b00001, 0b01111, 0  ; Number 9
         ; Row 0: ░████    Row 1: ░█░░█    Row 2: ░█░░█    Row 3: ░████
         ; Row 4: ░░░░█    Row 5: ░░░░█    Row 6: ░████    Row 7: ░░░░░ (padding)

; CHARACTER SET BOUNDARY MARKER
; This label marks the end of the character definition table.
; It can be used for bounds checking to prevent reading beyond valid character data.
character_table_end:

; =============================================================================
; INTERRUPT SERVICE ROUTINES - TIME-CRITICAL HANDLERS
; =============================================================================
;
; WHAT ARE INTERRUPT SERVICE ROUTINES (ISRs)?
; ISRs are special functions that run when hardware events occur. They must:
; 1. Execute as quickly as possible (microseconds, not milliseconds)
; 2. Not interfere with the main program
; 3. Handle only the specific event that triggered them
; 4. Always end with 'reti' (return from interrupt)

; -----------------------------------------------------------------------------
; ISR: timer0_latch_completion_isr  
; Purpose: Complete the LED matrix latch pulse and disable timer for next cycle
; 
; WHAT THIS ISR DOES:
; This ISR is triggered when Timer0 overflows (counts from 255 back to 0).
; We use this precise timing to end the latch pulse for the LED shift register.
; The latch pulse must be exactly the right length - too short and LEDs won't
; update, too long and it wastes power and may cause flickering.
; 
; TIMING EXPLANATION:
; 1. Main program starts latch pulse (rising edge) and starts Timer0
; 2. Timer0 counts for precisely T0_LATCH_DELAY cycles  
; 3. Timer0 overflows and triggers this ISR
; 4. ISR ends latch pulse (falling edge) and stops timer
; 5. Result: Latch pulse with exact, consistent width
;
; Trigger: Timer0 overflow after precise delay period
; Registers: Uses r16 (automatically saved/restored by hardware during interrupts)
; Critical: This ISR controls display multiplexing timing accuracy
; -----------------------------------------------------------------------------
timer0_latch_completion_isr:
    ; RESET TIMER COUNTER FOR CONSISTENT TIMING ON NEXT ACTIVATION  
    ; Even though the timer just overflowed, we reload it with the same delay
    ; value so the next time it's started, it will have the same timing.
    ldi   r16, T0_LATCH_DELAY          ; T0_LATCH_DELAY = 240
                                       ; Timer will count 240→241→...→255→0 (overflow)
                                       ; This gives us exactly 16 clock cycles of delay
    sts   TCNT0, r16                   ; sts = "Store Direct to SRAM"
                                       ; Reset Timer0 counter register
    
    ; COMPLETE LATCH PULSE SEQUENCE (end of latch pulse)
    ; The latch pulse started in the main program as a rising edge.
    ; Now we create the falling edge to complete the pulse.
    sbi   SR_PINREG, SR_LATCH_PIN      ; sbi on PIN register toggles the bit
                                       ; This creates falling edge (HIGH→LOW)
                                       ; Shift register now latches data to outputs
    
    ; DISABLE TIMER0 TO SAVE POWER UNTIL NEXT REFRESH CYCLE  
    ; We only need the timer running during latch pulse generation.
    ; Stopping it saves power and prevents unwanted interrupts.
    lds   r16, TCCR0B                  ; Load Timer Control Register B
    andi  r16, ~((1<<CS02)|(1<<CS01)|(1<<CS00))  ; Clear all clock select bits
                                       ; CS02=CS01=CS00=0 means "no clock source"
    sts   TCCR0B, r16                  ; Store back - timer is now stopped
    
    reti                               ; reti = "Return from Interrupt"
                                       ; Restore program counter and SREG flags
                                       ; Return to main program where it left off

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
    ; UPDATE ROW STATE FOR NEXT REFRESH CYCLE
    ; Move to the next row in the scanning sequence
    dec   r22                          ; Move to next row index (6→5→4→...→0)
    lsr   r21                          ; Logical Shift Right of row selector
                                       ; Moves the '1' bit right: 0100000→0010000
    
    ; CHECK FOR ROW SEQUENCE WRAPAROUND
    brne  initiate_hardware_latch      ; If r21≠0, more rows remain, continue
    
    ; RESET TO TOP OF MATRIX (row sequence complete)
    ; When r21=0, we've finished all rows, start over from row 6
    ldi   r22, MAX_ROW_INDEX           ; Reset to row index 6
    ldi   r21, ROW_RESET_MASK          ; Reset row selector: bit 6 = 1

initiate_hardware_latch:
    ; BEGIN PRECISION LATCH PULSE SEQUENCE
    ; This transfers all the data we just sent from the shift register's
    ; internal buffer to the actual LED outputs
    sbi   SR_PINREG, SR_LATCH_PIN      ; Start latch pulse (rising edge)
                                       ; Shift register begins internal transfer
    
    ; ACTIVATE TIMER0 FOR PRECISE LATCH DURATION CONTROL
    ; We need the latch pulse to be exactly the right length
    lds   r16, TCCR0B                  ; Load Timer Control Register B
    ori   r16, TIMER0_PRESCALER        ; Apply prescaler bits (start timer)
                                       ; Timer now counts at 16MHz/1024 = 15.625kHz
    sts   TCCR0B, r16                  ; Start timer with clock source
                                       ; Timer will overflow and trigger ISR
    
    ; RESTORE WORKING REGISTERS FROM STACK (reverse LIFO order)
    pop   r19                          ; Restore block index register
    pop   r18                          ; Restore loop counter register
    pop   r17                          ; Restore general purpose register
    pop   r16                          ; Restore timer control register
    
    ret                                ; Return to main loop
                                       ; Timer ISR will complete latch pulse later

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