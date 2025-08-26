; ==============================================================================
; DESCRIPTION:
; This program displays a checkerboard pattern on a custom 80x7 LED matrix.
; The matrix is driven by a chain of shift registers. The display is refreshed
; row-by-row using an interrupt service routine (ISR) triggered by the
; Timer2 overflow. This version is written without macros for clarity and
; compatibility with AVRASM2/Microchip Studio.
;
; HARDWARE CONNECTIONS (PORTB):
;   - PB3 (SDI):  Serial Data In for the shift register chain.
;   - PB4 (LEOE): Latch Enable (rising edge) and Output Enable (active-low).
;                 This pin controls when the shifted data is displayed.
;   - PB5 (CLK):  Shift Clock for the shift registers.
;
; ==============================================================================

.include "m328pdef.inc"

; ------------------------------------------------------------------------------
; -- Constant and Pin Definitions
; ------------------------------------------------------------------------------
; Using .equ makes the code more readable and easier to modify if pins change.
.equ SDI  =  3       ; PORTB bit for Serial Data In
.equ LEOE =  4       ; PORTB bit for Latch Enable / Output Enable
.equ CLK  =  5       ; PORTB bit for Shift Clock

; ------------------------------------------------------------------------------
; -- SRAM Data Segment
; ------------------------------------------------------------------------------
; .dseg places variables in the SRAM data memory space.
.dseg
row_idx: .byte 1    ; Stores the current row being refreshed (0-7).
                    ; We use 8 rows logically for easier pattern generation,
                    ; even if only 7 are physically connected.

.cseg
; ------------------------------------------------------------------------------
; -- Interrupt Vector Table
; ------------------------------------------------------------------------------
; The vector table contains jump instructions to the appropriate handlers.
.org 0x0000
    rjmp    reset_handler   ; Reset Vector: Code execution starts here on power-up or reset.

.org 0x0012
    rjmp    isr_timer2_ovf  ; Timer2 Overflow Vector: Jumps here when Timer2 overflows.

; ==============================================================================
; -- SUBROUTINE: shift_byte_lsb
; -- DESCRIPTION: Shifts out a single byte from a register (r24) via the SDI pin,
; --              LSB first. It generates the necessary clock pulses on the CLK pin.
; -- INPUTS:      r24 - The byte to be shifted out.
; -- CLOBBERS:    r24, r25
; ==============================================================================
shift_byte_lsb:
    ldi     r25, 8          ; Initialize a loop counter for 8 bits.
shift_byte_loop:
    lsr     r24             ; Logical Shift Right: Moves the LSB into the Carry flag (C).
    brcc    shift_bit_zero  ; Branch if Carry Clear (i.e., the bit was 0).
    sbi     PORTB, SDI      ; If Carry was set (bit was 1), set the SDI pin high.
    rjmp    pulse_clock     ; Jump to the clock pulse logic.
shift_bit_zero:
    cbi     PORTB, SDI      ; If Carry was clear (bit was 0), set the SDI pin low.
pulse_clock:
    sbi     PORTB, CLK      ; Generate a rising edge on the CLK pin.
    cbi     PORTB, CLK      ; Generate a falling edge on the CLK pin, completing the pulse.
    dec     r25             ; Decrement the bit counter.
    brne    shift_byte_loop ; If not all 8 bits are sent, loop again.
    ret                     ; Return from subroutine.

; ==============================================================================
; -- INITIALIZATION ROUTINE
; ==============================================================================
reset_handler:
    ; --- Stack Pointer Initialization (Good Practice) ---
    ; Although not strictly necessary for this specific code as there are no
    ; subroutines called from the main loop, it's crucial for any real application.
    ldi     r16, high(RAMEND)
    out     SPH, r16
    ldi     r16, low(RAMEND)
    out     SPL, r16

    ; --- I/O Port Configuration ---
    ; Configure the control pins (PB3, PB4, PB5) as outputs.
    ldi     r16, (1<<SDI) | (1<<LEOE) | (1<<CLK)
    out     DDRB, r16
    ; Start with all output pins low. LEOE is active-low for output enable,
    ; so starting it low would enable outputs, but we immediately disable them
    ; in the ISR before updating, so this initial state is safe.
    clr     r17
    out     PORTB, r17

    ; --- Timer2 Configuration ---
    ; We need a periodic interrupt to refresh the display row-by-row.
    ; At 16MHz CPU clock, a prescaler of 64 gives a timer clock of 250kHz.
    ; Timer2 (8-bit) overflows every 256 ticks.
    ; Overflow frequency = 250,000 Hz / 256 = ~976.5 Hz.
    ; Refresh rate for all 8 rows = 976.5 / 8 = ~122 Hz, which is flicker-free.
    ldi     r16, 0x00
    sts     TCCR2A, r16     ; Set Timer2 to Normal mode (WGM2[1:0]=00).
    ldi     r16, (1<<CS22)  ; Set prescaler to /64 (CS2[2:0]=100).
    sts     TCCR2B, r16

    ; --- Interrupt Configuration ---
    ; Enable the Timer2 Overflow Interrupt (TOIE2).
    lds     r16, TIMSK2
    ori     r16, (1<<TOIE2)
    sts     TIMSK2, r16

    ; --- State Initialization ---
    ; Initialize the row index to the first row (row 0).
    clr     r16
    sts     row_idx, r16

    sei                     ; Enable global interrupts.

; ==============================================================================
; -- MAIN PROGRAM LOOP
; ==============================================================================
main_loop:
    ; This is an interrupt-driven program. The main loop has nothing to do
    ; except wait for interrupts to occur. All the display logic is in the ISR.
    rjmp    main_loop

; ==============================================================================
; -- INTERRUPT SERVICE ROUTINE: Timer2 Overflow
; -- DESCRIPTION: This routine is executed each time Timer2 overflows (~976 Hz).
; --              It handles the drawing of a single row of the LED matrix.
; -- CLOBBERS:    r0, r16-r21, r24-r25 (registers are saved/restored)
; ==============================================================================
isr_timer2_ovf:
    ; --- ISR Prologue: Save Context ---
    ; Push registers that will be used in the ISR onto the stack to preserve their
    ; state from the main program. The SREG must be saved first.
    push    r0
    in      r0, SREG
    push    r0
    push    r16
    push    r17
    push    r18
    push    r19
    push    r20
    push    r21
    push    r24
    push    r25

    ; --- Refresh Step 1: Disable LED Outputs ---
    ; Set LEOE pin high. Since OE is active-low, this disables the LED outputs
    ; to prevent "ghosting" or flickering while the shift registers are updated.
    sbi     PORTB, LEOE

    ; --- Refresh Step 2: Determine Column Pattern for the Current Row ---
    lds     r16, row_idx        ; r16 = current row index (0-7).
    mov     r17, r16
    andi    r17, 1              ; Check if the LSB is 1 (i.e., if the row is odd).
    brne    pattern_for_odd_row ; Branch if not equal to zero (row is odd).
pattern_for_even_row:
    ldi     r18, 0xAA           ; Pattern for even rows: 10101010
    rjmp    pattern_selected
pattern_for_odd_row:
    ldi     r18, 0x55           ; Pattern for odd rows:  01010101
pattern_selected:

    ; --- Refresh Step 3: Shift Out 80 Column Bits ---
    ; The 80x7 matrix requires 80 bits of column data. This is 10 bytes.
    ; We shift out the same pattern byte (r18) ten times.
    ldi     r19, 10             ; Loop counter for 10 bytes.
shift_10_bytes_loop:
    mov     r24, r18            ; Load the pattern into the register used by the shift routine.
    rcall   shift_byte_lsb
    dec     r19
    brne    shift_10_bytes_loop

    ; --- Refresh Step 4: Create and Shift the One-Hot Row Selection Byte ---
    ; To activate a single row, we need a byte where only one bit is set.
    ; e.g., row 0 = 0b00000001, row 1 = 0b00000010, row 7 = 0b10000000.
    ldi     r24, 1              ; Start with bit 0 set.
    mov     r20, r16            ; Use r20 as a temporary counter, holds row_idx.
row_select_shift_loop:
    tst     r20                 ; Test if r20 is zero.
    breq    row_select_ready    ; If zero, the '1' is in the correct position.
    lsl     r24                 ; Logical Shift Left: Move the '1' one position to the left.
    dec     r20                 ; Decrement the counter.
    rjmp    row_select_shift_loop
row_select_ready:
    ; Now r24 contains the one-hot byte. Shift it out.
    rcall   shift_byte_lsb

    ; --- Refresh Step 5: Latch Data and Re-enable Outputs ---
    ; A rising edge on LEOE latches the 88 bits from the shift registers
    ; into their internal storage registers.
    sbi     PORTB, LEOE
    ; A falling edge on LEOE enables the outputs (since OE is active-low),
    ; displaying the new row data.
    cbi     PORTB, LEOE

    ; --- Refresh Step 6: Advance to the Next Row ---
    lds     r16, row_idx        ; Load the current row index.
    inc     r16                 ; Increment to the next row.
    cpi     r16, 8              ; Compare with 8. Have we gone past the last row (7)?
    brlo    store_new_row_idx   ; If less than 8, the index is valid.
    clr     r16                 ; If it was 8, wrap around to 0.
store_new_row_idx:
    sts     row_idx, r16        ; Save the new row index back to SRAM.

    ; --- ISR Epilogue: Restore Context ---
    ; Pop all saved registers from the stack in the reverse order they were pushed.
    pop     r25
    pop     r24
    pop     r21
    pop     r20
    pop     r19
    pop     r18
    pop     r17
    pop     r16
    pop     r0
    out     SREG, r0            ; Restore the Status Register.
    pop     r0
    reti                        ; Return from interrupt.
