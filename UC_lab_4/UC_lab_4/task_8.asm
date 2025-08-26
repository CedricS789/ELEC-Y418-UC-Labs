;================================================================
; Description:  This program demonstrates the most basic control
;               of the 80x7 LED matrix. It serially shifts 88
;               bits of '1' into the screen's driver chain
;               of shift registers. This action should enable all
;               rows and all columns, lighting up the entire screen.
;               This confirms the physical connections and the
;               basic serial protocol are working correctly.
;================================================================

.include "m328pdef.inc"

; --- Register Definitions ---
.def temp          = r16  ; A temporary register for various operations
.def bit_counter   = r17  ; A counter for the 88 bits to be shifted

;================================================================
; INTERRUPT AND RESET VECTORS
;================================================================
.ORG 0x0000             ; Reset Vector: Where the MCU starts execution
RJMP init               ; On reset, jump to the initialization routine

;================================================================
; INITIALIZATION ROUTINE
;================================================================
init:
    ; Configure screen control pins (PB3, PB4, PB5) as outputs.
    ; This is done by setting the corresponding bits in the Data Direction Register for Port B.
    SBI DDRB, 3         ; Set DDB3 to 1 -> PB3 (SDI - Serial Data In) is an output
    SBI DDRB, 4         ; Set DDB4 to 1 -> PB4 (LE - Latch Enable) is an output
    SBI DDRB, 5         ; Set DDB5 to 1 -> PB5 (CLK - Clock) is an output

    RJMP main           ; After initialization, jump to the main program loop

;================================================================
; MAIN PROGRAM LOOP
;================================================================
main:
    ; This section shifts 88 bits of data to the screen's shift registers.
    LDI bit_counter, 88 ; Initialize our loop counter to 88

shift_loop:
    SET                 ; Set the T-flag in the SREG to 1. We will send this '1' bit.
    RCALL send_bit      ; Call the subroutine that sends the bit in the T-flag
    DEC bit_counter     ; Decrement the loop counter
    BRNE shift_loop     ; If the counter is not zero, loop back and send the next bit

    ; After all 88 bits have been shifted into the registers, we need to make them
    ; appear on the outputs. This is a two-step process for our hardware:
    ; 1. Latch the data: A rising edge on LE copies the shifted data to the output latches.
    ; 2. Enable the outputs: A low level on LE enables the physical outputs.
    SBI PORTB, 4        ; Drive PB4 (LE) HIGH -> Rising edge latches the data
    CBI PORTB, 4        ; Drive PB4 (LE) LOW  -> Enables the shift register outputs

loop_forever:
    RJMP loop_forever   ; The screen is now set. We loop here indefinitely to keep it on.

;================================================================
; SUBROUTINES
;================================================================

; --- Subroutine: send_bit ---
; Description:   Sends the single bit currently stored in the SREG T-flag
;                out on the SDI pin (PB3), followed by a clock pulse on CLK (PB5).
; Registers used: None (Flags are affected by BRTC)
send_bit:
    ; Set the data line (PB3) according to the T-flag's value
    CBI PORTB, 3        ; Assume the bit is 0, so set PB3 LOW
    BRTC bit_is_0       ; Check the T-flag. If it's Clear (0), branch to bit_is_0
    SBI PORTB, 3        ; If the branch was not taken, the T-flag was Set (1), so set PB3 HIGH

bit_is_0:
    ; Pulse the clock line (PB5) to shift the bit into the register chain
    SBI PORTB, 5        ; Drive PB5 HIGH (Rising edge)
    CBI PORTB, 5        ; Drive PB5 LOW (Falling edge)
    RET                 ; Return from subroutine
