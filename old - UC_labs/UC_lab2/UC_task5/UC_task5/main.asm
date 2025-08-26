;
; UC_task5.asm
;
; Created: 20/07/2025 11:18:56
; Author : Cedric
;

.include "m328pdef.inc"


; ---- Constants Definition ----
.def temp = r16
.equ F_CPU = 16000000
.equ BUZZER_FREQ = 440
.equ INTERRUPT_FREQ = BUZZER_FREQ * 2 ; We need to toggle the buzzer at twice the frequency
.equ OCR2A_VALUE = 141


.org 0x0000
    rjmp RESET_HANDLER      ; When the chip resets, jump to our setup code
.org OC2Aaddr
    rjmp TIMER2_COMPA_ISR   ; When Timer2 Compare Match A occurs, jump to our ISR

RESET_HANDLER:
    ; ---- Set up the stack pointer ----
    ldi temp, high(RAMEND)
    out SPH, temp
    ldi temp, low(RAMEND)
    out SPL, temp

    ; ---- I/O Pin Setup ----
    sbi DDRB, DDB1          ; Set PB1 (Buzzer) as output
    cbi DDRB, DDB2          ; Set PB2 (LED) as input
    sbi PORTB, PB2          ; Enable pull-up resistor on PB2

    ; ---- Timer2 Setup ----
    ldi temp, (1 << WGM21)  ; Set CTC mode
    out TCCR2A, temp 
    ldi temp, OCR2A_VALUE   ; Set compare value for 440Hz
    out OCR2A, temp         ; Set compare value for 440Hz
