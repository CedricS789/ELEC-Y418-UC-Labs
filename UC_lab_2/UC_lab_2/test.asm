.def val1 = r16
.def val2 = r17

main:
    ; Load some initial values to experiment with
    ldi val1, 10      ; Load decimal 10 into r16
    ldi val2, 0x20    ; Load hexadecimal 20 into r17

loop:
    rjmp loop         ; Loop forever so the program doesn't end