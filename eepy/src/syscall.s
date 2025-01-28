.global SVCall

    .thumb_func
SVCall:
    ldr     r2, =handle_syscall
    movs    r0, #4
    mov     r1, lr
    tst     r0, r1
    beq     100f    // use_msp
    mrs     r0, psp
    movs    r1, #1
    bx      r2
100: // use_msp
    mrs     r0, msp
    movs    r1, #0
    bx      r2