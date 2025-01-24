.global SVCall
.equ    SVCall, _SVCall + 1 /* thumb bit */

_SVCall:
    movs    r0, #4
    mov     r1, lr
    tst     r0, r1
    beq     .use_msp
    mrs     r0, psp
    ldr     r1, =syscall
    bx      r1
.use_msp:
    mrs     r0, msp
    ldr     r1, =syscall
    bx      r1