.global USBCTRL_IRQ
//.global SW5_IRQ

    .thumb_func
USBCTRL_IRQ:
//SW5_IRQ:
    ldr     r2, =handle_usb_irq
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

.global usb_ret

    .thumb_func
usb_ret:
    movs    r0, #0
    svc     #3