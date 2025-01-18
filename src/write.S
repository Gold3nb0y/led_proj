.include "common.h"

.extern led_map
.extern GPIOC

.thumb
.section .text

delay:


write_map:
    push {r3, r4, r5, r6, r7, sp, lr}
    ldr r5, [GPIOC]
    ldr r7, [led_map]
    push {r7}
    mov r3, #0xEFFF
    mov r4, #0x1000
    mul r0, r0, #3
    mov r1, #0
write_byte:
    pop {r7}
    ldr r6, [r7, r1]
    push {r7}
    mov r2, #0
write_bit:
    lsl r7, r6, 1
    and r7, r7, 1
    cmp r7, #0
    inc r2
    cmp r2, #8
    bne write_bit
    inc r1
    cmp r1, r0
    bne write_byte
    pop {r6, r7, sp, pc}
