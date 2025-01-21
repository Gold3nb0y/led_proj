.extern led_map
.equ GPIOC, 0x40020800
.equ GPIOC, 0x40020800
.equ GPIOC, 0x40020800
.global write_chain

.thumb
.section .text

delay:
    sub r7, r7, #1 
    bne delay 
    bx lr

debug:
    ldr r7, [r5, #0x14] 
    eor r7, r7, r2 //1 0100 0000 0000 0000 force pin 14 high
    str r7, [r5, #0x14] 
    pop {r3, r4, r5, r6, r7, pc}

write_chain:
    push {r3, r4, r5, r6, r7, lr}
    ldr r5, =GPIOC
    ldr r7, =led_map
    //b debug
    push {r7}
    mov r1, #3
    mul r0, r0, r1
    mov r1, #0
    ldr r4, =0x4000
    ldr r3, =0xBFFF
write_byte: //14 cycles + 7 + 10 = 31 + 6 + 1 = 38
    pop {r7} 
    ldr r6, [r7, r1] 
    push {r7} 
    push {r6} 
    mov r2, #8 
    mov r7, #7 
    bl delay 
    nop
    b write_bit 
write_delay: 
    mov r7, #11 
    bl delay 
write_bit: 
    sub r2, r2, #1 
    ldr r7, [r5, #0x14] 
    orr r7, r7, r4 //1 0100 0000 0000 0000 force pin 14 high
    str r7, [r5, #0x14] 
    mov r7, #1 
    pop {r6}
    push {r6}
    lsr r6, r6, r2 
    and r7, r7, r6
    cmp r7, #0 
    beq write_low 
write_high: 
    mov r7, #27
    bl delay 
    nop 
    ldr r7, [r5, #0x14] 
    and r7, r7, r3 //1 1011 1111 1111 1111 force pin 14 low
    str r7, [r5, #0x14] 
    b write_done 
write_low: //12 total + 3 from above
    nop
    mov r7, #10
    bl delay 
    ldr r7, [r5, #0x14] 
    and r7, r7, r3 //1 1011 1111 1111 1111 force pin 14 low
    str r7, [r5, #0x14] 
    mov r7, #20 
    nop
    bl delay 
    nop
    nop
write_done: 
    cmp r2, #0 
    bne write_delay 
    pop {r6}
    add r1, r1, #1 
    cmp r1, r0 
    bne write_byte 
    pop {r7} 
    pop {r3, r4, r5, r6, r7, pc}


