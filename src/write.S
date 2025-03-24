.extern led_map
.equ GPIOC, 0x40020800
.global write_chain

.thumb
.section .text

delay:
    push {r5, lr}
    mov r5, r0
_delay:
    sub r5, r5, #1 
    bne _delay 
    pop {r5, pc}

//debug:
//    ldr r7, [r5, #0x14] 
//    eor r7, r7, r2 //1 0100 0000 0000 0000 force pin 14 high
//    str r7, [r5, #0x14] 
//    pop {r3, r4, r5, r6, r7, pc}

//r0 contains leds to write
//#4 is number of leds
//#8 is count
//#8 is byte count
write_chain:
    push {r4, r5, r6, r7, lr}
    mov r7, sp
    sub sp, sp, #0x20
    str r0, [sp, #0x4]
    mov r1, #3
    mul r0, r0, r1
    str r0, [sp, #0x18]
    mov r0, #0
    str r0, [sp, #0x8]
    str r0, [sp, #0xc]
write_byte: //14 cycles + 7 + 10 = 31 + 6 + 1 = 38
    ldr r5, =led_map
    ldr r0, [sp, #8]
    mov r1, #3
    mul r0, r0, r1
    ldr r1, [sp, #0xc]
    add r0, r0, r1
    ldr r6, [r6, r0] 
    str r6, [sp, #0x14]
    add r1, r1, #1
    cmp r1, #3
    bne bend
    mov r1, #0
    ldr r0, [sp, #8]
    add r0, #1
    str r0, [sp, #8]
bend:
    str r1, [sp, #0xc]
    mov r0, #8 
    str r0, [sp, #0x10]
    mov r0, #4 
    bl delay 
    nop
    b write_bit 
write_delay: 
    mov r0, #11 
    bl delay 
write_bit: 
    ldr r0, [sp, #0x10]
    sub r0, r0, #1 
    str r0, [sp, #0x10]
    ldr r5, =GPIOC
    ldr r6, [r5, #0x14] 
    ldr r4, =0x4000
    orr r6, r6, r4 //1 0100 0000 0000 0000 force pin 14 high
    str r6, [r5, #0x14] 
    ldr r0, [sp, #0x10]
    ldr r6, [sp, #0x14]
    mov r5, #1
    lsr r6, r6, r0 
    and r5, r5, r6
    cmp r5, #0 
    beq write_low 
write_high: 
    mov r0, #17
    bl delay 
    ldr r5, =GPIOC
    ldr r6, [r5, #0x14] 
    ldr r4, =0xBFFF
    and r6, r6, r4 //1 1011 1111 1111 1111 force pin 14 low
    str r6, [r5, #0x14] 
    b write_done 
write_low: //12 total + 3 from above
    nop
    mov r0, #2
    bl delay 
    ldr r5, =GPIOC
    ldr r6, [r5, #0x14] 
    ldr r4, =0xBFFF
    and r6, r6, r4 //1 1011 1111 1111 1111 force pin 14 low
    str r6, [r5, #0x14] 
    mov r0, #17 
    bl delay 
write_done: 
    ldr r0, [sp, #0x10]
    cmp r0, #0 
    bne write_delay 
    ldr r0, [sp, #0x8]
    mov r1, #3
    mul r0, r0, r1
    ldr r1, [sp, #0xc]
    add r0, r0, r1
    ldr r1, [sp, #0x18]
    cmp r0, r1 
    bne write_byte 
    mov sp, r7
    pop {r4, r5, r6, r7, pc}


