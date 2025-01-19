.include "common.h"

.extern led_map
.equ GPIOC, 0x40020800
.equ GPIOC, 0x40020800
.equ GPIOC, 0x40020800
.global write_chain

.thumb
.section .text

//takes MS to delay in r7
//3n+1 cycles where n is r7
delay:
    sub r7, r7, #1 //1
    bne delay //2 one on last
    bx lr

debug:
    ldr r7, [r5, #0x14] //2
    eor r7, r7, r2 //1 0100 0000 0000 0000 force pin 14 high
    str r7, [r5, #0x14] //2
    pop {r3, r4, r5, r6, r7, pc}


// total overhead for high = 1.125 microseconds
// total overhead for low = .75 microseconds
// bytes to write stored in r0
// at a rate of 84mHz I have around 33 clock cycles for the initial low period
// write to GPIOC 14
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
    pop {r7} //2
    ldr r6, [r7, r1] //2
    push {r7} //2
    mov r2, #0 //1 
    mov r7, #7 //1
    bl delay //3 + 1
    nop
    b write_bit //2
write_delay: //cur 7+5 for end
    mov r7, #11 //1
    bl delay //3 + 1 + 1 + 21 = 26; 26 + 12 = 38
write_bit: //5 cycles for low period 3 for high
    ldr r7, [r5, #0x14] //2
    orr r7, r7, r4 //1 0100 0000 0000 0000 force pin 14 high
    str r7, [r5, #0x14] //2
    mov r7, #1 //1
    and r7, r6, r7 //1
    cmp r7, #0 //1
    beq write_low // 1 or 2 if true
write_high: //12 total + 3 from above
    mov r7, #30
    bl delay //3
    nop //+1 to make 67
    ldr r7, [r5, #0x14] //2
    and r7, r7, r3 //1 1011 1111 1111 1111 force pin 14 low
    str r7, [r5, #0x14] //2
    b write_done //2
write_low: //12 total + 3 from above
    nop
    mov r7, #13
    bl delay //3
    ldr r7, [r5, #0x14] //2
    and r7, r7, r3 //1 1011 1111 1111 1111 force pin 14 low
    str r7, [r5, #0x14] //2
    // this account for the other low period, now both should be synced up
    mov r7, #20 //1
    nop
    bl delay //3 + 1

    nop
    nop
write_done: //cur 2
    add r2, r2, #1 //1
    lsr r6, r6, #1 //1
    cmp r2, #8 //1
    bne write_delay //2
    add r1, r1, #1 //1
    cmp r1, r0 //1
    bne write_byte //2
    pop {r7} //2
    pop {r3, r4, r5, r6, r7, pc}
