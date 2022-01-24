#pragma once

#define PAGE_SHIFT 12 
#define TABLE_SHIFT 9
#define SECTION_SHIFT (PAGE_SHIFT + TABLE_SHIFT)

#define PAGE_SIZE (1 << PAGE_SHIFT)
#define TABLE_SIZE (1 << TABLE_SHIFT)
#define SECTION_SIZE (1 << SECTION_SHIFT)

#define LOW_MEMORY (2 * SECTION_SIZE)

delay: @@ void (u32 count)
  sub r0, r0, #1  
  cmp r0, #0
  bne delay
  bx lr

put32: @@ void (u32 *addr, u32 value)
  str r1, [r0] 
  bx lr

get32: @@ u32 (u32 *addr)
  ldr r0, [r0]
  bx lr

@ TODO(Ryan): Are we incrementing by word-size to satisfy alignment? Why? Where? 
memzero: @@ void (u32 *src, u32 len)
  mov r2, #0
zeroloop:
  str r2, [r0], #4
  sub r1, r1, #4
  bgt zeroloop
  bx lr
  
main:
  bl uart_init
  ldr r0, ="hi there"
  bl uart_send_str

  loop: 
    bl uart_recv_str
    bl uart_send_str
    b loop
  

@ halt cores, (ldr sp, #LOW_MEMORY), jump to main

@ vim:ft=armv5
