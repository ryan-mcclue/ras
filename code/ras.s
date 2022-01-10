@ NOTE(Ryan): Learning assembly
@ 1. CISC/RISC, endianness
@ 2. word size, instruction sizing, general/special registers

@ Addressing types are just ways in which we can store/transfer data between locations
@ We have immediate, register direct, direct, 
@ register indirect:
@   1. ldr r1, [r0 + #4]
@   2. ldr r1, [r0 + #4]! (pre-increment r0 as well)
@   3. ldr r1, [r0], #4 (post-increment r0 as well)

@ twos-complement is used for signed numbers (MSB is negative). 
@ however, we can also treat as unsigned
@ so, require different status flags to indicate for each.
@ carry flag if too big for unsigned (however, also set if subtraction >= 0)
@ overflow if too big for signed

@ barrel shifter in ALU allows to include shifts in instructions, e.g. move and multiply

#define LOCAL_BASE 0x40000000 

.global entry
.global main

.section .text
entry:
  ldr r10, =message
main:
  mov r0, #0x1
  add r1, r0, r0

@ IMPORTANT(Ryan): By default, the linker makes .text section executable - expecting alignment. 
@ Therefore, must put data definitions in other sections to avoid unaligned opcodes.
.section .data
message:
  .asciz "       NZCV____________________EAIFTMMMMM" 
  .word 4, 5, -9, 11, 20 

@ IMPORTANT(Ryan): PC when debug stepping holds current instruction being executed.
@ However, during execution, it will hold two instructions ahead of current instruction.
@ This becomes important for analysing PC relative (ldr r0, [pc + #12])

@ We have a literal pool, which is an area of memory residing in same place as code
@ Arm can only load an immediate 8-bit value.
@ For ease, just use ldr r0, =511 construct?

@ C performs arithmetic shift for signed and logical for unsigned

@ inspect with $(objdump -d file.elf)
@ verify on bin with $(hexdump -C file.bin)

@ TODO(Ryan): How does ring-levels come into play with bare-metal?

@
@ risc will have more registers as typically less instructions to work directly with memory, only registers
@
@ TODO(Ryan): Can be in arm mode (32-bit) or thumb mode (16-bit; however thumb2 can use some 32-bit for branching?)
@ thumb gives less ram accesses and instruction cache misses, cpu uses less power
@
@ ldr (word), ldrh (half word), ldrsh (signed half word), ldrb (byte)
@
@_start:
@  ; Pseudo-op that handles larger than 8 bit 
@  ldr r0, =GPIO_BASE ; without '=' would be the address
@  ldr r1, =
@  ; # is immediate value
