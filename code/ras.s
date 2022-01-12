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

@ an arm coprocessor is a bit of a misnomer. 
@ it's just extended functionality, e.g. MMU, hardware floating point etc.
@ p15 is system control (cache, MMU), p14 is debug (breakpoint)
@ generic coprocessor instruction like: MCR p10, 0, r1, cr2, 0 (p10 is single-precision coprocessor, p11 is double)
@ or may have more informative like: FMSR s4, r1

@ mrc p15, 0, r1, c0, c0, 0
@ (op-code, co-processor lhs/rhs, optional extra info)
@ 
@ Command format ：MRC P15,0,R0,C0,C0,0, 
@ Determined by the manufacturer: 30**～24**
@ Product subnumber: 23**～20**
@ ARM System version number: 19**～16**
@ Product master number: 15**～4**
@ Processor version number: 3**～0**


.global entry
.global main

@ 10011 --> starts in SVC (always on reset)

#define SYSTEM_CONTROL_COPROCESSOR p15 

.section .text
entry:
  @ NOTE(Ryan): Some registers have pre-declared names, e.g. r10 is sl (stack limit)
  ldr r10, =message
main:
  mrc SYSTEM_CONTROL_COPROCESSOR, 0, r0, c0, c0, 0
  lsr r1, r0, #4
  ldr r2, =#0xFFF
  and r1, r2 
   

@ IMPORTANT(Ryan): By default, the linker makes .text section executable - expecting alignment. 
@ Therefore, must put data definitions in other sections to avoid unaligned opcodes.
.section .data
message:
  @ TODO(Ryan): Investigate modes: User mode, FIQ/IRQ (interrupts), supervisor (on reset)
  @ non-user modes will have access to 'private-copies, i.e. physically distinct from user registers' of registers, e.g R13_SVC, R13_IRQ 
  @ if IRQ/FIQ bits set, interrupts will be allowed
  @ we typically leave user mode in interrupts and exceptions (illegal operation - could be illegal instruction, access protected memory)
  @ arm defines location of vectors starting from 0x00000000
  .asciz "       NZCV____________________EAIFTMMMMM" 
  .word 4, 5, -9, 11, 20 

@ IMPORTANT(Ryan): PC when debug stepping holds current instruction being executed.
@ However, during execution, it will hold two instructions ahead of current instruction.
@ This becomes important for analysing PC relative (ldr r0, [pc + #12])

@ We have a literal pool, which is an area of memory residing in same place as code
@ Arm can only load an immediate 8-bit value (as has fixed sized instruction, and each instruction has shifter, comparison, etc.)
@ For ease, just use ldr r0, =511 construct?

@ C performs arithmetic shift for signed and logical for unsigned

@ inspect with $(objdump -d file.elf)
@ verify on bin with $(hexdump -C file.bin)

@ Reading assembly docs:
@ Mnemonic{s}{Condition} Rd, Rn, <Operand2>
@ {} is optional. <condition> can be EQ. <operand> means various addressing modes possible
@ if instruction suffixed with 's', e.g. adds ; condition flags set 
@ instruction suffixed with condition, e.g. addEQ ; perform if 


@ TODO(Ryan): How does ring-levels come into play with bare-metal?

@ risc will have more registers as typically less instructions to work directly with memory, only registers
@
@ TODO(Ryan): Can be in arm mode (32-bit) or thumb mode (16-bit; however thumb2 can use some 32-bit for branching?)
@ thumb gives less ram accesses and instruction cache misses, cpu uses less power
@
@ ldr (word), ldrh (half word), ldrsh (signed half word), ldrb (byte)
@
@_start:
@  ldr r0, =GPIO_BASE ; the '=' variant of ldr is a pseudo-op

@ to indicate version at end:
@ .org 0xf0
@ .word 0xdeadbeef
