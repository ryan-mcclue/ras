@ CSI - camera, DSI - display

@ NOTE(Ryan): Learning assembly
@ 1. CISC/RISC, endianness
@ 2. word size, instruction sizing, general/special registers

@ Addressing types are just ways in which we can store/transfer data between locations
@ We have immediate, register direct, direct, 
@ register indirect:
@   1. ldr r1, [r0, #4]
@   2. ldr r1, [r0, #4]! (pre-increment r0 as well)
@   3. ldr r1, [r0], #4 (post-increment r0 as well)
@ register indirect no arithmetic like in x86, so for multiplication (array accessing):
@ mov r2, [r0, r1, lsl #2]
@ add r1, r1, #1

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
@ documentation often in the cortex-specific manual

@ SomePiGuy@gmail.com

@ could update stack with: str! r0, sp, 4


@ IMPORTANT(Ryan): Although '.syntax unified' allows us to not prepend immediates with '#'
@ I don't like having to append 's' to instructions to change flags. 
@ So, prefer default '.syntax divided'

@ IMPORTANT(Ryan): What I know about the PI's GPU bootloader:
@   1. Handles clock initialisation
@   2. Copies our binary from flash to RAM at 0x00008000
@ Writing to VTOR register to alter isr-vector table location is only available for cortex-m.
@ So, we must copy isr-vector table to default 0x00000000

@ TODO(Ryan): If arm procedure call standard says SP 8-byte aligned for ldrd,
@ does this mean always have to push/pop in pairs? yes!
@ push {r4, r5} ⟷ pop {r4, r5} (pop works from right to left)

.extern _svc_stack 
.extern _irq_stack 
.extern _init_ram_addr
.extern _exception_table_addr

.section .init
.global _start
_start:
  @ TODO(Ryan): Implement basic versions of all exceptions.
  @ Currently only implement reset and irq.
  b reset_handler
  b undefined_instruction_handler
  b software_interrupt_handler 
  b prefetch_abort_handler 
  b data_abort_handler
  b reserved_handler
  b irq_handler
  b fiq_handler

@ IMPORTANT(Ryan): Necessary to ensure fixed memory location 
reset_handler: .word reset
undefined_instruction_handler: .word undefined_instruction
software_interrupt_handler: .word software_interrupt
prefetch_abort_handler: .word prefetch_abort
data_abort_handler: .word data_abort
reserved_handler: .word reserved
irq_handler: .word irq
fiq_handler: .word fiq

reset:
  ldr r0, =_init_ram_addr
  ldr r1, =_exception_table_addr

  @@ Copy over branch instructions 
  ldmia r0!, {r2, r3, r4, r5, r6, r7, r8, r9}
  stmia r1!, {r2, r3, r4, r5, r6, r7, r8, r9}

  @@ Copy over vectors (necessary because of pc-relative branching)
  ldmia r0!, {r2, r3, r4, r5, r6, r7, r8, r9}
  stmia r1!, {r2, r3, r4, r5, r6, r7, r8, r9}

  b main

@ TODO(Ryan): When does the NVIC come into play? 
@ An easier way to distinguish between multiple interrupts?
@ How is the built-in interrupt handler for ARM different to a GIC?
irq:
  @ TODO(Ryan): Do I have to clear the interrupt as well?
  @ pop any shared registers
  @ pop lr
  subs pc, lr, #4 @ this will return modes as well using spsr register

undefined_instruction: bkpt 
software_interrupt: bkpt
prefetch_abort: bkpt
data_abort: bkpt
reserved: bkpt
fiq: bkpt

@ TODO(Ryan): Investigate how to analyse linux source code to obtain this information. 
@ Methods to obtain non-discoverable hardware: 
@  1. Directly in code
@  2. ACPI tables (used by x86 provided by firmware, i.e motherboard BIOS)
@  3. Device tree (.dts compiled to .dtb) (arch/<arm>/boot/dts)
@ inspect on system inside /sys/firmware

@ reg = <base-address size>

@ the device tree specification describes generic format?
@ the compatible property describes the device tree binding that is being implemented

@ board in .dts, soc in .dtsi

@ label: node-name
@ labels used as phandles, i.e. way for properties to reference node.
@ this allows to override node property without replicating node hierarchy  
@
@ sub nodes will typically take node@0 syntax?

@ uboot and barebox sync/copy linux kernel device tree files?
@ Perhaps, have to load in VM and obtain with user-space programs like $(gpioinfo)?

@ IMPORTANT(Ryan): The CPU is a coprocessor of the GPU, therefore different peripheral
@ access addresses depending on source. Due to poor documentation, easier to inspect
@ base addresses inside of linux device tree files.
@   $(gcc -E -P -I/home/ryan/prog/sources/linux/include \
@       -x assembler-with-cpp bcm2836-rpi-2-b.dts -o bcm2836.i)
#define MMIO_BASE 0x3F000000

#define GPIO_OFFSET 0x200000 
#define UART0_OFFSET 0x201000

.section .text
main:
  ldr r0, =(MMIO_BASE + GPIO_OFFSET)
  ldr r1, =(MMIO_BASE + UART0_OFFSET)

  @@ GPPUD - disable all gpio pull up/down resistors
  mov r2, #0
  str r2, [r0, #0x94]
  bl delay_150

  @@ GPPUDCLK0 - enable clock signal for pins 14 and 15
  ldr r2, =((1 << 14) | (1 << 15))
  str r2, [r0, #0x98]
  bl delay_150

  @@ UART0_CR - disable uart
  mov r2, #0
  str r2, [r1, #0x30]

  @@ UART0_ICR - clear pending interrupts
  ldr r2, =0x7FF
  str r2, [r1, #0x44]

  @ Baud = 115200
  @@ UART0_IBRD - set integer part of baud rate
  mov r2, #1
  str r2, [r1, #0x24]

  @ NOTE(Ryan): UART0_FBRD - set fractional part of baud rate
  mov r2, #40
  str r2, [r1, #0x28]
 
  @ IMPORTANT(Ryan): We could set to 0x60 with 8-bit xfer, but disable FIFO if using interrupts
  @ as we would just directly check the data register.
  @ NOTE(Ryan): UART0_LCRH - 8bits, 1 stop bit, no parity 
  ldr r2, =(1 << 4 | 1 << 5 | 1 << 6) 
  str r2, [r1, #0x2C]
 
  @ NOTE(Ryan): UART0_IMSC - mask all interrupts
  ldr r2, =#0x7F2
  str r2, [r1, #0x38]

  @ NOTE(Ryan): UART0_CR - enable uart
  ldr r2, =(1 << 0 | 1 << 8 | 1 << 9)
  str r2, [r1, #0x30]

  @ UART_FR = 0x18
  @ UART_DR = 0x0
loop:
  mov r2, #'H'
  str r2, [r1]
  bl delay_150
  b loop

write_uart_ch:
  ldr r3, [r1, #0x18]
  and r3, r3, #0x20
  cmp r3, #0
  bne write_uart_ch
  str r2, [r1]
  mov pc, lr @ could also do: bx lr

read_uart_ch:
  ldr r3, [r1, #0x18]
  and r3, r3, #0x10
  cmp r3, #0
  bne read_uart_ch
  @ TODO(Ryan): Do we need to and to only get lower 8 bits? 

  ldr r2, [r1]
  mov pc, lr

@ simple echo:
@  bl read_char
@  bl write_char

@ TODO(Ryan): More robust way of handling subroutine variables
delay_150:
  mov r9, #0
  delay_loop:
    add r9, r9, #1 
    cmp r9, #150
    bne delay_loop
  mov pc, lr

  nop 

#if 0
#define IRQ_MODE 0b10010
#define SVC_MODE 0b10011

  mrs r0, cpsr
  and r0, r0, #0xffffff00
  @ IRQ_MASKED | FIQ_MASKED | THUMB_DIS | IRQ_MODE
  orr r0, r0, #(0b110 | IRQ_MODE)
  msr cpsr, r0
  ldr sp, =_irq_stack

  mrs r0, cpsr
  and r0, r0, #0xffffff00
  @ IRQ_ENABLED | FIQ_MASKED | THUMB_DIS | SVC_MODE
  orr r0, r0, #(0b010 | SVC_MODE)
  msr cpsr, r0
  ldr sp, =_svc_stack


#define SYSTEM_CONTROL_COPROCESSOR p15 


@ ABI is r0-r3 arguments then stack. r4-r12 will not change we calling a function


@ B000 is offset for interrupt controller register?

@ TODO(Ryan): Investigate the implications of starting in SVC mode and reasons for wanting
@ to go to user mode? Is this only applicable for OS and not bare metal? (similar to real 16-bit mode for x86)
@ kernel would operate in supervisor mode
@ in user mode, we cannot directly modify the CPSR mode flags. so we get to the other modes
@ from user mode with interrupts or exceptions
@ if requesting an interrupt, we would go into IRQ mode
@ abort mode is invalid memory
@ undefined mode is illegal instruction

@ interrupts are different to subroutines in that they operate in a unique mode and
@ are typically asynchronous, i.e. called from some external hardware 


  @ NOTE(Ryan): Read main id register 
  mrc SYSTEM_CONTROL_COPROCESSOR, 0, r0, c0, c0, 0
  lsr r1, r0, #4
  ldr r2, =#0xFFF
  and r1, r2 

  @ We should also set up stack address
  @ Also enable FPU

  @ read. coprocessor access control register. 
  @ we enable access rights for coprocessor 10 and 11 in all modes
  @ then actually enable
ldr r0, =(0xF << 20)
mcr p15, 0, r0, c1, c0, 2

  @ NOTE(Ryan): Read multiprocessor affinity register
  mrc SYSTEM_CONTROL_COPROCESSOR, 0, r0, c0, c0, 5
  and r0, r0, #0x3
  cmp r0, #0x0
  bne halt
halt:
  @ NOTE(Ryan): wait-for-event applicable only for multiprocessor environment
  wfe
  b halt
   

@ IMPORTANT(Ryan): By default, the linker makes .text section executable - expecting alignment. 
@ Therefore, must put data definitions in other sections to avoid unaligned opcodes.
.section .data
message:
  @ TODO(Ryan): Investigate modes: User mode, FIQ/IRQ (interrupts), supervisor (on reset)
  @ non-user modes may have access to register banks that are distinct, e.g R13_IRQ, i.e. R13 in IRQ mode is distinct from R13 in user mode
  @ we typically leave user mode in interrupts and exceptions (illegal operation - could be illegal instruction, access protected memory)
  @ arm defines location of vectors starting from 0x00000000
  .asciz "       NZCV____________________EAIFTMMMMM" 

array: .word 4, 5, -9, 11, 20 
array_len: (. - array) / 4


@stack:
@  .space 1024 * 4

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

@ IMPORTANT(Ryan): All instructions are conditional, e.g. suffixed with condition like:
@ cmp r3, #0
@ addeq r5, r5, #1

@ TODO(Ryan): How does ring-levels come into play with bare-metal?

@ risc will have more registers as typically less instructions to work directly with memory, only registers

@ TODO(Ryan): Can be in arm mode (32-bit) or thumb mode (16-bit; however thumb2 can use some 32-bit for branching?)
@ thumb gives less ram accesses and instruction cache misses, cpu uses less power
@
@ IMPORTANT(Ryan): Although the following load/store sub-word size, they zero-extend:
@ ldr (word), ldrh (half word), ldrsh (signed half word), ldrb (byte)
@ For actual sub-word operations, use MOVT (for high half) and BFI/UBFX (for generic)
@
@_start:
@  ldr r0, =GPIO_BASE ; the '=' variant of ldr is a pseudo-op

@ to indicate version at end:
@ .word 0xdeadbeef
@ .fill 510 - (. - entry), 0

@ IMPORTANT(Ryan): When writing assembly, go from python while(), to ifs with gotos 

@ sp is top of stack (so lower address)
@ fp is bottom of stack
@ r4 - r11 preserved
@ freely use r0-r3, r12
@ if require other registers, save and restore on stack
@ if calling another, save and restore lr

@ vim:ft=armv5
#endif
