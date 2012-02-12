/*****************************************************************************
Copyright (c) 2011 Abhin Chhabra, Jordan Woehr, Lisa Graham, Ryan Bray

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************/

.code 32

// Export these for startup.c
.global fiq_handler
.global default_handler
.global spurious_handler

// Initialization routine in C
.extern init

// ARM defines
.equ mode_fiq, 0x11
.equ mode_irq, 0x12
.equ mode_svc, 0x13
.equ i_bit,    0x80
.equ f_bit,    0x40

// Processor peripheral defines
.equ interrupt_vector_register,  0xfffff100
.equ end_of_interrupt_register,  0xfffff130

// Other constants
.equ stack_init_value, 0xaaaaaaaa

// Define vectors
.section .vectors, "ax"
	ldr   pc, =reset_handler
	ldr   pc, =undef_handler
	ldr   pc, =swi_handler
	ldr   pc, =pabort_handler
	ldr   pc, =dabort_handler
	ldr   pc, =0               /* reserved */
	ldr   pc, =irq_handler
	ldr   pc, =fiq_handler

// First real code to execute
.section .init, "ax"
reset_handler:
	ldr   sp, =__stack_top__ // Set temporary stack pointer
	ldr   r0, =init          // Initialize processor
	mov   lr, pc
	bx    r0

	// Configure processor modes
	// The processor will normally run in supervisor mode. When an interrupt
	// occurs, the processor will be in irq mode and will disable supervisor
	// interrupts and execute the interrupt in supervisor mode using its stack.

	// Interrupts should all already be disabled ...
	// Disable interrupts in fiq mode
	msr   cpsr_c, #(mode_fiq | i_bit | f_bit)
	// Disable interrupts in irq mode
	msr   cpsr_c, #(mode_irq | i_bit | f_bit)
	// Set the irq stack (very top)
	ldr   sp, =__irq_stack_top__
	// Disable interrupts in supervisor mode
	msr   cpsr_c, #(mode_svc | i_bit | f_bit)
	// Set supervisors stack (just below the irq stack)
	ldr   sp, =__svc_stack_top__

// Initialize the stack
	ldr     r0, =__stack_start__
    ldr     r1, =__stack_top__
    ldr     r2, =stack_init_value
loop_stack_fill:
    cmp     r0, r1
    stmltia r0!, {r2}
    blt     loop_stack_fill

// Relocation data and ram functions to ram
	ldr     r1, =_etext
	ldr     r2, =_data
	ldr     r3, =_edata
loop_relocate:
	cmp     r2, r3
	ldrlo   r0, [r1], #4
	strlo   r0, [r2], #4
	blo     loop_relocate

// Zero the bss section
	mov     r0, #0
	ldr     r1, =__bss_start__
	ldr     r2, =__bss_end__
loop_bss_init:
	cmp     r1, r2
	strlo   r0, [r1], #4
	blo     loop_bss_init

// Jump to main
	ldr   r0, =main
	mov   lr, pc
	bx    r0

c_exited:
	b     c_exited

undef_handler:
	b     undef_handler

swi_handler:
	b     swi_handler

pabort_handler:
	b     pabort_handler

dabort_handler:
	b     dabort_handler

irq_handler:
	// In IRQ mode, first thing to do is save the return address
	sub   lr, lr, #4
	// We also need a single register for the irq handler address
	stmfd sp!, {r0, lr}
	// Using r14 (lr) as a temp until branching into the handler
	ldr   r14, =interrupt_vector_register
	// Make sure interrupt_vector_register is read and written to support protect mode
	ldr   r0, [r14]
	str   r0, [r14]
	// Interrupts are executed in supervisor mode with the I bit set
	// That is, they use the same stack as most everything else and can't be interrupted
	msr   cpsr_c, #(mode_svc | i_bit | f_bit)
	mov   lr, pc           // Save return address
	bx    r0               // Branch to handler
	msr   cpsr_c, #(mode_irq | i_bit | f_bit)  // Back to irq mode
	ldr   r0, =end_of_interrupt_register       // Write to the end of interrupt register
	str   r0, [r0]
	ldmia sp!, {r0, pc}^    // Restore registers and return to normal program flow

fiq_handler:
	b     fiq_handler

default_handler:
	b     default_handler

spurious_handler:
	b     spurious_handler
