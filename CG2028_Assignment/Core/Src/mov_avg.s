/*
 * mov_avg.s
 *
 * CG2028 Assignment starter file.
 */
.syntax unified
.cpu cortex-m4
.thumb
.global ewma_filter
.type ewma_filter, %function

.text
.align 2

@ CG2028 Assignment
@ (c) ECE NUS
@ Write Student 1's Name here: Leon Matthew Wei (A0320416U)
@ Write Student 2's Name here: Li Yi An (A0334942B)
@
@ Function prototype:
@   int ewma_filter(int new_data, int old_output, int alpha_percent);
@
@ ARM calling convention:
@   R0 = new_data       (signed integer sensor sample)
@   R1 = old_output     (previous filtered output)
@   R2 = alpha_percent  (integer from 0 to 100)
@   Return R0 = (alpha_percent * new_data
@                + (100 - alpha_percent) * old_output) / 100
@
@ Notes:
@ - Use signed integer arithmetic.
@ - Integer division must truncate towards zero, matching C integer division.
@ - Preserve all callee-saved registers that you use (R4-R11).
@ - Do not call a C helper function and do not use floating-point instructions.
@
@ Register table:
@   R0 ...
@   R1 ...
@   R2 ...
@   R3 ...
@   R4 ...
@
@ Write your program from here.
ewma_filter:
    PUSH {r4-r7, lr}

    @ TODO: Implement the EWMA low-pass filter in pure ARM assembly.
    @ output = [alpha_percent x new_data + (100 - alpha_percent) x old_output] / 100
	@ R0 - new_data   (signed 32-bit integer)
	@ R1 - old_output (signed 32-bit integer)
	@ R2 - alpha_percent (integer, 0 to 100)
	@ Return R0 - new filtered output (signed 32-bit integer)

	/*
	MUL  R0, R2, R0    // R0 = alpha_percent[R2] * new_data[R0]
	RSB  R2, R2, #100  // R2 = 100 - alpha_percent[R2]
	MUL  R2, R2, R1    // R2 = (100 - alpha_percent)[R2] * old_output[R1]
	ADD  R0, R0, R2    // R0 = (alpha_percent x new_data)[R0] + ((100 - alpha_percent) x old_output)[R2]
	MOV  R3, #100
	SDIV R0, R0, R3    // R0 = (alpha_percent x new_data + (100 - alpha_percent) x old_output)[R0] / 100
	*/

	SMULL R4, R5, R2, R0  // alpha_percent[R2] * new_data[R0]
	RSB   R2, R2, #100
	SMULL R6, R7, R2, R1  // (100 - alpha_percent)[R2] * old_output[R1]

	ADDS  R4, R4, R6      // add low words and sets flag
	ADC   R5, R5, R7      // add high words with carry

	MOV   R0, R4          // numerator low word
    MOV   R1, R5          // numerator high word

    MOV   R2, #100        // divisor low word
    MOV   R3, #0          // divisor high word

    BL    __aeabi_ldivmod // quotient is in R1:R0; signed 32-bit result is in R0

    POP  {r4-r7, pc}

.size ewma_filter, .-ewma_filter
