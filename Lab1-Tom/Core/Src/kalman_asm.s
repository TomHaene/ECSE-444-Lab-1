

.global kalmanfilter_single_asm



kalmanfilter_single_asm:
	push {R4, LR}
	vpush {S4-S10} //Note the syntax difference when pushing a range of registers
	VLDR.F32 S1, [R0]  //q
	VLDR.F32 S2, [R0, #4] //r
	VLDR.F32 S3, [R0, #8]   //x
	VLDR.F32 S4, [R0, #12]   //p
	VLDR.F32 S5, [R0, #16]   //k

@Registers used: R2, R3,

VADD.F32 S4, S4, S1  @ p = p + q

VADD.F32 S6, S4, S2  @ p + r

VDIV.F32 S5, S4, S6 @ k = p / (p + r)

VSUB.F32 S7, S0, S3 @ (measurement - x)

VMUL.F32 S8, S5, S7 @ k * (measurement - x)

VADD.F32 S3, S3, S8 @ x + k*(measurement - x)


VMOV.F32 S10, #1
VSUB.F32 S9, S10, S5 @1 - k

VMUL.F32 S4, S9, S4 @ (1 - k) * p



VMRS R3, FPSCR


LDR R4, =0x0000000F //Checking last four bits
AND R4, R3, R4
CMP R4, #0
//If R4 is more than 0 it means at least one of the flags is 1
BGT overflow_handler

	VSTR.F32 S1, [R0]  //q
	VSTR.F32 S2, [R0, #4] //r
	VSTR.F32 S3, [R0, #8]   //x
	VSTR.F32 S4, [R0, #12]   //p
	VSTR.F32 S5, [R0, #16]   //k
B end


overflow_handler:
	//Will just return 1 if error
	@ALso need to reset the FCSCR now
	LDR R6, =0xFFFFFFF0
	// Move the value 0 into another general-purpose register (R7)
	AND R3, R3, R6
	VMSR FPSCR, R3
	MOV R0, #1


end:
	vpop {S4-S10}
	pop {R4, PC}
