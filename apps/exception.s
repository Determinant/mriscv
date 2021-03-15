.global main
.text

handler:
	csrrs x4, mcause, x0
	add x3, x3, x4
	mret

main:
	addi x3, x0, 0
	la x4, handler
	csrrw x0, mtvec, x4
	ecall
	addi x3, x3, 2
	jalr x0, 0(x1)
