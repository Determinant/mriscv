.global main
.text

main:
	# forward type 1 (executor -> decoder)
	addi x3, x0, 42 # x3 = 42
	addi x3, x3, 1 # x3++ (=43)
	nop
	nop
	nop
	nop
	# forward type 2 (memory -> decoder, with a stall)
	sw x3, -4(x2) # 0x2000000[-4] = 43
	nop
	nop
	nop
	nop
	lw x4, -4(x2) # x3 = 0x2000000[-4] (=43)
	addi x5, x4, 1 # x4 = x3 + 1 (=44)
	nop
	nop
	nop
	nop
	# forward type 2b (memory -> decoder, no stall)
	addi x6, x0, 20 # x6 = 20
	nop
	addi x6, x6, 1 # x6++ (x6 = 21)
	nop
	nop
	nop
	nop
	sw x5, -4(x2)
	lw x4, -4(x2) # x4 = 44
	addi x4, x4, 1 # x4++ (=45)
	sw x4, -4(x2) # 0x2000000[-4] = 45
	nop
	nop
	nop
	nop
	# forward type 3 (register file bypass)
	lw x3, -4(x2) # x3 = 45
	nop
	nop
	addi x3, x3, 1 # x3++ (=46)
	nop
	nop
	nop
	nop
	jalr x0, 0(x1)
