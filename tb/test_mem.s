nop
addi x1, x0, 0x123
addi x4, x0, 0x456
lui x3, 0x10010
nop
nop
sw x1, 0(x3)
sw x4, 4(x3)
nop
nop
nop
lw x2, 0(x3)
lw x5, 4(x3)
nop
nop
nop
