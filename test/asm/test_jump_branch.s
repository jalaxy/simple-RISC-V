nop
mv x1, x0
mv x2, x0
addi x3, x0, 0x20
nop
nop
l1: addi x1, x1, 1
addi x2, x2, 2
blt x1, x3, l1
j l2
l2: addi x1, x1, 4
jal x4, l3
l3: nop
nop
nop
nop
