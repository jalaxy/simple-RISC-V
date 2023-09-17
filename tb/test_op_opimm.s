nop
lui x1, 0x1234
auipc x2, 0x5
nop
addi x3, x1, 0x567
slti x4, x2, 0x321
nop
sltiu x5, x3, 0x456
xori x6, x4, 0x789
ori x1, x3, 0x333
andi x4, x2, 0x233
slli x5, x3, 0x2
lui x4, 0xfffff
nop
nop
srli x2, x4, 4
srai x3, x4, 8
nop
nop
nop
nop
nop
add x3, x1, x4
sub x4, x1, x2
sll x5, x2, x1
slt x6, x1, x2
slt x7, x2, x1
sltu x8, x1, x2
sltu x9, x2, x1
xor x10, x1, x2
srl x11, x4, x6
sra x12, x4, x8
or x13, x2, x4
and x14, x3, x4