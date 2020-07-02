.data
    n: .word 20 # You can change this number
    
.text
.globl __start

FUNCTION:
    # Todo: define your own function in HW1
  Recursion:
    addi sp, sp, -16
    sw x1, 8(sp)
    sw x10, 0(sp)
    srli x29, x10, 1
    bne x29, x0, L1
    addi x12, x0, 1
    addi sp, sp, 16
    jalr x0, 0(x1)
  
  L1:
    srli x10, x10, 1
    jal x1, Recursion
    addi x6, x10, 0
    lw x1, 8(sp)
    lw x10, 0(sp)
    addi sp, sp, 16
    add x28, x10, x0 ##copy
    addi x7, x0, 4  #multiplied by 4
    mul x10, x6, x7 #
    add x10, x10, x28 
    add x10, x10, x28 
    add x12, x10, x0
    jalr x0 0(x1)    

# Do not modify this part!!! #
__start:                     #
    la   t0, n               #
    lw   x10, 0(t0)          #
    jal  x1,FUNCTION         #
    la   t0, n               #
    sw   x10, 4(t0)          #
    addi a0,x0,10            #
    ecall                    #
##############################