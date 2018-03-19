# The actual code
.section .text
.global wait_us
.type wait_us, %function

wait_us:
delay_loop:
	SUB R0,R0,#1
  BNE delay_loop
  BX LR
