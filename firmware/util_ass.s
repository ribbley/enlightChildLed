.section .text
.global wait_0h
.type wait_0h, %function

wait_0h:
	LDR R0,=0x00000004
	SUB R0,R0,#1
  BNE wait_0h
  BX LR

.section .text
.global wait_0l
.type wait_0l, %function

wait_0l:
	LDR R0,=0x00000009
	SUB R0,R0,#1
  BNE wait_0l
  BX LR

.section .text
.global wait_1h
.type wait_1h, %function

wait_1h:
	MOV R0,#8
	SUB R0,R0,#1
  BNE wait_1h
  BX LR

.section .text
.global wait_1l
.type wait_1l, %function

wait_1l:
	MOV R0,#5
	SUB R0,R0,#1
  BNE wait_1l
  BX LR

.section .text
.global wait_reset
.type wait_reset, %function

wait_reset:
	LDR R0,=0x00000500
	SUB R0,R0,#1
  BNE wait_reset
  BX LR
