	section code
	j _main
	hlt
_max:
	ldsp_b 3	; param -> B (clobbers A)
	ldsp 2
	cmp $b
	jnc .L4
.L3:
	ldsp 2
	j .L1
.L4:
	ldsp 3
.L1:
	ret

_count_above:
	; allocate 1 locals on stack
	push_imm 0
	ldi $a,0
	stsp 1
	ldsp_b 3	; param -> B (clobbers A)
	ldsp 4
	cmp $b
	jz .L8
	jnc .L8
.L7:
	ldsp 1
	inc
	stsp 1
.L8:
	ldsp_b 3	; param -> B (clobbers A)
	ldsp 5
	cmp $b
	jz .L10
	jnc .L10
.L9:
	ldsp 1
	inc
	stsp 1
.L10:
	ldsp_b 3	; param -> B (clobbers A)
	ldsp 6
	cmp $b
	jz .L12
	jnc .L12
.L11:
	ldsp 1
	inc
	stsp 1
.L12:
	ldsp_b 3	; param -> B (clobbers A)
	ldsp 7
	cmp $b
	jz .L14
	jnc .L14
.L13:
	ldsp 1
	inc
	stsp 1
.L14:
	ldsp 1
.L5:
	pop $d
	ret

_main:
	; allocate 1 locals on stack
	push_imm 0
	push_imm 25
	push_imm 10
	jal _max
	pop $d
	pop $d
	stsp 1
	out

	push_imm 150
	push_imm 200
	jal _max
	pop $d
	pop $d
	stsp 1
	out

	push_imm 30
	push_imm 100
	push_imm 75
	push_imm 20
	push_imm 50
	jal _count_above
	pop $d
	pop $d
	pop $d
	pop $d
	pop $d
	stsp 1
	out

	hlt

.L15:
	pop $d
	ret

