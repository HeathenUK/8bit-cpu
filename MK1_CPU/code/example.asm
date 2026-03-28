#bank ".instr"
_max:
	ldsp 3
	mov $a $b
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
	ldi $a 0
	st $a 0
	ldsp 3
	mov $a $b
	ldsp 4
	cmp $b
	jz .L8
	jnc .L8
.L7:
	ldsp 0
	inc
	st $a 0
.L8:
	ldsp 3
	mov $a $b
	ldsp 5
	cmp $b
	jz .L10
	jnc .L10
.L9:
	ldsp 0
	inc
	st $a 0
.L10:
	ldsp 3
	mov $a $b
	ldsp 6
	cmp $b
	jz .L12
	jnc .L12
.L11:
	ldsp 0
	inc
	st $a 0
.L12:
	ldsp 3
	mov $a $b
	ldsp 7
	cmp $b
	jz .L14
	jnc .L14
.L13:
	ldsp 0
	inc
	st $a 0
.L14:
	ldsp 0
.L5:
	ret

_main:
	ldi $a 10
	push $a
	ldi $a 25
	push $a
	jal _max
	pop $d
	pop $d
	st $a 0
	out

	ldi $a 200
	push $a
	ldi $a 150
	push $a
	jal _max
	pop $d
	pop $d
	st $a 0
	out

	ldi $a 50
	push $a
	ldi $a 20
	push $a
	ldi $a 75
	push $a
	ldi $a 100
	push $a
	ldi $a 30
	push $a
	jal _count_above
	pop $d
	pop $d
	pop $d
	pop $d
	pop $d
	st $a 0
	out

	hlt

.L15:
	ret

