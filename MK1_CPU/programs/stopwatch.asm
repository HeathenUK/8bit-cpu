; Stopwatch with PA1 buzzer — DDRA=0 normally, DDRA=0x02 only during beep

.via_dly:
	dec
	jnz .via_dly
	clr $a
	exw 0 0
	exw 0 2

	; Configure SQW
	exrw 2
	ldi $a, 0x01
	exw 0 2
	ldi $a, 0x03
	exw 0 2
	ldi $a, 0xD0
	jal __i2c_sb
	ldi $a, 0x0E
	jal __i2c_sb
	clr $a
	jal __i2c_sb
	jal __i2c_sp

.s1:
	exrw 1
	tst 0x01
	jz .s2
	j .s1
.s2:
	exrw 1
	tst 0x01
	jnz .cal
	j .s2

.cal:
	ldi $b, 0
.cal_hi_ovf:
	clr $a
.cal_hi_inner:
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	dec
	jnz .cal_hi_inner
	mov $b, $a
	inc
	mov $a, $b
	exrw 1
	tst 0x01
	jz .cal_lo
	j .cal_hi_ovf
.cal_lo:
	clr $a
.cal_lo_inner:
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	dec
	jnz .cal_lo_inner
	mov $b, $a
	inc
	mov $a, $b
	exrw 1
	tst 0x01
	jnz .cal_done
	j .cal_lo

.cal_done:
	mov $b, $a
	slr
	slr
	ldi $b, 0
	ideref

	ldi $d, 0
.tick:
	mov $d, $a
	out
	jal delay_250ms
	jal delay_250ms
	jal delay_250ms
	jal delay_250ms
	jal beep_check
	mov $d, $a
	inc
	mov $a, $d
	j .tick

delay_250ms:
	clr $a
	deref
	mov $a, $b
.d_outer:
	clr $a
.d_inner:
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	dec
	jnz .d_inner
	mov $b, $a
	dec
	mov $a, $b
	jnz .d_outer
	ret

beep_check:
	clr $a
	exw 0 3			; DDRA = 0 (PA0 input, PA1 input — safe)
	ldi $a, 1
	deref
	dec
	ldi $b, 1
	ideref
	jnz .no_beep
	jal beep
	ldi $a, 10
	ldi $b, 1
	ideref
.no_beep:
	ret

beep:
	ldi $a, 0x02
	exw 0 3			; DDRA = 0x02 (PA1 output for beep)
	ldi $c, 0
.beep_loop:
	ldi $a, 0x02
	exw 0 1			; PA1 HIGH
	clr $a
	exw 0 1			; PA1 LOW
	mov $c, $a
	dec
	mov $a, $c
	jnz .beep_loop
	clr $a
	exw 0 3			; DDRA = 0 (restore all input)
	ret

__i2c_sb:
	mov $a, $b
	ldi $a, 8
	mov $a, $c
.isb:
	mov $b, $a
	tst 0x80
	jnz .isbh
	ldi $a, 0x03
	exw 0 2
	ldi $a, 0x01
	exw 0 2
	ldi $a, 0x03
	exw 0 2
	j .isbn
.isbh:
	ldi $a, 0x02
	exw 0 2
	clr $a
	exw 0 2
	ldi $a, 0x02
	exw 0 2
.isbn:
	mov $b, $a
	sll
	mov $a, $b
	mov $c, $a
	dec
	mov $a, $c
	jnz .isb
	ldi $a, 0x02
	exw 0 2
	nop
	nop
	nop
	nop
	nop
	clr $a
	exw 0 2
	nop
	nop
	nop
	nop
	nop
	exrw 0
	push $a
	ldi $a, 0x02
	exw 0 2
	pop $a
	ret
__i2c_sp:
	ldi $a, 0x03
	exw 0 2
	ldi $a, 0x01
	exw 0 2
	clr $a
	exw 0 2
	ret

	section data
	byte 0
	byte 10
