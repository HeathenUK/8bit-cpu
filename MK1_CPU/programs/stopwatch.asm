; Stopwatch with DDRA init, optimized to 204B
; Removed redundant ldi $d,0 at start (D=0 from reset, re-inited at .tick)

.via_dly:
	dec
	jnz .via_dly
	clr $a
	exw 0 0
	exw 0 2
	exw 0 3			; DDRA = 0 (PA0 input for SQW)
	nop				; align delay to working address

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

	; Sync to rising edge
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

	; Calibrate
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
