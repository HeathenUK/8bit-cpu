; Stopwatch: same-loop calibration (zero systematic error)
; The calibration runs the EXACT same inner loop as the delay.
; D overflows in 1 SQW cycle = 1 second. D/4 overflows = 250ms.

	ldi $d, 0
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

	; Calibrate: run delay inner loop, check SQW between overflows
.cal:
	ldi $b, 0		; B = overflow count
	; HIGH phase
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
	; overflow — check SQW
	mov $b, $a
	inc
	mov $a, $b
	exrw 1
	tst 0x01
	jz .cal_lo		; SQW fell — switch to LOW phase
	j .cal_hi_ovf
	; LOW phase
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
	jnz .cal_done		; SQW rose — full cycle done
	j .cal_lo

.cal_done:
	; B = D = overflows per second using the EXACT delay loop
	; D/4 = overflows per 250ms
	mov $b, $a
	slr
	slr
	push $a			; stack: D/4

	; Stopwatch
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

; delay_250ms: stack[2] overflows of 256 × 13-cycle loop
delay_250ms:
	ldsp 2
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
