; Stream-execute PoC — fetch+exec EE64 bytes one instruction at a time.
;
; Features (verified on hardware):
;   - Linear 2-byte instructions
;   - Unconditional jump (`j target`) — close I²C, re-seek
;   - Indirect call to kernel (`jal kernel_addr`)
;   - `ret` (0x6C) — terminate
;
; Conditional branches (jz / jnz / jc / jnc) are handled by the
; full streamer in __stream_full below; this base version is the
; minimum kernel for verifying the architecture works end-to-end.
;
; Slot at code[252..254]:  [opcode, operand, ret(0x6C)] — jal_r 252.
; State (page 3): vpc_lo at page3[0xE0].

VPC_LO_P3   = 0xE0

	section code
_main:
	ldi $b,0xFF
	mov $b,$sp
	clr $a
	exw 0 0
	ddrb_imm 0x00
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x00
	clr $a
	jal __stream
	hlt

__stream:
	ldi $b,VPC_LO_P3
	iderefp3              ; page3[VPC_LO_P3] = vpc_lo
	; Pre-seed slot[254] = 0x6C (ret)
	ldi $b,254
	ldi $a,0x6C
	istc_inc

.outer:
	exrw 2
	ddrb_imm 0x01
	ddrb_imm 0x03
	ldi $a,0xA0
	jal __i2c_sb
	clr $a
	jal __i2c_sb
	ldi $a,VPC_LO_P3
	derefp3
	jal __i2c_sb
	ddrb_imm 0x00
	ddrb_imm 0x01
	ddrb_imm 0x03
	ldi $a,0xA1
	jal __i2c_sb

.fetch:
	jal __i2c_rb
	mov $b,$a
	cmpi 0x6C
	jz .do_ret
	cmpi 0x3D
	jz .do_jmp
	cmpi 0xAC
	jz .do_jal
	j .do_normal

.do_normal:
	jal __ack
	ldi $b,252
	istc_inc
	push_b
	jal __i2c_rb
	mov $b,$a
	pop_b
	istc_inc
	jal __ack
	jal __vpc_add2        ; vpc += 2 (track natural chip-pointer advance)
	ldi $a,252
	jal_r
	mov $a,$a ;!keep
	j .fetch

.do_jmp:
	jal __ack
	jal __i2c_rb
	mov $b,$a
	ldi $b,VPC_LO_P3
	iderefp3              ; vpc = target
	jal __i2c_stop
	j .outer

.do_jal:
	jal __ack
	jal __i2c_rb
	mov $b,$a             ; A = kernel target
	push_b                ; save target
	jal __vpc_add2        ; vpc += 2 (past the jal so .outer re-seeks correctly)
	jal __i2c_stop
	pop_b
	mov $b,$a
	jal_r
	mov $a,$a ;!keep
	j .outer

.do_ret:
	jal __i2c_stop
	ret

__vpc_add2:
	ldi $a,VPC_LO_P3
	derefp3
	addi 2,$a
	ldi $b,VPC_LO_P3
	iderefp3
	ret

__ack:
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x03
	ddrb_imm 0x02
	ret

__i2c_stop:
	ddrb_imm 0x00
	ddrb_imm 0x02
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x00
	ret

__i2c_sb:
	mov $a,$d
	ldi $b,8
.isb_lp:
	mov $d,$a
	tst 0x80
	jnz .isbh
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x03
	j .isbn
.isbh:
	ddrb_imm 0x02
	ddrb_imm 0x00
	ddrb_imm 0x02
.isbn:
	sll
	mov $a,$d
	decb
	jnz .isb_lp
	ddrb_imm 0x02
	ddrb_imm 0x00
	exrw 0
	ddrb_imm 0x02
	ret

__i2c_rb:
	ldi $b,0
	ldi $d,8
.rb_lp:
	sllb
	ddrb_imm 0x00
	exrw 0
	exrw 0
	tst 0x01
	jz .rz
	incb
.rz:
	ddrb_imm 0x02
	decd
	jnz .rb_lp
	mov $b,$d
	ret

_emit_kc:
	out_imm 0xCA
	ret

	section ee64
	org 200
_streamed:
	out_imm 0xAA
	out_imm 0xBB
	jal _emit_kc
	out_imm 0xCC
	j 0x10
	out_imm 0x99
	out_imm 0xDE
	out_imm 0xDF
	out_imm 0xEE
	byte 0x6C
