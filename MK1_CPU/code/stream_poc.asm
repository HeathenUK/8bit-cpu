; Stream-execute PoC — fetch+exec EE64 bytes one instruction at a time.
;
; Layout:
;   code[0..]  : _main, __stream, __i2c_sb, __i2c_rb
;   code[250..252] : exec_buf (opcode, operand, ret=0x6C)
;   ee64[0..6] : streamed function (out_imm 0xAA; out_imm 0xBB; out_imm 0xCC; sentinel 0x77)
;
; Expected output: 0xAA, 0xBB, 0xCC then halt.

	section code
_main:
	ldi $b,0xFF
	mov $b,$sp
	; VIA init (i2c idle bus)
	clr $a
	exw 0 0
	ddrb_imm 0x00
	; Bus recovery: SCL low, SDA low, then idle
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x00
	; Stream EE64 starting at addr 0
	jal __stream
	hlt

; ── __stream: opcode-by-opcode streamer ─────────────────────────────
; Reads bytes from EE64[0..] in sequential-read mode, writes each
; pair (opcode, operand) into exec_buf at code[250..251], then jal_r's
; into code[250]. exec_buf[252] is set to 0x6C (ret) once at entry so
; the executed instruction returns control here.
;
; Sentinel: opcode 0x77 ends the stream (closes I²C, returns).
; Assumes all streamed instructions are 2 bytes long (PoC scope).
__stream:
	out_imm 0xE0       ; debug: streamer entered
	; Pre-load exec_buf[2] = 0x6C (ret) — istc_inc writes to code page
	ldi $b,252
	ldi $a,0x6C
	istc_inc           ; code[252] = 0x6C, B becomes 253 (don't care)
	out_imm 0xE1       ; debug: exec_buf set
	; I²C random-read setup at EE64 addr 0x0000
	exrw 2
	ddrb_imm 0x01
	ddrb_imm 0x03
	ldi $a,0xA0
	jal __i2c_sb
	clr $a
	jal __i2c_sb
	clr $a
	jal __i2c_sb
	; REP-START + SLA+R (now in sequential read)
	ddrb_imm 0x00
	ddrb_imm 0x01
	ddrb_imm 0x03
	ldi $a,0xA1
	jal __i2c_sb
	out_imm 0xE2       ; debug: i2c read mode entered
.fetch:
	; Read opcode byte (returned in $b after __i2c_rb)
	jal __i2c_rb
	mov $b,$a
	cmpi 0x77
	jz .done
	; ACK to chip so next read advances the byte pointer
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x03
	ddrb_imm 0x02
	; Stash opcode in exec_buf[0]
	ldi $b,250
	istc_inc            ; code[250]=opcode, B=251
	; Read operand byte
	push_b
	jal __i2c_rb
	mov $b,$a
	pop_b
	istc_inc            ; code[251]=operand, B=252
	; ACK before next opcode read
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x03
	ddrb_imm 0x02
	; Execute exec_buf
	ldi $a,250
	jal_r
	mov $a,$a ;!keep
	j .fetch
.done:
	; STOP i²c, then spin (hlt unreliable in RUNHZ post-out)
	ddrb_imm 0x00
	ddrb_imm 0x02
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x00
	out_imm 0xEF       ; debug: stream done
.spin:
	j .spin

; ── __i2c_sb: send byte (mirrors compiler's __i2c_sb) ───────────────
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

; ── __i2c_rb: read byte (returns in B) ──────────────────────────────
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

	section ee64
	org 200
_streamed:
	out_imm 0xAA
	out_imm 0xBB
	out_imm 0xCC
	byte 0x77
