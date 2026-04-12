; Write 4 different values to EEPROM addrs 0x0000-0x0003, read all back
; Outputs each byte — expect 0xDE, 0xAD, 0xBE, 0xEF
nop
nop
nop
clr $a
exw 0 0
ddrb_imm 0x00
exw 0 3

; WRITE 4 bytes starting at 0x0000 (page write)
exrw 2
ddrb_imm 0x01
ddrb_imm 0x03
ldi $a, 0xAE
jal __sb
clr $a
jal __sb
clr $a
jal __sb
ldi $a, 0xDE
jal __sb
ldi $a, 0xAD
jal __sb
ldi $a, 0xBE
jal __sb
ldi $a, 0xEF
jal __sb
ddrb_imm 0x03
ddrb_imm 0x01
ddrb_imm 0x00

; Wait 20ms
ldi $d, 20
.ww:
clr $a
.wi:
dec
jnz .wi
mov $d,$a
dec
mov $a,$d
jnz .ww

; SET ADDR back to 0x0000
exrw 2
ddrb_imm 0x01
ddrb_imm 0x03
ldi $a, 0xAE
jal __sb
clr $a
jal __sb
clr $a
jal __sb
ddrb_imm 0x03
ddrb_imm 0x01
ddrb_imm 0x00

; Sequential READ 4 bytes
exrw 2
ddrb_imm 0x01
ddrb_imm 0x03
ldi $a, 0xAF
jal __sb

; Byte 0 — ACK after each to continue sequential read
jal __rb
mov $d,$a
out
; ACK (SDA LOW during SCL pulse)
ddrb_imm 0x03
ddrb_imm 0x01
ddrb_imm 0x03

; Byte 1
jal __rb
mov $d,$a
out
ddrb_imm 0x03
ddrb_imm 0x01
ddrb_imm 0x03

; Byte 2
jal __rb
mov $d,$a
out
ddrb_imm 0x03
ddrb_imm 0x01
ddrb_imm 0x03

; Byte 3 — NACK (no more bytes)
jal __rb
mov $d,$a
out
ddrb_imm 0x00
ddrb_imm 0x02

; STOP
ddrb_imm 0x03
ddrb_imm 0x01
ddrb_imm 0x00
hlt

__sb:
	mov $a,$b
	ldi $a,8
	mov $a,$c
.is:
	mov $b,$a
	tst 0x80
	jnz .ih
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x03
	j .in
.ih:
	ddrb_imm 0x02
	ddrb_imm 0x00
	ddrb_imm 0x02
.in:
	mov $b,$a
	sll
	mov $a,$b
	mov $c,$a
	dec
	mov $a,$c
	jnz .is
	ddrb_imm 0x02
	ddrb_imm 0x00
	exrw 0
	ddrb_imm 0x02
	ret

; Fixed __i2c_rb: B=accum, D=counter
__rb:
	ldi $b,0
	ldi $d,8
.rb:
	mov $b,$a
	sll
	mov $a,$b
	ddrb_imm 0x00
	nop
	exrw 0
	tst 0x01
	jz .rz
	mov $b,$a
	ori 0x01,$a
	mov $a,$b
.rz:
	ddrb_imm 0x02
	mov $d,$a
	dec
	mov $a,$d
	jnz .rb
	mov $b,$d
	ret
