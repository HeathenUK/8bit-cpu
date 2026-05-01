; Stream-execute PoC v2 — full feature set ready for compiler integration.
;
; Features (verified on hardware):
;   - Linear 2-byte instructions: fetch from EE64, execute via 3-byte slot
;   - Unconditional jump (`j target`): close I²C, re-open at target offset
;   - `jal kernel_target`: indirect call to a code-page address (kernel
;     helper or user function); streamer suspends I²C, runs the call,
;     reopens I²C at vpc+2 after return
;   - `ret` (0x6C): terminates streamed function, returns to streamer caller
;
; Constraints (compiler-side requirements when integrating):
;   - Streamed bodies use ONLY 2-byte instruction units. 1-byte instructions
;     must be padded with NOP (mov $a,$a, 0x00) at compile time. Trades
;     +EE64 size for streamer simplicity.
;   - Branch / call targets in EE64 are 8-bit offsets within the streamed
;     body (relative to body base). Hi byte of EE64 addr is fixed at 0
;     for this PoC; full 64K addressing is straightforward extension.
;   - Conditional branches (jz/jnz/jc/jnc): NOT implemented in this PoC.
;     Design sketched in WORKLOG: requires immediately-after-slot-exec
;     dispatch via page-3 handler lookup table to preserve flags through
;     to the branch test. ~30 B additional streamer code, ~256 B page-3
;     lookup table. Targets oled_temp/oled_test would need this.
;
; Slot: code[252..254] = 3 bytes (opcode, operand, ret=0x6C).
; State (page 3): vpc_lo at page3[0xE0].
;
; Entry: $a = ee64 offset lo of streamed body (assumes hi=0).
; Exit: ret returns to caller (e.g., after streaming a synth_main_body).

VPC_LO_P3   = 0xE0       ; page3 addr of vpc cache

	section code
_main:
	ldi $b,0xFF
	mov $b,$sp
	; VIA init
	clr $a
	exw 0 0
	ddrb_imm 0x00
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x00
	; Stream the function at EE64 offset 0
	clr $a
	jal __stream
	hlt

; ── __stream: byte-by-byte EE64 fetcher + executor ──────────────────
;
; Maintains a 3-byte execution buffer at code[252..254]:
;   slot[252] = opcode, slot[253] = operand, slot[254] = 0x6C (ret).
; Each fetched 2-byte instruction is materialised into slot[252..253],
; jal_r'd into, executed, and ret returns to the streamer's post-jal_r
; filler. The slot[254]=ret is set once at entry.
;
; vpc (current EE64 read offset) is cached at page3[VPC_LO_P3] so it
; survives across I²C re-seeks (when handling `j` and `jal`).

__stream:
	out_imm 0xE0          ; debug
	; Save initial vpc (in $a) to page3
	ldi $b,VPC_LO_P3
	iderefp3              ; page3[E0] = vpc_lo
	out_imm 0xE1          ; debug

	; Pre-seed slot[254] = 0x6C (ret) — istc_inc writes to code page
	ldi $b,254
	ldi $a,0x6C
	istc_inc

.outer_loop:
	; (Re-)open I²C random read at page3[VPC_LO_P3]
	exrw 2
	ddrb_imm 0x01
	ddrb_imm 0x03
	ldi $a,0xA0
	jal __i2c_sb
	clr $a                ; addr_hi (0 for PoC; 8-bit offset within body)
	jal __i2c_sb
	ldi $a,VPC_LO_P3
	derefp3               ; A = vpc_lo
	jal __i2c_sb
	; REP-START + SLA+R
	ddrb_imm 0x00
	ddrb_imm 0x01
	ddrb_imm 0x03
	ldi $a,0xA1
	jal __i2c_sb

.fetch:
	; Read opcode byte (returned in $b after __i2c_rb)
	jal __i2c_rb
	mov $b,$a             ; A = opcode

	; ── Dispatch on opcode ─────────────────────────────────────
	cmpi 0x6C
	jz .do_ret
	cmpi 0x3D
	jz .do_jmp
	cmpi 0xAC
	jz .do_jal
	; Default: regular 2-byte instruction → fetch operand, write slot, exec
	j .do_normal

; ── Regular 2-byte instruction ───────────────────────────────────────
.do_normal:
	; ACK previous read
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x03
	ddrb_imm 0x02
	; Stash opcode in slot[252]
	ldi $b,252
	istc_inc              ; code[252] = opcode, B = 253

	; Read operand (with B preserved across __i2c_rb via push/pop)
	push_b
	jal __i2c_rb
	mov $b,$a
	pop_b
	istc_inc              ; code[253] = operand, B = 254

	; ACK so the chip's pointer advances for the next opcode read
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x03
	ddrb_imm 0x02

	; Advance vpc by 2
	ldi $a,VPC_LO_P3
	derefp3
	addi 2,$a
	ldi $b,VPC_LO_P3
	iderefp3

	; jal_r into slot, return via slot[254]=ret
	ldi $a,252
	jal_r
	mov $a,$a ;!keep      ; jal_r off-by-one filler
	j .fetch

; ── Unconditional jump (`j target` opcode 0x3D) ──────────────────────
.do_jmp:
	; ACK + read target byte (= new vpc)
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x03
	ddrb_imm 0x02
	jal __i2c_rb
	mov $b,$a             ; A = target
	; vpc = target
	ldi $b,VPC_LO_P3
	iderefp3
	; Close I²C (STOP) and re-open at new vpc
	ddrb_imm 0x00
	ddrb_imm 0x02
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x00
	j .outer_loop

; ── jal kernel_target — indirect call to code-page function ──────────
;
; Streamed `jal X` means "call function at code-page address X". We:
;   1. Read target byte (1 ACK + 1 read)
;   2. Advance vpc by 2 (past the jal)
;   3. Close I²C transaction
;   4. jal_r into the kernel target (uses code-0 jal_r since target is
;      a regular code address)
;   5. After return, jump back to .outer_loop which re-opens I²C at
;      vpc and continues streaming.
;
; Caveat: the called kernel function must be flag-clean by convention
; (most kernel helpers already are; cold-tier dispatchers __cold_call
; etc. obviously are). It also must not modify page3[VPC_LO_P3] which
; is reserved for the streamer's state.
.do_jal:
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x03
	ddrb_imm 0x02
	jal __i2c_rb
	mov $b,$a             ; A = target (kernel addr)
	; Save target in B-temp slot via push_b (cheap)
	push_b
	; Advance vpc by 2 (past the jal instruction)
	ldi $a,VPC_LO_P3
	derefp3
	addi 2,$a
	ldi $b,VPC_LO_P3
	iderefp3
	; Close I²C
	ddrb_imm 0x00
	ddrb_imm 0x02
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x00
	; Restore target into A and call indirectly
	pop_b                 ; B = target
	mov $b,$a             ; A = target
	jal_r
	mov $a,$a ;!keep      ; off-by-one filler
	; Kernel call returned; reopen I²C at vpc and continue
	j .outer_loop

; ── ret (opcode 0x6C) — terminate streamed function ──────────────────
.do_ret:
	; Close I²C
	ddrb_imm 0x00
	ddrb_imm 0x02
	ddrb_imm 0x03
	ddrb_imm 0x01
	ddrb_imm 0x00
	ret

; ── I²C primitives (mirrored from compiler kernel) ───────────────────
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

; ── A simple kernel function the streamed body can call ─────────────
;
; Streamed function exercises this by emitting `jal _emit_kc` (where
; _emit_kc's address is the operand byte). Outputs 0xCA when called.
_emit_kc:
	out_imm 0xCA
	ret

; ── Streamed function in EE64 ───────────────────────────────────────
;
; Layout (8-bit offsets within EE64 image):
;   0x00: out_imm 0xAA           D1 AA
;   0x02: out_imm 0xBB           D1 BB
;   0x04: jal _emit_kc           AC <_emit_kc-addr>
;   0x06: out_imm 0xCC           D1 CC
;   0x08: j 0x10                 3D 10           (jump forward to skip 0x0A..0x0F)
;   0x0A: out_imm 0xDD           D1 DD           (skipped — should NOT emit)
;   0x0C: out_imm 0xDE           D1 DE           (skipped)
;   0x0E: out_imm 0xDF           D1 DF           (skipped)
;   0x10: out_imm 0xEE           D1 EE
;   0x12: ret                    6C
;
; Expected output: AA, BB, CA (kernel call), CC, EE.
;   (DD/DE/DF must not appear — they're after `j 0x10`.)

	section ee64
	org 200
_streamed:
	out_imm 0xAA          ; offset 0x00
	out_imm 0xBB          ; 0x02
	jal _emit_kc          ; 0x04 — operand = code-page address of _emit_kc
	out_imm 0xCC          ; 0x06
	j 0x10                ; 0x08 — unconditional forward jump
	out_imm 0xDD          ; 0x0A — skipped
	out_imm 0xDE          ; 0x0C — skipped
	out_imm 0xDF          ; 0x0E — skipped
	; offset 0x10:
	out_imm 0xEE          ; 0x10
	byte 0x6C             ; 0x12 — ret (1 byte; streamer sees as terminator)
