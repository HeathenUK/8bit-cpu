#!/usr/bin/env python3
"""EEPROM stress test v3 — proven patterns only."""

import requests, time, sys

BASE = "http://mk1.local"

I2C_SUBS = """
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
__i2c_rb:
	ldi $d, 0
	ldi $c, 8
.rb:
	ldi $a, 0x02
	exw 0 2
	nop
	nop
	clr $a
	exw 0 2
	nop
	nop
	nop
	exrw 0
	push $a
	mov $d, $a
	sll
	mov $a, $d
	pop $a
	andi 0x01, $a
	or $d, $a
	mov $a, $d
	ldi $a, 0x02
	exw 0 2
	mov $c, $a
	dec
	mov $a, $c
	jnz .rb
	ret
"""

VIA_INIT = """
	nop
	nop
	nop
	clr $a
	exw 0 0
	exw 0 2
	ldi $a, 0x03
	exw 0 2
	ldi $a, 0x01
	exw 0 2
	clr $a
	exw 0 2
"""

BUS_RECOVERY_ASM = """
	nop
	nop
	nop
	clr $a
	exw 0 0
	exw 0 2
	ldi $c, 9
.busrec:
	ldi $a, 0x02
	exw 0 2
	clr $a
	exw 0 2
	mov $c, $a
	dec
	mov $a, $c
	jnz .busrec
	ldi $a, 0x03
	exw 0 2
	ldi $a, 0x01
	exw 0 2
	clr $a
	exw 0 2
	out_imm 1
	hlt
""" + I2C_SUBS

def assemble_upload_run(asm, max_cycles=5000000):
    r = requests.post(f"{BASE}/assemble", data=asm, headers={"Content-Type": "text/plain"})
    d = r.json()
    if d.get("errors"):
        print(f"  ASM ERRORS: {d['errors']}")
        return None
    if d["code_size"] > 256:
        print(f"  CODE TOO BIG: {d['code_size']}B")
        return None
    requests.post(f"{BASE}/upload")
    r = requests.post(f"{BASE}/run_cycles", params={"n": max_cycles, "us": 1})
    # Wait for any EEPROM write cycle triggered by STOP (10ms max + big margin)
    time.sleep(0.15)
    r2 = requests.get(f"{BASE}/read_output")
    d2 = r2.json()
    if not d2.get("captured"):
        return None
    return d2["value"]

def make_write_asm(addr_hi, addr_lo, value):
    """Write byte. ACK poll after to confirm write cycle complete."""
    return VIA_INIT + f"""
	exrw 2
	ldi $a, 0x01
	exw 0 2
	ldi $a, 0x03
	exw 0 2
	ldi $a, 0xAE
	jal __i2c_sb
	tst 0x01
	jnz .wfail
	ldi $a, {addr_hi}
	jal __i2c_sb
	ldi $a, {addr_lo}
	jal __i2c_sb
	ldi $a, {value}
	jal __i2c_sb
	jal __i2c_sp
	out_imm 1
	hlt
.wfail:
	jal __i2c_sp
	out_imm 222
	hlt
""" + I2C_SUBS

def make_page_write_asm(addr_hi, addr_lo, values):
    data_lines = ""
    for v in values:
        data_lines += f"\tldi $a, {v}\n\tjal __i2c_sb\n"
    return VIA_INIT + f"""
	exrw 2
	ldi $a, 0x01
	exw 0 2
	ldi $a, 0x03
	exw 0 2
	ldi $a, 0xAE
	jal __i2c_sb
	tst 0x01
	jnz .pwfail
	ldi $a, {addr_hi}
	jal __i2c_sb
	ldi $a, {addr_lo}
	jal __i2c_sb
{data_lines}	jal __i2c_sp
	out_imm {len(values)}
	hlt
.pwfail:
	jal __i2c_sp
	out_imm 222
	hlt
""" + I2C_SUBS

def make_random_read_asm(addr_hi, addr_lo):
    return VIA_INIT + f"""
	exrw 2
	ldi $a, 0x01
	exw 0 2
	ldi $a, 0x03
	exw 0 2
	ldi $a, 0xAE
	jal __i2c_sb
	tst 0x01
	jnz .rfail
	ldi $a, {addr_hi}
	jal __i2c_sb
	ldi $a, {addr_lo}
	jal __i2c_sb
	jal __i2c_sp
	; Read
	exrw 2
	ldi $a, 0x01
	exw 0 2
	ldi $a, 0x03
	exw 0 2
	ldi $a, 0xAF
	jal __i2c_sb
	tst 0x01
	jnz .rfail
	jal __i2c_rb
	ldi $a, 0x02
	exw 0 2
	clr $a
	exw 0 2
	nop
	nop
	ldi $a, 0x02
	exw 0 2
	jal __i2c_sp
	mov $d, $a
	out
	hlt
.rfail:
	jal __i2c_sp
	out_imm 222
	hlt
""" + I2C_SUBS

def make_current_read_asm():
    return VIA_INIT + """
	exrw 2
	ldi $a, 0x01
	exw 0 2
	ldi $a, 0x03
	exw 0 2
	ldi $a, 0xAF
	jal __i2c_sb
	jal __i2c_rb
	ldi $a, 0x02
	exw 0 2
	clr $a
	exw 0 2
	nop
	nop
	ldi $a, 0x02
	exw 0 2
	jal __i2c_sp
	mov $d, $a
	out
	hlt
""" + I2C_SUBS

# ── Test suite ──

passes = 0
fails = 0

print("=== EEPROM Stress Test v3 ===\n")
print("Running bus recovery...")
assemble_upload_run(BUS_RECOVERY_ASM)
time.sleep(0.5)  # let any write cycle from recovery complete
print()

# Test 1: Byte write/read
print("--- Test 1: Single byte write/read ---")
test_cases = [
    (0x00, 0x50, 0x00),
    (0x00, 0x51, 0xFF),
    (0x00, 0x52, 0xAA),
    (0x00, 0x53, 0x55),
    (0x00, 0x54, 0x01),
    (0x00, 0x55, 0x80),
    (0x00, 0x56, 0x7F),
    (0x00, 0x57, 42),
    (0x01, 0x00, 99),
    (0x02, 0x00, 200),
]

for hi, lo, val in test_cases:
    w = assemble_upload_run(make_write_asm(hi, lo, val))
    ok_w = (w == 1)
    if not ok_w:
        addr = (hi << 8) | lo
        print(f"  [FAIL] addr=0x{addr:04X} val={val} (write OI={w})")
        fails += 1
        continue
    r = assemble_upload_run(make_random_read_asm(hi, lo))
    addr = (hi << 8) | lo
    if r == val:
        print(f"  [PASS] addr=0x{addr:04X} val={val}")
        passes += 1
    else:
        print(f"  [FAIL] addr=0x{addr:04X} wrote {val}, read {r}")
        fails += 1

# Test 2: Re-read persistence
print("\n--- Test 2: Re-read persistence ---")
for hi, lo, val in test_cases:
    r = assemble_upload_run(make_random_read_asm(hi, lo))
    addr = (hi << 8) | lo
    if r == val:
        passes += 1
        print(f"  [PASS] addr=0x{addr:04X} val={val}")
    else:
        fails += 1
        print(f"  [FAIL] addr=0x{addr:04X} expected={val} got={r}")

# Test 3: Page write + sequential read
print("\n--- Test 3: Page write + sequential read ---")
for hi, lo, vals in [(0x00, 0x60, [11,22,33,44]), (0x00, 0x80, [0,128,255,1])]:
    w = assemble_upload_run(make_page_write_asm(hi, lo, vals))
    if w == 222 or w is None:
        print(f"  [FAIL] addr=0x{(hi<<8)|lo:04X} page write failed (OI={w})")
        fails += 1
        continue
    # Bus recovery after page write (clears any stuck state)
    assemble_upload_run(BUS_RECOVERY_ASM)
    time.sleep(1.0)
    # Read each byte individually via random read
    results = []
    for i in range(len(vals)):
        addr = lo + i
        addr_hi_r = hi + (addr >> 8)
        addr_lo_r = addr & 0xFF
        r = assemble_upload_run(make_random_read_asm(addr_hi_r, addr_lo_r))
        results.append(r)
    if results == vals:
        passes += 1
        print(f"  [PASS] addr=0x{(hi<<8)|lo:04X} {vals}")
    else:
        fails += 1
        print(f"  [FAIL] addr=0x{(hi<<8)|lo:04X} wrote {vals}, read {results}")

# Test 4: Overwrite
print("\n--- Test 4: Overwrite test ---")
for val in [0, 127, 255, 42, 0]:
    w = assemble_upload_run(make_write_asm(0x00, 0x70, val))
    if w != 1:
        print(f"  [FAIL] overwrite with {val}: write OI={w}")
        fails += 1
        continue
    time.sleep(0.02)
    r = assemble_upload_run(make_random_read_asm(0x00, 0x70))
    if r == val:
        passes += 1
        print(f"  [PASS] overwrite with {val}")
    else:
        fails += 1
        print(f"  [FAIL] overwrite with {val}, read {r}")

# Test 5: Repeated reads
print("\n--- Test 5: Repeated read consistency (10x) ---")
all_ok = True
for i in range(10):
    r = assemble_upload_run(make_random_read_asm(0x00, 0x50))
    expected = 0x00
    if r != expected:
        print(f"  [FAIL] read {i+1}: expected {expected}, got {r}")
        all_ok = False
        fails += 1
if all_ok:
    print(f"  [PASS] 10/10 consistent")
    passes += 1

print(f"\n=== Results: {passes} passed, {fails} failed ===")
sys.exit(1 if fails > 0 else 0)
