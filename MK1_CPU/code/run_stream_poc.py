"""Stream-PoC runner v2 — clean serial protocol."""
import sys, time, json, serial

PORT = '/dev/cu.usbmodem3C8427C2E7C82'

def drain_until_json(ser, timeout=10.0):
    """Drain until we hit a JSON line, return it parsed (or None)."""
    end = time.time() + timeout
    buf = b''
    while time.time() < end:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            for line in buf.split(b'\n'):
                line = line.strip()
                if line.startswith(b'{') and line.endswith(b'}'):
                    try:
                        return json.loads(line.decode())
                    except: pass
        else:
            time.sleep(0.05)
    return None

with open('/tmp/stream_poc.asm') as f:
    asm = f.read()

ser = serial.Serial(PORT, 115200, timeout=5)
time.sleep(0.5)
ser.reset_input_buffer()
ser.write(b'RESET\n')
print('reset:', drain_until_json(ser, 3.0))

esc = asm.replace('\n', '\\n')
payload = f'ASM:{esc}\n'.encode()
for i in range(0, len(payload), 4096):
    ser.write(payload[i:i + 4096])
    time.sleep(0.05)
print('asm:', drain_until_json(ser, 3.0))

# Dump ee64 RAM BEFORE upload
ser.reset_input_buffer()
ser.write(b'DUMP_EE64:0,32\n')
hl = b''
end = time.time() + 3.0
while time.time() < end:
    if ser.in_waiting:
        b = ser.read(1)
        if b == b'\n': break
        hl += b
    else: time.sleep(0.05)
print('ee64 pre-upload:', hl.decode(errors='replace').strip())

ser.write(b'UPLOAD\n')
print('upload:', drain_until_json(ser, 15.0))
time.sleep(1.5)

ser.reset_input_buffer()
ser.write(b'DUMP_EE64:0,32\n')
hex_line = b''
end = time.time() + 5.0
while time.time() < end:
    if ser.in_waiting:
        b = ser.read(1)
        if b == b'\n':
            break
        hex_line += b
    else:
        time.sleep(0.05)
print('ee64 RAM:', hex_line.decode(errors='replace').strip())

ser.reset_input_buffer()
ser.write(b'READ_CHIP:0,32\n')
hex_line = b''
end = time.time() + 5.0
while time.time() < end:
    if ser.in_waiting:
        b = ser.read(1)
        if b == b'\n':
            break
        hex_line += b
    else:
        time.sleep(0.05)
print('chip:', hex_line.decode(errors='replace').strip())
drain_until_json(ser, 2.0)

ser.reset_input_buffer()
# READ_CHIP overwrote uploadBuf; re-upload our program before running.
esc = asm.replace('\n', '\\n')
payload = f'ASM:{esc}\n'.encode()
for i in range(0, len(payload), 4096):
    ser.write(payload[i:i + 4096])
    time.sleep(0.05)
print('asm2:', drain_until_json(ser, 3.0))
ser.write(b'UPLOAD\n')
print('upload2:', drain_until_json(ser, 15.0))
time.sleep(1.5)

# Verify chip after re-upload
ser.reset_input_buffer()
ser.write(b'READ_CHIP:0,32\n')
hl = b''
end = time.time() + 5.0
while time.time() < end:
    if ser.in_waiting:
        b = ser.read(1)
        if b == b'\n': break
        hl += b
    else: time.sleep(0.05)
print('chip after upload2:', hl.decode(errors='replace').strip())
drain_until_json(ser, 2.0)

# Re-upload AGAIN to see if multiple uploads stabilize the chip
for i in range(0, len(payload), 4096):
    ser.write(payload[i:i + 4096])
    time.sleep(0.05)
print('asm3:', drain_until_json(ser, 3.0))
ser.write(b'UPLOAD\n')
print('upload3:', drain_until_json(ser, 15.0))
time.sleep(1.5)

ser.reset_input_buffer()
all_bytes = []
for off in range(0, 256, 64):
    ser.reset_input_buffer()
    ser.write(f'DUMP:{off},64\n'.encode())
    hl = b''
    end = time.time() + 3.0
    while time.time() < end:
        if ser.in_waiting:
            b = ser.read(1)
            if b == b'\n': break
            hl += b
        else: time.sleep(0.05)
    line = hl.decode(errors='replace').strip()
    all_bytes.extend(line.split())
print('all 256B:')
for off in range(0, 256, 16):
    print(f'  {off:02X}:', ' '.join(all_bytes[off:off+16]))
ser.write(b'DUMP:128,64\n')
hex_line = b''
end = time.time() + 5.0
while time.time() < end:
    if ser.in_waiting:
        b = ser.read(1)
        if b == b'\n': break
        hex_line += b
    else:
        time.sleep(0.05)
print('code[0..128]:', hex_line.decode(errors='replace').strip())

ser.reset_input_buffer()
ser.write(b'RUNHZ:200000,145000,16\n')
result = drain_until_json(ser, 30.0)
print('runhz:', result)
# Drain rest
import time as _t
_t.sleep(2.0)
extra = b''
while ser.in_waiting:
    extra += ser.read(ser.in_waiting)
print('runhz extra:', extra.decode(errors='replace')[:500])
ser.close()
