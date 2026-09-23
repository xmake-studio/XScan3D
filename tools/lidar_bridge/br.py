import serial, struct, time
HDR = b'\x55\xAA\x02\x08'
class Br:
    def __init__(s, port):  # e.g. 'COM4' or '/dev/ttyACM0'
        s.s = serial.Serial(port, 115200, timeout=0.05); s.s.dtr = True
        time.sleep(0.2); s.s.reset_input_buffer()
    def cmd(s, c, a=0, wait=0.3):
        s.s.write(b'\xFE\xED\xC0' + c.encode() + struct.pack('<I', a)); return s.read(wait)
    def send(s, data): s.s.write(data)
    def read(s, t):
        end = time.time() + t; out = bytearray()
        while time.time() < end: out += s.s.read(4096)
        return bytes(out)
def frames(buf):
    out = []; i = 0
    while True:
        j = buf.find(HDR, i)
        if j < 0 or j + 28 > len(buf): break
        out.append(buf[j:j+28]); i = j + 28
    return out
def other(buf):
    """bytes not belonging to 28-byte frames"""
    res = bytearray(); i = 0
    while i < len(buf):
        if buf[i:i+4] == HDR and i + 28 <= len(buf): i += 28; continue
        res.append(buf[i]); i += 1
    return bytes(res)
def edgelog(b, pin, ms):
    """Returns [(t_us, level)] of every edge on `pin` over `ms` milliseconds."""
    import re
    b.s.write(b'\xFE\xED\xC0L' + struct.pack('<I', (pin << 16) | ms))
    d = b.read(ms / 1000 + 1.0)
    m = re.search(rb'\n#EL (\d+)\n', d)
    n = int(m.group(1)); raw = d[m.end():m.end() + 5 * n]
    return [struct.unpack_from('<IB', raw, 5 * i) for i in range(n)]
