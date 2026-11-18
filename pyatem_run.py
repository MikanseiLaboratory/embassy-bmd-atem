import sys, time
sys.path.insert(0, '/tmp/pyatem')
from pyatem.transport import UsbProtocol, ConnectionReady

p = UsbProtocol()
print("device found, rev-probe starting", flush=True)
p.connect()
print("connect() done rev=%s" % p.rev, flush=True)
p.mark_next_connected = True

end = time.time() + 20
n = 0
while time.time() < end:
    try:
        pkt = p.receive_packet()
    except Exception as e:
        print("exc:", e, flush=True)
        break
    if isinstance(pkt, ConnectionReady):
        print("CONNECTED", flush=True)
        continue
    if pkt is None:
        print("NONE(disconnect)", flush=True)
        break
    n += 1
    if n <= 5:
        print("PKT %d bytes" % len(pkt.data), flush=True)
print("total packets:", n, flush=True)
