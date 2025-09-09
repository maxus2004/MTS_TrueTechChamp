import socket
import struct
import time

CMD_HOST = "127.0.0.1"
CMD_PORT = 5555
TEL_HOST = "127.0.0.1"
TEL_PORT = 5600
PROTO = "tcp"

sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

if PROTO == "udp":
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
else:
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
    sock_tel.listen(1)
    print(f"[client] waiting for telemetry TCP on {TEL_HOST}:{TEL_PORT}...")
    conn, _ = sock_tel.accept()
    sock_tel = conn
    print("[client] connected to udp_diff telemetry")

def send_cmd(v: float, w: float):
    packet = struct.pack("<2f", v, w)
    sock_cmd.sendto(packet, (CMD_HOST, CMD_PORT))

def recv_tel():
    if PROTO == "udp":
        data, _ = sock_tel.recvfrom(65535)
    else:
        size_bytes = sock_tel.recv(4)
        if not size_bytes:
            return None
        size = struct.unpack("<I", size_bytes)[0]
        data = sock_tel.recv(size)

    if not data.startswith(b"WBT2"):
        return None

    header_size = 4 + 6 * 4
    odom_x, odom_y, odom_th, vx, vy, vth = struct.unpack("<6f", data[4:header_size])
    n = struct.unpack("<I", data[header_size:header_size+4])[0]
    ranges = []
    if n > 0:
        ranges = struct.unpack(f"<{n}f", data[header_size+4:header_size+4+4*n])
    return odom_x, odom_y, odom_th, ranges



print(len(recv_tel()[3]))
exit()

while True:
    print(recv_tel())
    send_cmd(0.1, 0)
