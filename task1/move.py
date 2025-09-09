import socket
import struct
import time

CMD_HOST = "127.0.0.1"
CMD_PORT = 5555
TEL_HOST = "127.0.0.1"
TEL_PORT = 5600
PROTO = "tcp"

sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_cmd(v: float, w: float):
    packet = struct.pack("<2f", v, w)
    sock_cmd.sendto(packet, (CMD_HOST, CMD_PORT))

for i in range(130):
    send_cmd(0,0.3)
    time.sleep(0.1)

send_cmd(0,0)
