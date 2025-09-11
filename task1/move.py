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

while True:
    c = input()
    match c:
        case "w":
            send_cmd(0.5,0)
        case "s":
            send_cmd(-0.5,0)
        case "a":
            send_cmd(0,0.5)
        case "d":
            send_cmd(0,-0.5)
        case _:
            send_cmd(0,0)