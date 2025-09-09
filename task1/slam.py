MAP_SIZE_PIXELS         = 1000
MAP_SIZE_METERS         = 10

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from roboviz import MapVisualizer
from threading import Thread

import socket
import struct
import time
import math

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


def tel_loop():
    global last_tel
    while True:
        last_tel = list(recv_tel())

Thread(target=tel_loop).start()

send_cmd(0,0)

last_tel = None
while last_tel is None:
    time.sleep(0.1)

if __name__ == '__main__':

    # Connect to Lidar unit

    laser_model =  Laser(360, 1, 90, 7000, 0, 0)
    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(laser_model, MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # Create an iterator to collect scan data from the RPLidar

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None

    # First scan is crap, so ignore it


    received_coords = None
    prev_recieved_coords = None
    prev_time = time.time()
    current_time = time.time()
    
    while True:
        prev_time = current_time
        current_time = time.time()
        # sock_tel.close()
        # sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # sock_tel.bind((TEL_HOST, TEL_PORT))
        # sock_tel.listen(1)
        # conn, _ = sock_tel.accept()
        # sock_tel = conn

        tel = last_tel.copy()
        distances = list(tel[3])

        angles = []
        for i in range(360):
            angles.append(i)

        i = 0
        while i < len(distances):
            if distances[i] >= 8:
                distances.pop(i)
                angles.pop(i)
            else:
                i+=1

        prev_recieved_coords = received_coords
        received_coords = tel[0:3]
        if(prev_recieved_coords == None): prev_recieved_coords = received_coords

        d_x = received_coords[0]-prev_recieved_coords[0]
        d_y = received_coords[1]-prev_recieved_coords[1]
        d_theta = received_coords[2]-prev_recieved_coords[2]

        theta = prev_recieved_coords[2]
        d_pos = d_x*math.cos(theta)+d_y*math.sin(theta)
        d_t = current_time-prev_time

        odometry = (d_pos*1000,d_theta*57.295779513,d_t)
        print(odometry)


        for i in range(len(distances)):
            distances[i] = distances[i]*1000


        # Update SLAM with current Lidar scan and scan angles if adequate

        slam.update(distances, pose_change=odometry, scan_angles_degrees=angles)

        # Get current robot positionUDP
        x, y, theta = slam.getpos()
        print(x,y)

        # Get current map bytes as grayscale
        slam.getmap(mapbytes)

        # Display map and robot pose, exiting gracefully if user closes it
        if not viz.display(x/1000., y/1000., theta, mapbytes):
            exit(0)
