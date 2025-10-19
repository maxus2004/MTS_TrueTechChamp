# -*- coding: utf-8 -*-
"""
Merged optimized udp_diff controller — combines legacy telemetry + odometry + race timing
with new RGB-D streaming (chunked UDP / TCP) and non-blocking command listener.
Environment-controlled features: TELEMETRY_PROTO/HOST/PORT, CMD_LISTEN_PORT,
DEPTH_ENABLE/DEPTH_SEND_EVERY/DEPTH_DOWNSAMPLE, RGB_ENABLE/RGB_SEND_EVERY/RGB_DOWNSAMPLE,
CHUNKING/MAX_DGRAM, SPEEDUP, RESULTS_FILE, STOP_ON_FINISH, etc.
"""
import os, sys, time, math, struct, socket, select, itertools
from controller import Supervisor

# ---- env helpers ----
E = os.getenv
def _i(k,d):
    try: return int(E(k, str(d)))
    except: return d
def _f(k,d):
    try: return float(E(k, str(d)))
    except: return d

# ---- constants / env ----
CMD_LISTEN_PORT = _i('CMD_LISTEN_PORT', 5555)
TELEMETRY_HOST  = E('TELEMETRY_HOST', '127.0.0.1')
TELEMETRY_PORT  = _i('TELEMETRY_PORT', 5600)
TELEMETRY_PROTO = E('TELEMETRY_PROTO', 'udp').lower()
TIME_STEP_MS = _i('TIME_STEP_MS', 32)
SPEEDUP = _i('SPEEDUP', 2)

# motion limits
BASE_MAX_LINEAR = 0.5
BASE_MAX_ANGULAR = 1.0
BASE_MAX_LINEAR_ACC = 0.05
BASE_MAX_ANGULAR_ACC = 0.2
MAX_LINEAR = BASE_MAX_LINEAR * SPEEDUP
MAX_ANGULAR = BASE_MAX_ANGULAR * SPEEDUP
MAX_LINEAR_ACC = BASE_MAX_LINEAR_ACC * SPEEDUP
MAX_ANGULAR_ACC = BASE_MAX_ANGULAR_ACC * SPEEDUP

WHEEL_BASE = float(E('WHEEL_BASE', '0.25'))
WHEEL_RADIUS = float(E('WHEEL_RADIUS', '0.035'))
MAX_VEL = float(E('MAX_VEL', '12.0'))

# timing/arena / results
ARENA = _f('ARENA', 8.0)
FINISH_X = _f('FINISH_X', 0.7)
FINISH_Y = _f('FINISH_Y', 1.1)
START_CMD_V_THRESH = _f('START_CMD_V_THRESH', 0.01)
START_CMD_W_THRESH = _f('START_CMD_W_THRESH', 0.01)
START_MOVE_DIST = _f('START_MOVE_DIST', 0.02)
STOP_ON_FINISH = E('STOP_ON_FINISH', 'pause').lower()
RESULTS_FILE = E('RESULTS_FILE', os.path.join(os.getcwd(), 'results.csv'))

# RGBD / chunking
DEPTH_ENABLE = _i('DEPTH_ENABLE', 1) == 1
DEPTH_SEND_EVERY = max(1, _i('DEPTH_SEND_EVERY', 3))
DEPTH_DOWNSAMPLE = max(1, _i('DEPTH_DOWNSAMPLE', 2))
RGB_ENABLE = _i('RGB_ENABLE', 0) == 1
RGB_SEND_EVERY = max(1, _i('RGB_SEND_EVERY', 6))
RGB_DOWNSAMPLE = max(1, _i('RGB_DOWNSAMPLE', 2))
CHUNKING = _i('CHUNKING', 1) == 1
MAX_DGRAM = max(512, _i('MAX_DGRAM', 1200))

# wire formats
DEPTH_MAGIC=b'WBTD'; DEPTH_HDR='<HHi f f'
RGB_MAGIC=b'WBTR';   RGB_HDR  ='<HHi'
CHNK_MAGIC=b'CHNK';  CHNK_HDR ='<I I H H'   # msg_id, total_len, idx, count
LEGACY_MAGIC=b'WBTG'  # legacy telemetry header

class UdpDiffController:
    def __init__(self):
        self.robot = Supervisor()
        ts = getattr(self.robot, 'getBasicTimeStep', lambda: TIME_STEP_MS)()
        self.ts = int(ts) if ts else TIME_STEP_MS

        # devices and motion
        self._pick_motors()
        self._enable_encoders()
        self.lidar = self._setup_lidar()
        self.gps = self._setup_gps()
        self.gyro = self._setup_gyro()

        # rgb/depth
        self.rgb = None
        self.depth = None
        self._setup_cameras()

        # odometry
        self.last_lf = self.last_lr = self.last_rf = self.last_rr = None
        self.odom_x = self.odom_y = self.odom_th = 0.0

        # motion smoothing
        self.current_v = 0.0; self.current_w = 0.0

        # race state
        self.started = False; self.finished = False
        self.init_gps = None; self.start_pos = None
        self.start_time = None; self.finish_time = None

        # network
        self.cmd_sock = self._setup_cmd_socket()
        self.tx = None
        self._setup_telemetry_socket()

        # counters
        self.step_ctr = 0
        self.msg_id = itertools.count(1)

        print(f"[udp_diff] READY proto={TELEMETRY_PROTO} dst={TELEMETRY_HOST}:{TELEMETRY_PORT} cmd@0.0.0.0:{CMD_LISTEN_PORT}"
              , flush=True)

    # --- device helpers ---
    def _pick_motors(self):
        self.LF = self._pick_motor('left_front_motor')
        self.LR = self._pick_motor('left_rear_motor')
        self.RF = self._pick_motor('right_front_motor')
        self.RR = self._pick_motor('right_rear_motor')

    def _pick_motor(self, name):
        try:
            d = self.robot.getDevice(name)
            d.setPosition(float('inf'))
            d.setVelocity(0.0)
            return d
        except Exception:
            return None

    def _enable_encoders(self):
        self.lf_enc = self._enable_sensor('left_front_encoder')
        self.lr_enc = self._enable_sensor('left_rear_encoder')
        self.rf_enc = self._enable_sensor('right_front_encoder')
        self.rr_enc = self._enable_sensor('right_rear_encoder')

    def _enable_sensor(self, name):
        try:
            d = self.robot.getDevice(name)
            d.enable(self.ts)
            return d
        except Exception:
            return None

    def _setup_lidar(self):
        try:
            lidar = self.robot.getDevice('lidar')
            lidar.enable(self.ts)
            try: lidar.enablePointCloud(True)
            except: pass
            return lidar
        except Exception:
            return None

    def _setup_gps(self):
        try:
            gps = self.robot.getDevice('gps'); gps.enable(self.ts); return gps
        except Exception:
            return None

    def _setup_gyro(self):
        try:
            gyro = self.robot.getDevice('gyro'); gyro.enable(self.ts); return gyro
        except Exception:
            return None

    def _setup_cameras(self):
        try:
            rgb = self.robot.getDevice('rgb')
            if rgb: rgb.enable(self.ts); self.rgb = rgb
        except Exception:
            self.rgb = None
        try:
            depth = self.robot.getDevice('depth')
            if depth: depth.enable(self.ts); self.depth = depth
        except Exception:
            self.depth = None

    # --- networking ---
    def _setup_cmd_socket(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try: s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except: pass
            s.bind(("0.0.0.0", CMD_LISTEN_PORT))
            s.setblocking(False)
            return s
        except Exception as e:
            print(f"[udp_diff] cmd socket failed: {e}", file=sys.stderr)
            return None

    def _setup_telemetry_socket(self):
        if TELEMETRY_PROTO == 'udp':
            self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            return
        # tcp connect with retry
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2.0); s.connect((TELEMETRY_HOST, TELEMETRY_PORT)); s.settimeout(None)
                self.tx = s; return
            except Exception as e:
                try: s.close()
                except: pass
                print(f"[udp_diff] TCP connect retry: {e}", file=sys.stderr)
                time.sleep(0.5)

    # --- odometry from encoders ---
    def update_encoder_odometry(self):
        a_lf = self.lf_enc.getValue() if self.lf_enc else None
        a_lr = self.lr_enc.getValue() if self.lr_enc else None
        a_rf = self.rf_enc.getValue() if self.rf_enc else None
        a_rr = self.rr_enc.getValue() if self.rr_enc else None
        if None in (a_lf, a_lr, a_rf, a_rr):
            return
        if self.last_lf is None:
            self.last_lf, self.last_lr, self.last_rf, self.last_rr = a_lf, a_lr, a_rf, a_rr
            return
        d_lf = (a_lf - self.last_lf) * WHEEL_RADIUS
        d_lr = (a_lr - self.last_lr) * WHEEL_RADIUS
        d_rf = (a_rf - self.last_rf) * WHEEL_RADIUS
        d_rr = (a_rr - self.last_rr) * WHEEL_RADIUS
        self.last_lf, self.last_lr, self.last_rf, self.last_rr = a_lf, a_lr, a_rf, a_rr
        d_left = 0.5 * (d_lf + d_lr); d_right = 0.5 * (d_rf + d_rr)
        ds = 0.5 * (d_left + d_right); dth = (d_right - d_left) / WHEEL_BASE
        th_mid = self.odom_th + 0.5 * dth
        self.odom_x += ds * math.cos(th_mid)
        self.odom_y += ds * math.sin(th_mid)
        self.odom_th += dth

    # --- motion control (diff drive with accel limits) ---
    def diff_drive(self, v, w):
        v = max(-MAX_LINEAR, min(MAX_LINEAR, v)); w = max(-MAX_ANGULAR, min(MAX_ANGULAR, w))
        dv = v - self.current_v; dw = w - self.current_w
        max_dv = MAX_LINEAR_ACC * (self.ts / 1000.0); max_dw = MAX_ANGULAR_ACC * (self.ts / 1000.0)
        if abs(dv) > max_dv: v = self.current_v + max_dv * (1 if dv > 0 else -1)
        if abs(dw) > max_dw: w = self.current_w + max_dw * (1 if dw > 0 else -1)
        self.current_v, self.current_w = v, w
        v_l = v - (w * WHEEL_BASE / 2); v_r = v + (w * WHEEL_BASE / 2)
        k = MAX_VEL / max(1.0, abs(v_l), abs(v_r)); vl, vr = v_l * k, v_r * k
        for m in (self.LF, self.LR):
            if m: m.setVelocity(vl)
        for m in (self.RF, self.RR):
            if m: m.setVelocity(vr)

    # --- packing rgb/depth ---
    def _pack_depth(self):
        w = self.depth.getWidth(); h = self.depth.getHeight(); ds = DEPTH_DOWNSAMPLE
        dw = (w + ds - 1)//ds; dh = (h + ds - 1)//ds
        buf = self.depth.getRangeImage()
        minR = self.depth.getMinRange() if hasattr(self.depth,'getMinRange') else 0.0
        maxR = self.depth.getMaxRange() if hasattr(self.depth,'getMaxRange') else 0.0
        out = bytearray(); out += DEPTH_MAGIC
        out += struct.pack(DEPTH_HDR, dw, dh, ds, float(minR), float(maxR))
        for v in range(0, h, ds):
            base = v * w
            for u in range(0, w, ds):
                z = buf[base + u]
                if not (z > 0.0) or math.isinf(z) or z != z:
                    mm = 0
                else:
                    mm = int(min(65535, round(z * 1000.0)))
                out += struct.pack('<H', mm)
        return bytes(out)

    def _pack_rgb(self):
        w = self.rgb.getWidth(); h = self.rgb.getHeight(); ds = RGB_DOWNSAMPLE
        dw = (w + ds - 1)//ds; dh = (h + ds - 1)//ds
        raw = self.rgb.getImage()   # BGRA
        out = bytearray(); out += RGB_MAGIC; out += struct.pack(RGB_HDR, dw, dh, ds)
        stride = w * 4
        for v in range(0, h, ds):
            row = v * stride
            for u in range(0, w, ds):
                i = row + u * 4
                out += raw[i:i+4]
        return bytes(out)

    # --- sending with chunking / tcp handling ---
    def _send_raw(self, payload: bytes):
        if TELEMETRY_PROTO == 'udp':
            try:
                if (not CHUNKING) or len(payload) <= MAX_DGRAM:
                    self.tx.sendto(payload, (TELEMETRY_HOST, TELEMETRY_PORT))
                    return
                # chunk
                msg_id = next(self.msg_id)
                hdr_len = 4 + struct.calcsize(CHNK_HDR)
                max_payload = max(1, MAX_DGRAM - hdr_len)
                count = (len(payload) + max_payload - 1) // max_payload
                total = len(payload)
                for idx in range(count):
                    part = payload[idx*max_payload:(idx+1)*max_payload]
                    chunk = bytearray(); chunk += CHNK_MAGIC
                    chunk += struct.pack(CHNK_HDR, msg_id, total, idx, count)
                    chunk += part
                    self.tx.sendto(chunk, (TELEMETRY_HOST, TELEMETRY_PORT))
            except Exception as e:
                sys.stderr.write(f"[udp_diff] send failed: {e}\n"); sys.stderr.flush()
        else:
            # TCP: send with 4-byte length prefix, reconnect on error
            try:
                self.tx.sendall(struct.pack('<I', len(payload)) + payload)
            except Exception:
                try: self.tx.close()
                except: pass
                self._setup_telemetry_socket()

    # --- legacy telemetry packet (odom, speeds, gyro, lidar) ---
    def _pack_legacy_telemetry(self, ranges, wx, wy, wz):
        n = len(ranges)
        # pack: LEGACY_MAGIC + 9 floats + uint32 n + n floats
        payload = bytearray(); payload += LEGACY_MAGIC
        payload += struct.pack('<9f', self.odom_x, self.odom_y, self.odom_th, self.current_v, 0.0, self.current_w, wx, wy, wz)
        payload += struct.pack('<I', n)
        if n:
            payload += struct.pack(f'<{n}f', *ranges)
        return bytes(payload)

    # --- command poll ---
    def _poll_cmd(self):
        if not self.cmd_sock: return None
        try:
            r,_,_ = select.select([self.cmd_sock], [], [], 0.0)
            if not r: return None
            data, addr = self.cmd_sock.recvfrom(2048)
            if not data: return None
            # simple: if exactly 8 bytes -> two floats v,w
            if len(data) >= 8:
                try:
                    v,w = struct.unpack('<2f', data[:8])
                    return (v,w)
                except Exception:
                    pass
            # ping/pong example
            if data == b'ping':
                if TELEMETRY_PROTO == 'udp':
                    self.tx.sendto(b'pong', (TELEMETRY_HOST, TELEMETRY_PORT))
                else:
                    self.tx.sendall(struct.pack('<I',4)+b'pong')
        except Exception:
            pass
        return None

    # --- results csv ---
    def _next_attempt(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                n = sum(1 for _ in f) - 1
                return max(1, n + 1)
        except Exception:
            return 1

    def _write_result_csv(self, elapsed, start_t, finish_t, sx, sy, fx, fy, status=None):
        path = RESULTS_FILE; header = 'timestamp_iso,attempt,elapsed_s,start_x,start_y,finish_x,finish_y,start_t,finish_t,status\n'
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        if not os.path.exists(path):
            with open(path, 'w', encoding='utf-8', newline='') as f: f.write(header)
        attempt = self._next_attempt(path)
        ts = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(finish_t))
        with open(path, 'a', encoding='utf-8', newline='') as f:
            f.write(f"{ts},{attempt},{elapsed:.3f},{sx:.3f},{sy:.3f},{fx:.3f},{fy:.3f},{start_t:.3f},{finish_t:.3f},{status}\n")
        print(f"[udp_diff] saved result attempt={attempt} elapsed={elapsed:.3f}s status={status} → {path}")

    # --- main loop ---
    def run(self):
        linear_x = angular_z = 0.0
        last_log = 0.0
        try:
            while self.robot.step(self.ts) != -1:
                self.step_ctr += 1
                # commands
                cmd = self._poll_cmd()
                if cmd is not None:
                    linear_x, angular_z = cmd

                # drive and odom
                self.diff_drive(linear_x, angular_z)
                self.update_encoder_odometry()

                # sensors
                gx = gy = None
                if self.gps:
                    try:
                        px, py, pz = self.gps.getValues(); gx, gy = float(px), float(py)
                        if self.init_gps is None: self.init_gps = (gx, gy)
                    except Exception:
                        pass

                wx = wy = wz = 0.0
                if self.gyro:
                    try: wx, wy, wz = self.gyro.getValues()
                    except Exception: pass

                # start/finish logic
                now = time.time()
                if not self.started:
                    moved_cmd = (abs(linear_x) > START_CMD_V_THRESH) or (abs(angular_z) > START_CMD_W_THRESH)
                    moved_gps = False
                    if gx is not None and self.init_gps is not None:
                        moved_gps = math.hypot(gx - self.init_gps[0], gy - self.init_gps[1]) >= START_MOVE_DIST
                    if moved_cmd or moved_gps:
                        self.started = True; self.start_time = now
                        if gx is not None:
                            self.start_pos = (gx, gy); print(f"[udp_diff] timing START t={self.start_time:.3f} pos x={gx:.3f} y={gy:.3f}")
                        else:
                            print(f"[udp_diff] timing START t={self.start_time:.3f}")

                if self.started and not self.finished and gx is not None:
                    if abs(gx) <= FINISH_X and abs(gy) <= FINISH_Y:
                        self.finished = True; self.finish_time = now; self.finish_pos = (gx, gy)
                        elapsed = self.finish_time - self.start_time
                        sx, sy = self.start_pos if self.start_pos is not None else (gx, gy)
                        self._write_result_csv(elapsed, self.start_time, self.finish_time, sx, sy, gx, gy, status='finish')
                        print(f"[udp_diff] timing FINISH t={self.finish_time:.3f} elapsed={elapsed:.3f} pos x={gx:.3f} y={gy:.3f}")
                        # stop motors
                        for m in (self.LF, self.RF, self.LR, self.RR):
                            if m: m.setVelocity(0.0)
                        try:
                            if STOP_ON_FINISH == 'quit': self.robot.simulationQuit(0)
                            elif STOP_ON_FINISH == 'pause': self.robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
                        except Exception:
                            pass
                        break

                # lidar ranges
                ranges = []
                if self.lidar:
                    try:
                        vals = self.lidar.getRangeImage()
                        for r in vals:
                            if r != r or r == float('inf') or r <= 0.0: ranges.append(8.0)
                            else: ranges.append(float(r))
                    except Exception:
                        ranges = []

                # legacy telemetry (send every step)
                payload = self._pack_legacy_telemetry(ranges, wx, wy, wz)
                self._send_raw(payload)

                # rgb/depth sends on schedules
                if self.depth and DEPTH_ENABLE and (self.step_ctr % DEPTH_SEND_EVERY == 0):
                    try: self._send_raw(self._pack_depth())
                    except Exception as e: sys.stderr.write(f"[udp_diff] depth pack/send error: {e}\n")
                if self.rgb and RGB_ENABLE and (self.step_ctr % RGB_SEND_EVERY == 0):
                    try: self._send_raw(self._pack_rgb())
                    except Exception as e: sys.stderr.write(f"[udp_diff] rgb pack/send error: {e}\n")

                # compact log
                if time.time() - last_log > 1.0:
                    if ranges:
                        n = len(ranges); front = ranges[n//2]; right = ranges[n//4]; left = ranges[3*n//4]
                        lidar_info = f"n={n},front={front:.2f},left={left:.2f},right={right:.2f}"
                    else:
                        lidar_info = 'n=0'
                    print(f"[udp_diff] ODOM(x={self.odom_x:.3f},y={self.odom_y:.3f},θ={math.degrees(self.odom_th):.1f}°) "
                          f"CMD(v={linear_x:.2f},w={angular_z:.2f}) LIDAR({lidar_info}) Gyro: wx={wx:.3f},wy={wy:.3f},wz={wz:.3f}"
                          , flush=True)
                    last_log = time.time()

        except KeyboardInterrupt:
            print('[udp_diff] KeyboardInterrupt: shutting down...')
        finally:
            try: self.cmd_sock.close()
            except: pass
            try: self.tx.close()
            except: pass
            print('[udp_diff] terminated')


if __name__ == '__main__':
    c = UdpDiffController(); c.run()
