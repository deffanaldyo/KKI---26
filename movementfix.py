from pymavlink import mavutil
import time

class Movement:
    DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
    SERVO_CHANNEL = 9
    PWM_OPEN = 1900
    PWM_CLOSE = 1100

    def __init__(self, connection_string=None, port_list=None, baud=115200, heartbeat_timeout=5):
        self.armed = False
        self.last_yaw = 0
        self.yaw_offset = None
        self.depth_offset = None
        self._gripper_state = None
        self._gripper_ready = False

        if connection_string:
            self._connect(connection_string, baud, heartbeat_timeout)
        else:
            if port_list is None:
                port_list = self.DEFAULT_PORTS
            self.master = None
            for port in port_list:
                try:
                    print(f"[Movement] Menghubungkan ke {port} ...")
                    master = mavutil.mavlink_connection(port, baud=baud)
                    master.wait_heartbeat(timeout=heartbeat_timeout)
                    print(f"[Movement] Heartbeat diterima di {port} "
                          f"(sistem {master.target_system}, komponen {master.target_component})")
                    self.master = master
                    break
                except Exception as exc:
                    print(f"[Movement] Gagal di {port}: {exc}")

            if self.master is None:
                raise ConnectionError(
                    "Tidak bisa connect ke Pixhawk! Coba colok ulang USB atau cek port di /dev/ttyACM*"
                )

    def _connect(self, connection_string, baud, heartbeat_timeout):
        print(f"[Movement] Menghubungkan ke {connection_string} ...")
        if connection_string.startswith(("udp", "tcp")):
            self.master = mavutil.mavlink_connection(connection_string)
        elif baud:
            self.master = mavutil.mavlink_connection(connection_string, baud=baud)
        else:
            self.master = mavutil.mavlink_connection(connection_string)

        self.master.wait_heartbeat(timeout=heartbeat_timeout)
        print(f"[Movement] Heartbeat diterima (sistem {self.master.target_system}, "
              f"komponen {self.master.target_component})")

    def _result_name(self, result):
        try:
            return mavutil.mavlink.enums['MAV_RESULT'][result].name
        except Exception:
            return str(result)

    def _drain_statustext(self, duration=1.0):
        start = time.time()
        found = []
        while time.time() - start < duration:
            msg = self.master.recv_match(type='STATUSTEXT', blocking=True, timeout=0.2)
            if msg is None:
                continue
            text = msg.text.strip() if isinstance(msg.text, str) else msg.text.decode(errors='ignore').strip()
            print(f"[FC STATUSTEXT] {text}")
            found.append(text)
        return found

    def set_mode(self, mode_name='ALT_HOLD', retries=3, ack_timeout=3, confirm_timeout=5):
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f"[Movement] Mode '{mode_name}' tidak dikenal!")
            return False

        for attempt in range(1, retries + 1):
            print(f"[Movement] Set mode '{mode_name}' percobaan ke-{attempt}...")

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
                0, 0, 0, 0, 0
            )

            ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=ack_timeout)
            if not (ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE
                    and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED):
                if ack:
                    print(f"[Movement] Set mode ditolak FC: {self._result_name(ack.result)}")
                else:
                    print(f"[Movement] Tidak ada ACK untuk set mode (timeout).")
                self._drain_statustext(duration=1.0)
                time.sleep(0.5)
                continue

            start = time.time()
            while time.time() - start < confirm_timeout:
                msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg and msg.custom_mode == mode_id:
                    print(f"[Movement] Mode berhasil diset ke {mode_name}")
                    return True

            print(f"[Movement] ACK diterima tapi HEARTBEAT belum konfirmasi mode, retry...")

        print(f"[Movement] Gagal set mode ke {mode_name} setelah {retries} percobaan.")
        return False

    def arm(self, retries=3, timeout=5):
        for attempt in range(1, retries + 1):
            print(f"[Movement] Percobaan arming ke-{attempt}...")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )

            ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                    and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.armed = True
                print("[Movement] ROV berhasil ARMED")
                return True

            if ack:
                print(f"[Movement] Arming ditolak FC: {self._result_name(ack.result)}")
            else:
                print("[Movement] Arming gagal: tidak ada ACK (timeout).")
            # Pesan STATUSTEXT biasanya berisi alasan pre-arm check gagal,
            # mis. "PreArm: Compass not calibrated" atau "PreArm: RC not calibrated"
            self._drain_statustext(duration=1.0)
            time.sleep(1)

        print("[Movement] Arming gagal setelah beberapa percobaan.")
        return False

    def disarm(self, timeout=5):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.armed = False
            print("[Movement] ROV berhasil DISARMED")
            return True
        print("[Movement] Disarm gagal / tidak ada ACK")
        return False

    def manual_control(self, vx=0, vy=0, vz=500, yaw=0, buttons=0):
        self.master.mav.manual_control_send(
            self.master.target_system,
            vx, vy, vz, yaw, buttons
        )

    def request_data_stream(self, rate_hz=10):
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz,
            1  # start
        )

    def get_local_position(self, timeout=0.2):
        """Baca posisi lokal x, y (meter) dari LOCAL_POSITION_NED.
        Butuh sumber posisi seperti DVL/vision positioning aktif di ArduSub;
        kalau tidak ada, akan return (None, None)."""
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
        if msg is None:
            return None, None
        return msg.x, msg.y

    def _print_telemetry_line(self, x, y, yaw, depth_cm):
        x_str = f"{x:+.2f}" if x is not None else "N/A"
        y_str = f"{y:+.2f}" if y is not None else "N/A"
        yaw_str = f"{yaw:+.1f}" if yaw is not None else "N/A"
        depth_str = f"{depth_cm:+.1f}" if depth_cm is not None else "N/A"
        print(f"\r[Telemetry] x={x_str} m | y={y_str} m | yaw={yaw_str}° | depth={depth_str} cm     ",
              end="", flush=True)

    def monitor(self, duration=10, interval=0.2):
        """Tampilkan x, y, yaw, depth secara realtime di terminal selama `duration` detik,
        tanpa menggerakkan ROV. Cocok dipakai sebelum/di antara gerakan untuk memantau."""
        print(f"[Movement] Monitoring realtime selama {duration}s (Ctrl+C untuk berhenti lebih awal)...")
        start = time.time()
        try:
            while time.time() - start < duration:
                x, y = self.get_local_position(timeout=0.1)
                yaw = self.get_yaw(timeout=0.1)
                depth_m = self.get_depth(timeout=0.1)
                depth_cm = depth_m * 100 if depth_m is not None else None
                self._print_telemetry_line(x, y, yaw, depth_cm)
                time.sleep(interval)
        except KeyboardInterrupt:
            pass
        print()  # newline setelah selesai

    def _lock_depth_reference(self, retries=15, timeout=0.5):
        """Tunggu sampai dapat pembacaan tekanan asli, lalu jadikan itu referensi depth=0.
        Dipanggil sekali di awal (sebelum ROV bergerak) agar depth pasti 0 saat program mulai."""
        for attempt in range(1, retries + 1):
            msg = self.master.recv_match(type='SCALED_PRESSURE2', blocking=True, timeout=timeout)
            if msg is not None:
                self.depth_offset = msg.press_abs
                print(f"[Movement] Depth awal dijadikan referensi 0 cm (tekanan asli: {self.depth_offset:.2f} hPa)")
                return True
        print("[Movement] PERINGATAN: gagal dapat referensi depth awal, akan diset otomatis "
              "pada pembacaan pertama yang berhasil nanti.")
        return False

    def get_depth(self, timeout=1):
        """Return depth relatif (meter) terhadap tekanan referensi kalibrasi.
        0 = posisi saat program mulai / kalibrasi terakhir, bertambah positif saat turun ke bawah."""
        msg = self.master.recv_match(type='SCALED_PRESSURE2', blocking=True, timeout=timeout)
        if msg is None:
            return None

        pressure_hpa = msg.press_abs
        density = 1025  # kg/m3 air laut, ganti 997 kalau air tawar
        g = 9.80665

        if self.depth_offset is None:
            self.depth_offset = pressure_hpa
            print(f"[Movement] Depth awal dijadikan referensi 0 cm (tekanan asli: {self.depth_offset:.2f} hPa)")

        pressure_pa = (pressure_hpa - self.depth_offset) * 100
        depth = pressure_pa / (density * g)
        return depth

    def go_to_depth(self, target_depth_cm, tolerance_cm=5, kp=8, max_rate=150,
                     timeout=30, hold_confirm_time=1.0):

        target_m = target_depth_cm / 100.0
        print(f"[Movement] Menuju target depth {target_depth_cm} cm ...")

        start_time = time.time()
        stable_since = None

        while time.time() - start_time < timeout:
            current_depth = self.get_depth()
            if current_depth is None:
                print("[Movement] Gagal baca depth, retry...")
                time.sleep(0.1)
                continue

            error_m = target_m - current_depth
            error_cm = error_m * 100

            if abs(error_cm) <= tolerance_cm:
                self.manual_control(vx=0, vy=0, vz=500, yaw=0)
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since >= hold_confirm_time:
                    print(f"[Movement] Target depth tercapai: {current_depth*100:.1f} cm")
                    return True
            else:
                stable_since = None
                offset = kp * error_cm
                offset = max(-max_rate, min(max_rate, offset))
                z_cmd = int(500 + offset)
                z_cmd = max(0, min(1000, z_cmd))

                self.manual_control(vx=0, vy=0, vz=z_cmd, yaw=0)
                print(f"[Movement] Depth: {current_depth*100:.1f} cm | "
                      f"error: {error_cm:.1f} cm | vz_cmd: {z_cmd}")

            time.sleep(0.1)

        print("[Movement] Timeout, target depth tidak tercapai.")
        self.manual_control(vx=0, vy=0, vz=500, yaw=0)
        return False

    def _lock_yaw_reference(self, retries=15, timeout=0.5):
        """Tunggu sampai dapat pembacaan ATTITUDE asli, lalu jadikan itu referensi 0°.
        Dipanggil sekali di awal (sebelum ROV bergerak) agar yaw pasti 0 saat program mulai."""
        for attempt in range(1, retries + 1):
            msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=timeout)
            if msg is not None:
                raw_yaw_deg = (msg.yaw * 180.0 / 3.14159265) % 360
                self.yaw_offset = raw_yaw_deg
                self.last_yaw = 0.0
                # print(f"[Movement] Yaw awal dijadikan referensi 0° (heading asli: {self.yaw_offset:.1f}°)")
                return True
        #     print(f"[Movement] Menunggu data ATTITUDE untuk referensi yaw... ({attempt}/{retries})")

        # print("[Movement] PERINGATAN: gagal dapat referensi yaw awal, akan diset otomatis "
        #       "pada pembacaan pertama yang berhasil nanti.")
        return False

    def get_yaw(self, timeout=1):
        msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=timeout)

        if msg is None:
            return self.last_yaw  # pakai nilai terakhir

        raw_yaw_deg = (msg.yaw * 180.0 / 3.14159265) % 360

        if self.yaw_offset is None:
            self.yaw_offset = raw_yaw_deg
            print(f"[Movement] Yaw awal dijadikan nol referensi: {self.yaw_offset:.1f}°")

        yaw_deg = (raw_yaw_deg - self.yaw_offset + 180) % 360 - 180
        self.last_yaw = yaw_deg  # update cache yaw relatif

        return yaw_deg

    def _yaw_error(self, target_deg, current_deg):
        error = (target_deg - current_deg + 180) % 360 - 180
        return error

    def start(self):
        self._setup_gripper()
        if not self.set_mode('ALT_HOLD'):
            raise RuntimeError(
                "Gagal set mode ALT_HOLD. Cek pesan [FC STATUSTEXT] di atas untuk alasannya."
            )
        if not self.arm():
            raise RuntimeError(
                "Gagal arm ROV. Cek pesan [FC STATUSTEXT] di atas — biasanya karena "
                "pre-arm check gagal (kompas belum kalibrasi, EKF/GPS belum siap, "
                "RC belum dikalibrasi, atau safety switch belum ditekan). "
                "Bisa juga sementara nonaktifkan pre-arm check dengan set param "
                "ARMING_CHECK=0 di QGroundControl untuk isolasi masalah (jangan pakai saat operasi nyata)."
            )
        
        self.request_data_stream(10)
        time.sleep(0.5)  # beri waktu FC mulai mengirim stream ATTITUDE & SCALED_PRESSURE2
        self._lock_yaw_reference()
        self._lock_depth_reference()
        print("[Movement] ROV siap.")

    def stop(self):
        self.manual_control(vx=0, vy=0, vz=500, yaw=0)

    def _depth_heave(self, target_depth_cm, kp=8, max_rate=150):
        current_depth = self.get_depth(timeout=0.2)
        if current_depth is None:
            return 500

        error_cm = target_depth_cm - current_depth * 100
        offset = max(-max_rate, min(max_rate, kp * error_cm))
        return max(0, min(1000, int(500 + offset)))

    def _yaw_cmd(self, target_deg, kp_yaw=6, max_yaw_rate=300, deadband=2):
        error = self._yaw_error(target_deg, self.get_yaw())
        if abs(error) < deadband:
            return 0
        return int(max(-max_yaw_rate, min(max_yaw_rate, kp_yaw * error)))

    def rov(self, duration, angle, depth_cm, surge, sway, gripper):
        """Gerak ROV: (waktu, yaw target, depth cm, surge/vx, sway/vy), gripper."""
        print(f"[Movement] ROV: t={duration}s angle={angle}° depth={depth_cm}cm "
              f"surge={surge} sway={sway} gripper={gripper}")
        start = time.time()
        while time.time() - start < duration:
            current_yaw = self.get_yaw(timeout=0.05)
            self.set_gripper(gripper)
            self.manual_control(
                vx=surge,
                vy=sway,
                vz=self._depth_heave(depth_cm),
                yaw=self._yaw_cmd(angle),
            )

            x, y = self.get_local_position(timeout=0.05)
            depth_m = self.get_depth(timeout=0.05)
            depth_now_cm = depth_m * 100 if depth_m is not None else None
            self._print_telemetry_line(x, y, current_yaw, depth_now_cm)

            time.sleep(0.1)

        self.stop()
        print("\n[Movement] ROV selesai.")

    def bai(self, duration, angle, depth_cm, surge, sway, gripper):
        self.rov(duration, angle, depth_cm, surge, sway, gripper)

    def bairotasi(self, duration, angle, depth_cm, surge=0, sway=0, gripper=0):
        self.rov(duration, angle, depth_cm, surge, sway, gripper)

    def cleanup(self):
        self.stop()
        time.sleep(0.2)
        self.disarm()
        self.close()

    def cruise(self, vx=0, vy=0, target_yaw_deg=None, duration=5,
               kp_yaw=6, max_yaw_rate=300, gripper=0):

        if target_yaw_deg is None:
            locked_yaw = self.get_yaw(timeout=2)
            if locked_yaw is None:
                print("[Movement] Gagal baca yaw awal, yaw tidak dikunci (yaw=0).")
            else:
                print(f"[Movement] Yaw auto-lock pada {locked_yaw:.1f}°")
        else:
            locked_yaw = target_yaw_deg
            print(f"[Movement] Yaw dikunci manual ke {locked_yaw:.1f}°")

        print(f"[Movement] Cruise: vx={vx}, vy={vy}, durasi={duration}s")
        start = time.time()

        while time.time() - start < duration:
            r_cmd = 0
            if locked_yaw is not None:
                current_yaw = self.get_yaw(timeout=0.2)
                if current_yaw is not None:
                    error = self._yaw_error(locked_yaw, current_yaw)
                    r_cmd = max(-max_yaw_rate, min(max_yaw_rate, kp_yaw * error))
                    r_cmd = int(r_cmd)

            self.manual_control(vx=vx, vy=vy, vz=500, yaw=r_cmd)
            time.sleep(0.1)

        self.manual_control(vx=0, vy=0, vz=500, yaw=0)
        print("[Movement] Cruise selesai.")

    def _read_param(self, param_name, timeout=3):
        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode(),
            -1
        )
        start = time.time()
        while time.time() - start < timeout:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("utf-8", errors="ignore")
            pid = pid.rstrip("\x00")
            if pid == param_name:
                return msg.param_value
        return None

    def _set_param(self, param_name, value):
        print(f"[Movement] Set {param_name} = {value}")
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode(),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )

    def _setup_gripper(self):
        """Disable fungsi bawaan servo channel agar DO_SET_SERVO bisa kontrol gripper."""
        if self._gripper_ready:
            return True

        param_name = f"SERVO{self.SERVO_CHANNEL}_FUNCTION"
        current = self._read_param(param_name)
        if current == 0:
            print(f"[Movement] {param_name} sudah 0 (siap kontrol gripper).")
            self._gripper_ready = True
            return True

        self._set_param(param_name, 0)
        time.sleep(2)

        result = self._read_param(param_name)
        if result == 0:
            print(f"[Movement] {param_name} = 0 OK")
            self._gripper_ready = True
            return True

        print(f"[Movement] PERINGATAN: gagal set {param_name} ke 0 (nilai: {result})")
        return False

    def set_gripper(self, cmd):
        """cmd: 1=buka, -1=tutup, 0=diam (netral)."""
        if not self._gripper_ready:
            self._setup_gripper()

        if cmd == self._gripper_state:
            return

        pwm_map = {1: self.PWM_OPEN, -1: self.PWM_CLOSE, 0: 1500}
        label_map = {1: "membuka gripper", -1: "menutup gripper", 0: "diam"}
        pwm = pwm_map.get(cmd, 1500)
        if cmd in label_map:
            print(label_map[cmd])

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            self.SERVO_CHANNEL,
            pwm,
            0, 0, 0, 0, 0
        )
        self._gripper_state = cmd

    def gripper(self, cmd):
        self.set_gripper(cmd)
        return cmd
    
    def close(self):
        self.master.close()
        print("[Movement] Koneksi ditutup.")