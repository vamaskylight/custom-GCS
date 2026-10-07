"""Run an ArduCopter simulator in WSL and talk MAVLink to it from this PC.

Everything stays on this computer: the simulator listens on a TCP port inside
WSL, and this script connects to it through localhost.

Set up the simulator programs first with setup_sitl.sh (see README.md).
"""
from __future__ import annotations

import math
import pathlib
import subprocess
import time

from pymavlink import mavutil

DISTRO = "Ubuntu-24.04"
SITL_HOME = "~/vama-sitl"

COPTER_MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
                9: "LAND", 16: "POSHOLD", 17: "BRAKE", 21: "SMART_RTL"}

# Simulator parameter names that differ between versions.
GPS_OFF = {"4.6": ("SIM_GPS_DISABLE", 1), "4.7": ("SIM_GPS1_ENABLE", 0)}


def wsl(cmd: str, timeout: float = 60.0) -> str:
    out = subprocess.run(["wsl.exe", "-d", DISTRO, "--exec", "bash", "-lc", cmd],
                         capture_output=True, text=True, timeout=timeout, errors="replace")
    return (out.stdout or "") + (out.stderr or "")


def wsl_path(path: pathlib.Path) -> str:
    """C:\\folder\\file as WSL sees it: /mnt/c/folder/file."""
    full = str(pathlib.Path(path).resolve())
    return f"/mnt/{full[0].lower()}/" + full[3:].replace("\\", "/")


class Sitl:
    """One simulated quadcopter. The simulated clock runs `speedup` times faster than real time."""

    def __init__(self, version: str, scripts: dict[str, pathlib.Path], params: dict[str, float],
                 script_params: dict[str, float] | None = None, script_ready: str = "",
                 speedup: int = 10, home: str = "20.4347,72.8696,30,0", instance: int = 0,
                 sysid: int = 0) -> None:
        """params go into the start-up file. script_params are parameters that a script
        creates, so they are set after the script has said `script_ready`.

        instance: several simulators at once (a fleet). Instance N listens on
        TCP 5760 + 10 N (and 5762 + 10 N for a ground station). Start instance 0
        first: it clears out any simulator still running, and closing it stops
        them all."""
        self.version = version
        self.speedup = speedup
        self.instance = int(instance)
        self.port = 5760 + 10 * self.instance
        self.texts: list[tuple[float, str]] = []   # (simulated seconds, text)
        self.named: dict[str, float] = {}          # last NAMED_VALUE_FLOAT of each name
        self.named_at: dict[str, float] = {}       # and the simulated second it arrived
        self.mode_log: list[tuple[float, str]] = []  # (simulated seconds, new mode)
        self.mode = None
        self.armed = False
        self.sim_s = 0.0                           # simulated seconds since boot
        self.north = self.east = self.alt = 0.0    # metres from home
        self.vn = self.ve = 0.0                    # m/s
        self.lean_deg = 0.0
        self.wp_dist = 0.0
        self.wp_bearing = 0.0
        self.motors: list[int] = []                # PWM of motors 1 to 4
        self.wind_est = None
        self.ekf_flags = 0
        run = f"{SITL_HOME}/run-{version}" + (f"-i{self.instance}" if self.instance else "")
        lines = [] if self.instance else ["(pkill -x arducopter 2>/dev/null; sleep 0.5; true)"]
        lines.append(f"rm -rf {run} && mkdir -p {run}/scripts")
        for name, path in scripts.items():
            lines.append(f"cp '{wsl_path(path)}' {run}/scripts/{name}")
        extra = "\\n".join(f"{k} {v}" for k, v in params.items())
        lines.append(f"printf '{extra}\\n' > {run}/extra.parm")
        shown = wsl(" && ".join(lines)).strip()
        if shown:
            print(shown)
        # The simulator's process id goes to arducopter.pid, so one instance of
        # a fleet can be frozen (a radio silence) or stopped on its own.
        inst = f" -I {self.instance}" if self.instance else ""
        # The system id: SYSID_THISMAV on 4.6, MAV_SYSID on 4.7. --sysid sets either.
        if sysid:
            inst += f" --sysid {int(sysid)}"
        cmd = (f"cd {run} && ({SITL_HOME}/{version}/arducopter --model quad --speedup {speedup} -w{inst} "
               f"--defaults ../{version}/copter.parm,extra.parm --home {home} > sitl.log 2>&1 & "
               f"echo $! > arducopter.pid; wait $!; echo exit=$? >> sitl.log)")
        # stdin stays open on purpose: wsl.exe ends the Linux side when its input closes.
        self.proc = subprocess.Popen(["wsl.exe", "-d", DISTRO, "--exec", "bash", "-lc", cmd],
                                     stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.run_dir = run
        time.sleep(2.0)
        self.mav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{self.port}", source_system=255, retries=20)
        hb = self.mav.wait_heartbeat(timeout=60)
        if hb is None:
            raise RuntimeError("no heartbeat from the simulator")
        self.mav.mav.request_data_stream_send(self.mav.target_system, self.mav.target_component,
                                              mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        if script_ready:
            self.pump(60, until=lambda last: self.saw(script_ready))
            if not self.saw(script_ready):
                raise RuntimeError(f"the script did not start: no '{script_ready}' text. " + self.log_tail(8))
        for name, value in (script_params or {}).items():
            self.set_param(name, value)

    # --- reading ---------------------------------------------------------
    def _take(self, msg) -> None:
        kind = msg.get_type()
        if kind == "STATUSTEXT":
            self.texts.append((self.sim_s, msg.text))
        elif kind == "HEARTBEAT":
            if msg.get_srcComponent() == 1:
                mode = COPTER_MODES.get(msg.custom_mode, str(msg.custom_mode))
                if mode != self.mode:
                    self.mode_log.append((self.sim_s, mode))
                self.mode = mode
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == "NAMED_VALUE_FLOAT":
            name = str(msg.name).rstrip("\x00")
            self.named[name] = float(msg.value)
            self.named_at[name] = self.sim_s
        elif kind == "LOCAL_POSITION_NED":
            self.north, self.east, self.alt = msg.x, msg.y, -msg.z
            self.vn, self.ve = msg.vx, msg.vy
        elif kind == "ATTITUDE":
            self.sim_s = msg.time_boot_ms / 1000.0
            self.lean_deg = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(msg.roll) * math.cos(msg.pitch)))))
        elif kind == "NAV_CONTROLLER_OUTPUT":
            self.wp_dist, self.wp_bearing = float(msg.wp_dist), float(msg.target_bearing)
        elif kind == "SERVO_OUTPUT_RAW":
            self.motors = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
        elif kind == "WIND":
            self.wind_est = float(msg.speed)
        elif kind == "EKF_STATUS_REPORT":
            self.ekf_flags = int(msg.flags)

    def pump(self, seconds: float, until=None):
        """Read messages for some REAL seconds, or until the test says stop."""
        end = time.monotonic() + seconds
        last = {}
        while time.monotonic() < end:
            msg = self.mav.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            last[msg.get_type()] = msg
            self._take(msg)
            if until is not None and until(last):
                return last
        return last

    def fly(self, sim_seconds: float, until=None, each_second=None) -> bool:
        """Let the simulated clock run. Returns True when `until` came true first."""
        end = self.sim_s + sim_seconds
        deadline = time.monotonic() + sim_seconds / self.speedup * 4 + 20
        next_tick = math.floor(self.sim_s) + 1
        while self.sim_s < end and time.monotonic() < deadline:
            msg = self.mav.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            self._take(msg)
            if each_second is not None and self.sim_s >= next_tick:
                next_tick = math.floor(self.sim_s) + 1
                each_second()
            if until is not None and until():
                return True
        return False

    def saw(self, needle: str, since: float = 0.0) -> bool:
        return any(needle.lower() in text.lower() for when, text in self.texts if when >= since)

    def texts_with(self, needle: str, since: float = 0.0) -> list[str]:
        return [text for when, text in self.texts if when >= since and needle.lower() in text.lower()]

    # --- commands --------------------------------------------------------
    def set_param(self, name: str, value: float) -> None:
        for _ in range(5):
            self.mav.mav.param_set_send(self.mav.target_system, self.mav.target_component, name.encode(),
                                        float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            end = time.monotonic() + 2
            while time.monotonic() < end:
                got = self.mav.recv_match(blocking=True, timeout=0.5)
                if got is None:
                    continue
                self._take(got)
                if got.get_type() == "PARAM_VALUE" and got.param_id.strip("\x00") == name:
                    return
        raise RuntimeError(f"parameter {name} was not accepted")

    def get_param(self, name: str):
        for _ in range(5):
            self.mav.mav.param_request_read_send(self.mav.target_system, self.mav.target_component, name.encode(), -1)
            end = time.monotonic() + 2
            while time.monotonic() < end:
                got = self.mav.recv_match(blocking=True, timeout=0.5)
                if got is None:
                    continue
                self._take(got)
                if got.get_type() == "PARAM_VALUE" and got.param_id.strip("\x00") == name:
                    return got.param_value
        return None

    def gps_off(self) -> None:
        name, value = GPS_OFF[self.version[:3]]
        self.set_param(name, value)

    def request_mode(self, name: str) -> None:
        """Ask for a mode and do not wait."""
        number = {v: k for k, v in COPTER_MODES.items()}[name]
        self.mav.set_mode(number)

    def set_mode(self, name: str) -> None:
        self.request_mode(name)
        self.pump(15, until=lambda last: self.mode == name)
        if self.mode != name:
            raise RuntimeError(f"mode {name} not reached (still {self.mode})")

    def rc(self, **channels: int) -> None:
        """Hold RC sticks, for example rc(rc3=1500). Kept until changed (RC_OVERRIDE_TIME -1)."""
        values = [65535] * 8
        for key, pwm in channels.items():
            values[int(key[2:]) - 1] = pwm
        for _ in range(3):
            self.mav.mav.rc_channels_override_send(self.mav.target_system, self.mav.target_component, *values)
            self.pump(0.05)

    def goto(self, north_m: float, east_m: float, alt_m: float) -> None:
        """In GUIDED: fly to a point given in metres from home."""
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000_1101_1111_1000, north_m, east_m, -alt_m, 0, 0, 0, 0, 0, 0, 0, 0)

    def velocity(self, north_mps: float, east_mps: float) -> None:
        """In GUIDED: fly at this speed over the ground. Send it again every second, it times out."""
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000_1101_1100_0111, 0, 0, 0, north_mps, east_mps, 0, 0, 0, 0, 0, 0)

    def arm_and_takeoff(self, alt_m: float) -> None:
        # Wait until the simulator's position estimate is ready.
        self.pump(120, until=lambda last: "EKF_STATUS_REPORT" in last and (last["EKF_STATUS_REPORT"].flags & 0x18) == 0x18
                  and "GPS_RAW_INT" in last and last["GPS_RAW_INT"].fix_type >= 3)
        self.rc(rc3=1000)   # arming needs the throttle stick down
        self.set_mode("GUIDED")
        for _ in range(30):
            self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                           mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            self.pump(2, until=lambda last: self.armed)
            if self.armed:
                break
        if not self.armed:
            raise RuntimeError("could not arm: " + " | ".join(t for _, t in self.texts[-6:]))
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt_m)
        self.pump(90, until=lambda last: self.alt >= alt_m * 0.95)
        if self.alt < alt_m * 0.95:
            raise RuntimeError(f"take-off stopped at {self.alt:.1f} m")

    def hover_in_loiter(self, alt_m: float = 20.0) -> None:
        """Take off and hold position in LOITER with the throttle stick in the middle."""
        self.arm_and_takeoff(alt_m)
        self.rc(rc3=1500)
        self.set_mode("LOITER")

    def close(self) -> None:
        try:
            self.mav.close()
        except Exception:
            pass
        try:
            self.proc.terminate()
        except Exception:
            pass
        if self.instance:
            wsl(f"kill -CONT $(cat {self.run_dir}/arducopter.pid) 2>/dev/null; "
                f"kill $(cat {self.run_dir}/arducopter.pid) 2>/dev/null; true")
        else:
            wsl("pkill -x arducopter 2>/dev/null; true")

    def freeze(self) -> None:
        """Stop this simulator where it is: its link goes silent, like a radio out of range."""
        wsl(f"kill -STOP $(cat {self.run_dir}/arducopter.pid)")

    def unfreeze(self) -> None:
        wsl(f"kill -CONT $(cat {self.run_dir}/arducopter.pid)")

    def log_tail(self, lines: int = 15) -> str:
        return wsl(f"tail -n {lines} {self.run_dir}/sitl.log")
