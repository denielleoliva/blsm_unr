#!/usr/bin/env python3
import time
from typing import Dict, Optional

from dynamixel_sdk import PortHandler, PacketHandler 


# -------------------------------
# Control tables (edit if needed)
# -------------------------------
CT_XL320: Dict = {
    "PROTOCOL_VER": 2.0,
    "ADDR_TORQUE_ENABLE": 24,
    "ADDR_LED": 25,                    # optional
    "ADDR_GOAL_POSITION": 30,
    "ADDR_MOVING_SPEED": 32,           # optional
    "ADDR_PRESENT_POSITION": 37,
    "ADDR_PRESENT_VOLTAGE": 45,        # optional
    "ADDR_PRESENT_TEMPERATURE": 46,    # optional
    "LEN_GOAL_POSITION": 2,
    "LEN_PRESENT_POSITION": 2,
    "POS_MIN": 0,
    "POS_MAX": 1023,                   # ~0..300 deg
    "RANGE_DEG": 300.0,
    "DEFAULT_BAUD": 1000000,
}

CT_XL330: Dict = {
    "PROTOCOL_VER": 2.0,
    "ADDR_TORQUE_ENABLE": 64,
    "ADDR_OPERATING_MODE": 11,
    "ADDR_PROFILE_ACCEL": 108,
    "ADDR_PROFILE_VELOCITY": 112,
    "ADDR_GOAL_POSITION": 116,
    "ADDR_PRESENT_POSITION": 132,
    "ADDR_PRESENT_VOLTAGE": 144,       # optional
    "ADDR_PRESENT_TEMPERATURE": 146,   # optional
    "LEN_GOAL_POSITION": 4,
    "LEN_PRESENT_POSITION": 4,
    "POS_MIN": 0,
    "POS_MAX": 4095,                   # 0..360 deg
    "RANGE_DEG": 360.0,
    "DEFAULT_BAUD": 1000000,
}

KNOWN_MODELS: Dict[int, Dict] = {
    350: CT_XL320,  
    1200: CT_XL330, 
}


# -------------------------------
# Helpers
# -------------------------------
def _deg_to_pos(deg: float, ct: Dict) -> int:
    deg = max(0.0, min(ct["RANGE_DEG"], float(deg)))
    pos = int(round(deg * ct["POS_MAX"] / ct["RANGE_DEG"]))
    return max(ct["POS_MIN"], min(ct["POS_MAX"], pos))


def _pos_to_deg(pos: int, ct: Dict) -> float:
    return float(pos) * ct["RANGE_DEG"] / ct["POS_MAX"]


# Bus (serial) layer
class DXLBus:
    def __init__(self, port="/dev/ttyUSB0", baud=57600):
        self.port = PortHandler(port)
        if not self.port.openPort():
            raise RuntimeError(f"Failed to open port {port}")
        if not self.port.setBaudRate(baud):
            raise RuntimeError(f"Failed to set baud {baud}")
        self.packet = PacketHandler(2.0)
        self._servos = []  # track servos for reset-on-close

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close(reset=True)

    def register_servo(self, servo):
        self._servos.append(servo)

    @staticmethod
    def _norm_write(ret):
        if not isinstance(ret, tuple):
            return 0, 0
        if len(ret) == 3:
            _, result, error = ret
        elif len(ret) == 2:
            result, error = ret
        else:
            raise ValueError(f"Unexpected write return: {ret!r}")
        return result, error

    @staticmethod
    def _norm_read(ret):
        if not isinstance(ret, tuple) or len(ret) < 2:
            raise ValueError(f"Unexpected read return: {ret!r}")
        if len(ret) == 3:
            value, result, error = ret
        elif len(ret) == 2:
            value, result = ret
            error = 0
        else:
            raise ValueError(f"Unexpected read return: {ret!r}")
        return value, result, error

    # ---- retry wrappers for COMM_PORT_BUSY (-3001) ----
    def _retry_write(self, call, *args, tries=5, delay=0.01):
        last = (None, None)
        for _ in range(tries):
            result, error = self._norm_write(call(*args))
            last = (result, error)
            if result == -3001:  # COMM_PORT_BUSY
                time.sleep(delay)
                continue
            return result, error
        return last

    def _retry_read(self, call, *args, tries=5, delay=0.01):
        last = (None, None, None)
        for _ in range(tries):
            val, result, error = self._norm_read(call(*args))
            last = (val, result, error)
            if result == -3001:
                time.sleep(delay)
                continue
            return val, result, error
        return last

    # public read/write
    def write1(self, dxl_id: int, addr: int, val: int):
        result, error = self._retry_write(self.packet.write1ByteTxRx, self.port, dxl_id, addr, val)
        if result != 0 or error != 0:
            raise RuntimeError(f"write1 failed id={dxl_id} addr={addr} val={val} res={result} err={error}")

    def write2(self, dxl_id: int, addr: int, val: int):
        result, error = self._retry_write(self.packet.write2ByteTxRx, self.port, dxl_id, addr, val)
        if result != 0 or error != 0:
            raise RuntimeError(f"write2 failed id={dxl_id} addr={addr} val={val} res={result} err={error}")

    def write4(self, dxl_id: int, addr: int, val: int):
        result, error = self._retry_write(self.packet.write4ByteTxRx, self.port, dxl_id, addr, val)
        if result != 0 or error != 0:
            raise RuntimeError(f"write4 failed id={dxl_id} addr={addr} val={val} res={result} err={error}")

    def read2(self, dxl_id: int, addr: int) -> int:
        val, result, error = self._retry_read(self.packet.read2ByteTxRx, self.port, dxl_id, addr)
        if result != 0 or error != 0:
            raise RuntimeError(f"read2 failed id={dxl_id} addr={addr} res={result} err={error}")
        return val

    def read4(self, dxl_id: int, addr: int) -> int:
        val, result, error = self._retry_read(self.packet.read4ByteTxRx, self.port, dxl_id, addr)
        if result != 0 or error != 0:
            raise RuntimeError(f"read4 failed id={dxl_id} addr={addr} res={result} err={error}")
        return val

    # model detection
    def ping_model(self, dxl_id: int) -> Optional[int]:
        try:
            ret = self.packet.ping(self.port, dxl_id)
            if isinstance(ret, tuple):
                if len(ret) == 3:
                    model, _fw, comm = ret
                elif len(ret) == 2:
                    model, comm = ret
                else:
                    raise ValueError(f"Unexpected ping() return: {ret!r}")
                if comm != 0:
                    return None
                return int(model)
            try:
                return int(ret)
            except Exception:
                pass
        except Exception:
            pass

        # fallback for older SDK versions without ping() returning model
        try:
            model = self.read2(dxl_id, 0)
            return int(model)
        except Exception:
            return None

    # reset on close
    def close(self, reset: bool = True):
        if reset:
            for s in self._servos:
                try:
                    s.torque(False)
                except Exception as e:
                    print(f"[warn] failed to torque off ID {getattr(s, 'id', '?')}: {e}")
        try:
            self.port.closePort()
        except Exception as e:
            print(f"[warn] closing port failed: {e}")


# Servo wrapper
class DXLServo:
    """
    XL-320 / XL-330 wrapper.
    - model_override: "XL320", "XL330", or a CT dict. If None -> auto-detect.
    - skip_init: True to skip OperatingMode/Profile writes while stabilizing comms.
    """
    def __init__(self, bus: DXLBus, dxl_id: int, model_override=None, skip_init=False):
        self.bus = bus
        self.id = dxl_id
        self.skip_init = skip_init

        self.bus.register_servo(self)

        if isinstance(model_override, dict):
            self.ct = model_override
            self.model_number = -1
        elif model_override == "XL320":
            self.ct = CT_XL320
            self.model_number = 350
        elif model_override == "XL330":
            self.ct = CT_XL330
            self.model_number = 1200
        else:
            model = self.bus.ping_model(dxl_id)
            if model is None:
                raise RuntimeError(f"No response from ID {dxl_id}")
            self.model_number = int(model)
            if model in KNOWN_MODELS:
                self.ct = KNOWN_MODELS[model]
            elif 1200 <= model < 1300:
                self.ct = CT_XL330
            elif model == 350:
                self.ct = CT_XL320
            else:
                self.ct = CT_XL330  

        if not self.skip_init and "ADDR_OPERATING_MODE" in self.ct:
            try:
                self.bus.write1(self.id, self.ct["ADDR_OPERATING_MODE"], 3)
            except Exception as e:
                print(f"[warn] ID {self.id}: set operating mode failed: {e}")

        if (not self.skip_init and
            "ADDR_PROFILE_ACCEL" in self.ct and
            "ADDR_PROFILE_VELOCITY" in self.ct):
            try:
                self.bus.write4(self.id, self.ct["ADDR_PROFILE_ACCEL"], 0)
                self.bus.write4(self.id, self.ct["ADDR_PROFILE_VELOCITY"], 0)
            except Exception as e:
                print(f"[warn] ID {self.id}: set profile failed: {e}")

    # Public API
    def torque(self, enable: bool):
        self.bus.write1(self.id, self.ct["ADDR_TORQUE_ENABLE"], 1 if enable else 0)

    def set_goal_deg(self, deg: float):
        pos = _deg_to_pos(deg, self.ct)
        if self.ct["LEN_GOAL_POSITION"] == 2:
            self.bus.write2(self.id, self.ct["ADDR_GOAL_POSITION"], pos)
        else:
            self.bus.write4(self.id, self.ct["ADDR_GOAL_POSITION"], pos)

    def get_present_deg(self) -> float:
        if self.ct["LEN_PRESENT_POSITION"] == 2:
            pos = self.bus.read2(self.id, self.ct["ADDR_PRESENT_POSITION"])
        else:
            pos = self.bus.read4(self.id, self.ct["ADDR_PRESENT_POSITION"])
        return _pos_to_deg(pos, self.ct)


# Minimal demo (IDs 1–3 at 57,600)
def demo():
    with DXLBus("/dev/ttyUSB0", 57600) as bus:
        s1 = DXLServo(bus, 1, model_override="XL330", skip_init=True)
        s2 = DXLServo(bus, 2, model_override="XL330", skip_init=True)
        s3 = DXLServo(bus, 3, model_override="XL330", skip_init=True)
        # s4 = DXLServo(bus, 4, model_override="XL330", skip_init=True)

        for s in (s1, s2, s3):
            s.torque(True)

        # Center-ish pose
        s1.set_goal_deg(180); s2.set_goal_deg(180); s3.set_goal_deg(180)
        time.sleep(1.2)

        # Small playful offsets
        s1.set_goal_deg(200); s2.set_goal_deg(160); s3.set_goal_deg(200)
        time.sleep(1.2)

        # Release on exit handled by bus.close(reset=True)
        print("Demo complete.")


if __name__ == "__main__":
    demo()
