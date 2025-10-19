#!/usr/bin/env python3

import os, math, time, yaml, signal, random
from typing import Dict, Any, List, Tuple
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

def d2r(x): return x * math.pi / 180.0
def clamp(v, lo, hi): return max(lo, min(hi, v))

def load_yaml(path: str) -> Dict[str, Any]:
    if not path or not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}

def compose_strings(pitch_deg: float, roll_deg: float,
                    neu_front: float, neu_br: float, neu_bl: float,
                    k_pitch: float, k_roll: float, bias: float):
    A  = neu_front + (k_pitch * pitch_deg) + bias
    BR = neu_br    - (k_pitch * pitch_deg) + (k_roll * roll_deg) - bias/2
    BL = neu_bl    - (k_pitch * pitch_deg) - (k_roll * roll_deg) - bias/2
    return A, BR, BL

class YamlBehaviorPlayer(Node):
    def __init__(self):
        super().__init__("blossom_yaml_player")
        # Params
        self.declare_parameter("topic", "/blossom_position_controller/commands")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("behavior_yaml", "")   # path to behaviors.yaml
        self.declare_parameter("calibration_yaml", "")# path to blossom_calibration.yaml
        self.declare_parameter("base_neutral_deg", 180.0)

        self.topic   = self.get_parameter("topic").get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.beh_path= self.get_parameter("behavior_yaml").get_parameter_value().string_value
        self.cal_path= self.get_parameter("calibration_yaml").get_parameter_value().string_value
        self.base_neutral_deg = float(self.get_parameter("base_neutral_deg").value)

        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)

        # Calibration (optional)
        cal_raw = load_yaml(self.cal_path)
        self.cal = cal_raw.get("blossom_calibration", {})
        pose_cfg = cal_raw.get("pose_composer", {})
        self.k_pitch = float(pose_cfg.get("k_pitch", 0.6))
        self.k_roll  = float(pose_cfg.get("k_roll",  0.6))
        self.bias_default = float(pose_cfg.get("tension_bias_default", 0.0))

        def neu(name, default=180.0):
            return float(self.cal.get(name, {}).get("neutral_deg", default))
        self.neu_front = neu("string_front")
        self.neu_br    = neu("string_back_right")
        self.neu_bl    = neu("string_back_left")

        # Load behaviors YAML
        self.beh = load_yaml(self.beh_path)
        if not self.beh:
            self.get_logger().warn(f"No behaviors loaded from: {self.beh_path}")

        # Scheduler state
        self._t0 = time.time()
        self._tick = self.create_timer(1.0 / self.rate_hz, self._on_tick)
        self._program   = self.beh.get("program", [])     # list of blocks
        self._loop_prog = bool(self.beh.get("loop", True))
        self._pc        = 0                               # program counter
        self._block_t0  = time.time()
        self._active    = None                            # current block dict
        self._key_idx   = 0                               # for keyframe seq

        # graceful stop
        signal.signal(signal.SIGINT, self._sigint)

        self.get_logger().info(f"Loaded {len(self._program)} behavior blocks. loop={self._loop_prog}")
        self._advance_block(initial=True)

    # ---------- core loop ----------
    def _on_tick(self):
        if self._active is None:
            return

        kind = self._active.get("type", "idle")
        dt = time.time() - self._block_t0
        dur = float(self._active.get("duration", 5.0))

        # default generators
        base = self.base_neutral_deg
        pitch, roll = 0.0, 0.0
        bias = float(self._active.get("bias", self.bias_default))

        if kind == "idle_lissajous":
            # ambient motion
            amp_p = float(self._active.get("amplitude_pitch", 6.0))
            amp_r = float(self._active.get("amplitude_roll",  6.0))
            w_p   = float(self._active.get("omega_pitch", 0.10))
            w_r   = float(self._active.get("omega_roll",  0.13))
            base_amp = float(self._active.get("base_amp", 8.0))
            base_w   = float(self._active.get("base_omega", 0.15))
            base = self.base_neutral_deg + base_amp * math.sin(base_w * (time.time()-self._t0))
            pitch = amp_p * math.sin(w_p * (time.time()-self._t0))
            roll  = amp_r * math.sin(w_r * (time.time()-self._t0) + 1.2)

        elif kind == "glance":
            # quick roll to side and back
            side = float(self._active.get("side", random.choice([-1.0, 1.0])))
            amp  = float(self._active.get("roll_amp", 10.0))
            amp_p= float(self._active.get("pitch_coupling", -3.0))
            phase = min(1.0, dt / max(0.1, dur))
            roll  = side * amp * math.sin(math.pi * phase)
            pitch = amp_p * math.sin(math.pi * phase)

        elif kind == "nod":
            amp  = float(self._active.get("pitch_amp", 8.0))
            phase = min(1.0, dt / max(0.1, dur))
            pitch = amp * math.sin(math.pi * phase)

        elif kind == "keyframes":
            # sequence of absolute (base, pitch, roll) frames with durations
            frames: List[Dict[str, float]] = self._active.get("frames", [])
            if not frames:
                self._advance_block()
                return
            # progress inside this keyframe
            kf = frames[self._key_idx]
            kfdur = float(kf.get("duration", 1.0))
            kfbase = float(kf.get("base", self.base_neutral_deg))
            kfpitch= float(kf.get("pitch", 0.0))
            kfroll = float(kf.get("roll",  0.0))
            t_in = dt - sum(float(fr.get("duration",1.0)) for fr in frames[:self._key_idx])
            if t_in >= kfdur:
                self._key_idx += 1
                if self._key_idx >= len(frames):
                    self._advance_block()
                    return
                else:
                    return  # wait next tick to compute next frame
            # smoothstep interpolation within this frame (optional)
            s = max(0.0, min(1.0, t_in / max(1e-3, kfdur)))
            s = s*s*(3-2*s)
            # interpolate from previous frame end (or neutral)
            if self._key_idx == 0:
                p0 = 0.0; r0 = 0.0; b0 = self.base_neutral_deg
            else:
                prev = frames[self._key_idx-1]
                p0 = float(prev.get("pitch", 0.0))
                r0 = float(prev.get("roll",  0.0))
                b0 = float(prev.get("base",  self.base_neutral_deg))
            base  = b0 + (kfbase - b0) * s
            pitch = p0 + (kfpitch - p0) * s
            roll  = r0 + (kfroll  - r0) * s

        else:
            # unknown → skip
            self.get_logger().warn(f"Unknown behavior type: {kind}, skipping.")
            self._advance_block()
            return

        # compose strings
        f, br, bl = compose_strings(pitch, roll,
                                    self.neu_front, self.neu_br, self.neu_bl,
                                    self.k_pitch, self.k_roll, bias)
        self._send(base, f, br, bl)

        # done?
        if kind != "keyframes" and dt >= dur:
            self._advance_block()

    # ---------- helpers ----------
    def _advance_block(self, initial=False):
        if not initial:
            self.get_logger().debug("Advance behavior block")
        if self._pc >= len(self._program):
            if self._loop_prog:
                self._pc = 0
            else:
                self.get_logger().info("Program complete; idling.")
                self._active = None
                return
        self._active = self._program[self._pc]
        self._pc += 1
        self._block_t0 = time.time()
        self._key_idx = 0
        name = self._active.get("name", self._active.get("type", "block"))
        self.get_logger().info(f"→ Now playing: {name}")

    def _limit(self, name: str, val: float) -> float:
        entry = self.cal.get(name, {})
        mn = float(entry.get("min_deg", 150.0))
        mx = float(entry.get("max_deg", 210.0))
        return clamp(val, mn, mx)

    def _send(self, base_deg: float, front_deg: float, br_deg: float, bl_deg: float):
        front_deg = self._limit("string_front", front_deg)
        br_deg    = self._limit("string_back_right", br_deg)
        bl_deg    = self._limit("string_back_left",  bl_deg)
        msg = Float64MultiArray()
        msg.data = [d2r(base_deg), d2r(front_deg), d2r(br_deg), d2r(bl_deg)]
        self.pub.publish(msg)

    def _sigint(self, *_):
        self.get_logger().info("Stopping behaviors → return to neutral.")
        # Glide to neutral for ~1s
        steps = int(self.rate_hz)
        for _ in range(steps):
            f, br, bl = compose_strings(0.0, 0.0,
                self.neu_front, self.neu_br, self.neu_bl,
                self.k_pitch, self.k_roll, self.bias_default)
            self._send(self.base_neutral_deg, f, br, bl)
            time.sleep(1.0 / self.rate_hz)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = YamlBehaviorPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
