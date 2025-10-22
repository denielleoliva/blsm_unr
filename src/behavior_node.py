#!/usr/bin/env python3
import os, math, time, yaml, random, signal
from typing import Dict, Optional, List
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# ---------- helpers ----------
def clamp(x, lo, hi): return max(lo, min(hi, x))

def load_calib(path: Optional[str]) -> Dict[str, Dict[str, float]]:
    if not path or not os.path.exists(path): return {}
    with open(path, "r") as f: return yaml.safe_load(f) or {}

# Compose string joint angles (deg) for a desired head pitch/roll around neutral.
# Strings tighten when increased from neutral (≈ 180° ticks in XL-330 mapping).
def compose_strings(pitch_deg: float, roll_deg: float,
                    neu_front: float, neu_br: float, neu_bl: float,
                    k_pitch: float = 0.6, k_roll: float = 0.6,
                    bias: float = 0.0):
    # Front string reacts to +pitch by tightening; backs react to -pitch.
    A = neu_front + (k_pitch * pitch_deg) + bias
    BR = neu_br - (k_pitch * pitch_deg) + (k_roll * roll_deg) - bias/2
    BL = neu_bl - (k_pitch * pitch_deg) - (k_roll * roll_deg) - bias/2
    return A, BR, BL

# Convert deg to radians
def d2r(d): return d * math.pi / 180.0

# ---------- behavior node ----------
class BlossomBehaviorNode(Node):
    def __init__(self):
        super().__init__('blossom_behaviors')
        # Params
        self.declare_parameter('topic', '/blossom_position_controller/commands')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('calibration', '')  # path to blossom_calibration.yaml (optional)
        self.declare_parameter('base_neutral_deg', 180.0)
        self.declare_parameter('idle_speed', 0.5)          # base wiggle speed multiplier
        self.declare_parameter('amplitude_pitch', 6.0)     # deg
        self.declare_parameter('amplitude_roll',  6.0)     # deg

        self.topic  = self.get_parameter('topic').get_parameter_value().string_value
        self.rate   = float(self.get_parameter('rate_hz').value)
        calib_path  = self.get_parameter('calibration').get_parameter_value().string_value
        self.base_neutral_deg = float(self.get_parameter('base_neutral_deg').value)

        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)

        # Load calib (if available)
        self.cal = load_calib(calib_path).get('blossom_calibration', {})
        pose_cfg = load_calib(calib_path).get('pose_composer', {}) if calib_path else {}
        # Neutral defaults
        self.neu_front = self._get_neu('string_front', default=180.0)
        self.neu_br    = self._get_neu('string_back_right', default=180.0)
        self.neu_bl    = self._get_neu('string_back_left',  default=180.0)
        # Gains
        self.k_pitch = float(pose_cfg.get('k_pitch', 0.6))
        self.k_roll  = float(pose_cfg.get('k_roll',  0.6))
        self.bias_default = float(pose_cfg.get('tension_bias_default', 0.0))

        self.get_logger().info(f"Loaded calib neutrals: front={self.neu_front:.1f}  "
                               f"BR={self.neu_br:.1f}  BL={self.neu_bl:.1f}")
        self.get_logger().info(f"Gains: k_pitch={self.k_pitch} k_roll={self.k_roll}  bias={self.bias_default}")

        # Behavior state
        self.t0 = time.time()
        self.idle_speed = float(self.get_parameter('idle_speed').value)
        self.amp_pitch  = float(self.get_parameter('amplitude_pitch').value)
        self.amp_roll   = float(self.get_parameter('amplitude_roll').value)

        # Schedule ticks
        self.timer = self.create_timer(1.0 / self.rate, self._on_tick)

        # stochastic micro-behaviors
        self.next_glance_t = 0.0
        self.next_nod_t = 0.0
        self._schedule_spontaneous()

        # graceful stop: return to neutral when killed
        signal.signal(signal.SIGINT, self._sigint)

    def _get_neu(self, name, default=180.0):
        entry = self.cal.get(name, {})
        return float(entry.get('neutral_deg', default))

    def _limit(self, name, val):
        entry = self.cal.get(name, {})
        mn = entry.get('min_deg', 150.0)
        mx = entry.get('max_deg', 210.0)
        return clamp(val, mn, mx)

    def _send(self, base_deg, front_deg, br_deg, bl_deg):
        # Clamp if calibration provided
        front_deg = self._limit('string_front', front_deg)
        br_deg    = self._limit('string_back_right', br_deg)
        bl_deg    = self._limit('string_back_left',  bl_deg)
        msg = Float64MultiArray()
        msg.data = [d2r(base_deg), d2r(front_deg), d2r(br_deg), d2r(bl_deg)]
        self.pub.publish(msg)

    def _sigint(self, *_):
        self.get_logger().info("Stopping behaviors -> returning to neutral...")
        # smooth glide to neutral over ~1.0s
        curr_t = time.time()
        for k in range(int(self.rate)):
            alpha = 1.0 - (k / self.rate)
            base = self.base_neutral_deg
            f, br, bl = compose_strings(0.0, 0.0, self.neu_front, self.neu_br, self.neu_bl,
                                        self.k_pitch, self.k_roll, self.bias_default)
            self._send(base, f, br, bl)
            time.sleep(1.0 / self.rate)
        rclpy.shutdown()

    # ---------- behavior scheduler ----------
    def _schedule_spontaneous(self):
        now = time.time()
        self.next_glance_t = now + random.uniform(3.0, 8.0)
        self.next_nod_t    = now + random.uniform(10.0, 20.0)

    def _on_tick(self):
        t = time.time() - self.t0
        # base idle: gentle pan sway
        base = self.base_neutral_deg + 8.0 * math.sin(0.15 * self.idle_speed * t)

        # ambient breathing: slow pitch/roll Lissajous
        pitch = self.amp_pitch * math.sin(0.10 * self.idle_speed * t)
        roll  = self.amp_roll  * math.sin(0.13 * self.idle_speed * t + 1.2)

        # micro behavior triggers inject brief overlays
        overlay_pitch, overlay_roll = 0.0, 0.0
        now = time.time()

        if now >= self.next_glance_t:
            # glance: quick roll left or right with a slight counter-pitch
            side = random.choice([-1.0, 1.0])
            dur = 0.7
            phase = min(1.0, (now - self.next_glance_t) / dur)
            overlay_roll = side * 10.0 * math.sin(math.pi * phase)  # out and back
            overlay_pitch = -3.0 * math.sin(math.pi * phase)
            if phase >= 1.0:
                # reschedule next glance
                self.next_glance_t = now + random.uniform(4.0, 9.0)

        if now >= self.next_nod_t:
            # nod: quick +pitch then return
            dur = 0.9
            phase = min(1.0, (now - self.next_nod_t) / dur)
            overlay_pitch += 8.0 * math.sin(math.pi * phase)
            if phase >= 1.0:
                self.next_nod_t = now + random.uniform(12.0, 25.0)

        # combine ambient + overlays
        pitch += overlay_pitch
        roll  += overlay_roll

        # compose to strings
        f, br, bl = compose_strings(pitch, roll,
            self.neu_front, self.neu_br, self.neu_bl,
            self.k_pitch, self.k_roll, self.bias_default)

        self._send(base, f, br, bl)

def main():
    rclpy.init()
    node = BlossomBehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
