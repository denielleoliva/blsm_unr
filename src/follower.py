#!/usr/bin/env python3
import os, math, time, yaml, cv2, numpy as np
from typing import Dict, Any, Tuple, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def d2r(x): return x * math.pi / 180.0
def clamp(v, lo, hi): return max(lo, min(hi, v))

def load_yaml(path: Optional[str]) -> Dict[str, Any]:
    if not path or not os.path.exists(path): return {}
    with open(path, "r") as f: return yaml.safe_load(f) or {}

def compose_strings(pitch_deg: float, roll_deg: float,
                    neu_front: float, neu_br: float, neu_bl: float,
                    k_pitch: float, k_roll: float, bias: float):
    A  = neu_front + (k_pitch * pitch_deg) + bias
    BR = neu_br    - (k_pitch * pitch_deg) + (k_roll * roll_deg) - bias/2
    BL = neu_bl    - (k_pitch * pitch_deg) - (k_roll * roll_deg) - bias/2
    return A, BR, BL

class PI:
    def __init__(self, kp=0.0, ki=0.0, i_lim=1e9):
        self.kp = kp; self.ki = ki; self.ei = 0.0; self.i_lim = i_lim
    def reset(self): self.ei = 0.0
    def step(self, e, dt):
        self.ei = clamp(self.ei + e*dt, -self.i_lim, self.i_lim)
        return self.kp*e + self.ki*self.ei

class BlossomGaze(Node):
    def __init__(self):
        super().__init__("blossom_gaze_follow")
        # Parameters
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_topic", "/blossom_position_controller/commands")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("calibration_yaml", "")
        self.declare_parameter("base_neutral_deg", 180.0)
        self.declare_parameter("base_limits_deg", [140.0, 220.0])   # [min,max]
        self.declare_parameter("target_aspect", 0.56)               # w/h heuristic
        self.declare_parameter("detect_interval", 10)               # frames between HOG detections
        self.declare_parameter("debug_image", False)

        # Controller gains (tune as needed; units in degrees of output per normalized pixel error)
        self.declare_parameter("yaw_kp",  35.0)
        self.declare_parameter("yaw_ki",   0.0)
        self.declare_parameter("pitch_kp", 18.0)
        self.declare_parameter("pitch_ki",  0.0)
        self.declare_parameter("roll_kp",   6.0)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.cmd_topic   = self.get_parameter("cmd_topic").get_parameter_value().string_value
        self.rate_hz     = float(self.get_parameter("rate_hz").value)
        self.base_neu    = float(self.get_parameter("base_neutral_deg").value)
        self.base_lo, self.base_hi = [float(x) for x in self.get_parameter("base_limits_deg").get_parameter_value().double_array_value]
        self.detect_interval = int(self.get_parameter("detect_interval").value)
        self.debug_image = bool(self.get_parameter("debug_image").value)

        # Gains
        self.pi_yaw   = PI(self.get_parameter("yaw_kp").value,   self.get_parameter("yaw_ki").value,   i_lim=1.0)
        self.pi_pitch = PI(self.get_parameter("pitch_kp").value, self.get_parameter("pitch_ki").value, i_lim=1.0)
        self.k_roll   = float(self.get_parameter("roll_kp").value)

        # Calibration
        cal = load_yaml(self.get_parameter("calibration_yaml").get_parameter_value().string_value)
        bc  = cal.get("blossom_calibration", {})
        pc  = cal.get("pose_composer", {})
        self.neu_front = float(bc.get("string_front", {}).get("neutral_deg", 180.0))
        self.neu_br    = float(bc.get("string_back_right", {}).get("neutral_deg", 180.0))
        self.neu_bl    = float(bc.get("string_back_left", {}).get("neutral_deg", 180.0))
        self.g_pitch   = float(pc.get("k_pitch", 0.6))
        self.g_roll    = float(pc.get("k_roll",  0.6))
        self.bias      = float(pc.get("tension_bias_default", 0.0))

        # ROS I/O
        self.pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)

        # HOG person detector + KCF tracker
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.tracker = None  # cv2.TrackerKCF_create()
        self.frame_count = 0

        # State
        self.base_deg   = self.base_neu
        self.front_deg  = self.neu_front
        self.br_deg     = self.neu_br
        self.bl_deg     = self.neu_bl
        self.last_t     = time.time()
        self.get_logger().info(f"Blossom gaze node on {self.image_topic} → {self.cmd_topic}")

    # ---- Image callback ----
    def on_image(self, msg: Image):
        t = time.time()
        dt = max(1e-3, t - self.last_t)
        self.last_t = t

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        H, W = frame.shape[:2]
        cx_img, cy_img = W/2.0, H/2.0

        # Person ROI from tracker or detector
        bbox = None
        used_tracker = False

        if self.tracker is not None:
            ok, bb = self.tracker.update(frame)
            if ok: bbox = bb; used_tracker = True

        # Periodic detector refresh (or if tracker lost)
        if (self.frame_count % self.detect_interval == 0) or (bbox is None):
            # downscale for speed
            scale = 0.75 if max(H, W) > 720 else 1.0
            small = cv2.resize(frame, (int(W*scale), int(H*scale)))
            rects, _ = self.hog.detectMultiScale(small, winStride=(8,8), padding=(16,16), scale=1.05)
            rects = [(int(x/scale), int(y/scale), int(w/scale), int(h/scale)) for (x,y,w,h) in rects]
            # pick the largest (closest) detection
            if len(rects) > 0:
                x,y,w,h = max(rects, key=lambda r: r[2]*r[3])
                bbox = (x, y, w, h)
                self.tracker = cv2.TrackerKCF_create()
                self.tracker.init(frame, bbox)

        self.frame_count += 1

        # If we have a target, compute normalized error
        if bbox is not None:
            x,y,w,h = bbox
            tx = x + w/2.0
            ty = y + h/3.0        # weight upper body/face region
            ex = (tx - cx_img) / max(1.0, W)   # ~[-0.5, 0.5]
            ey = (ty - cy_img) / max(1.0, H)   # ~[-0.5, 0.5]

            # Controllers: yaw from ex (image left/right), pitch from ey (image up/down)
            d_yaw   = self.pi_yaw.step(-ex, dt)     # negative: if person on left (ex<0), yaw left (increase deg?) adjust sign to taste
            d_pitch = self.pi_pitch.step( ey, dt)   # positive ey → person lower → nod down (+pitch)

            # Integrate with small smoothing
            a = 0.7
            self.base_deg = clamp(self.base_deg + d_yaw, self.base_lo, self.base_hi)
            pitch_cmd = a*d_pitch + (1-a)*0.0  # mostly proportional; no slow bias

            # (Optional) roll a bit toward the target
            roll_cmd = -self.k_roll * ex

            # Compose strings and publish
            f, br, bl = compose_strings(pitch_cmd, roll_cmd,
                                        self.neu_front, self.neu_br, self.neu_bl,
                                        self.g_pitch, self.g_roll, self.bias)
            self.front_deg = clamp(f,  120.0, 240.0)
            self.br_deg    = clamp(br, 120.0, 240.0)
            self.bl_deg    = clamp(bl, 120.0, 240.0)
            self._publish(self.base_deg, self.front_deg, self.br_deg, self.bl_deg)

            # Debug overlay
            if self.debug_image:
                color = (0,255,0) if used_tracker else (0,140,255)
                cv2.rectangle(frame, (int(x),int(y)), (int(x+w),int(y+h)), color, 2)
                cv2.circle(frame, (int(tx),int(ty)), 5, (255,0,0), -1)
                cv2.circle(frame, (int(cx_img),int(cy_img)), 4, (255,255,255), -1)
                info = f"ex={ex:+.2f} ey={ey:+.2f} yaw={self.base_deg:.1f} pitch={pitch_cmd:.1f}"
                cv2.putText(frame, info, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)
        else:
            # No target: gently drift base back to neutral & level head
            self.base_deg += clamp(self.base_neu - self.base_deg, -0.8, 0.8)
            f, br, bl = compose_strings(0.0, 0.0,
                                        self.neu_front, self.neu_br, self.neu_bl,
                                        self.g_pitch, self.g_roll, self.bias)
            self._publish(self.base_deg, f, br, bl)

        # Optional: show debug window locally (not a ROS topic)
        if self.debug_image:
            cv2.imshow("blossom_gaze", frame)
            cv2.waitKey(1)

    # ---- Publisher ----
    def _publish(self, base_deg, f, br, bl):
        m = Float64MultiArray()
        m.data = [d2r(base_deg), d2r(f), d2r(br), d2r(bl)]
        self.pub.publish(m)

def main():
    rclpy.init()
    node = BlossomGaze()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: cv2.destroyAllWindows()
        except Exception: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
