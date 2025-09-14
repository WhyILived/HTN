#!/usr/bin/env python3
import cv2
import math
import json
import numpy as np
import threading
import time

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from cv_bridge import CvBridge

import mediapipe as mp

# =========================
# Globals / State
# =========================
bridge = CvBridge()

cam_lock = threading.Lock()
cam_rgb = None           # latest BGR frame
depth_img = None         # latest depth image as np.ndarray (meters)
K = None                 # camera intrinsics (fx, fy, cx, cy)
depth_encoding = None    # "16UC1" (mm) or "32FC1" (m)

target_lock = threading.Lock()
target_uv = None         # (u,v) pixel center of target
target_area = 0.0        # area used to pick largest bbox during warm-up
target_ready = False

hand3d = None            # last 3D point of hand (camera frame)
target3d = None          # last 3D point of target (camera frame)

# params
WARMUP_FRAMES = 30       # frames to memorize largest bbox
DEPTH_WIN = 5            # median window (odd) around pixel to sample depth
PIXEL_STEP = 1           # step in window
CLOSE_THRESH_M = 0.07    # distance at which we say we're "very close"
XY_DEADZONE_M = 0.03     # how precise in X/Y before we say centered
LABEL_FILTER = None      # only lock onto a specific label if set (string)

# topics (can be overridden by ROS params)
RGB_TOPIC_DEFAULT   = "/camera/rgb/image_color"
DEPTH_TOPIC_DEFAULT = "/camera/depth/image_raw"
CINFO_TOPIC_DEFAULT = "/camera/depth/camera_info"
YOLO_TOPIC_DEFAULT  = "/yolo_objects_raw"

# publishers
guidance_pub = None

# =========================
# Helpers
# =========================

def parse_yolo_objects(msg_str):
    """
    Parse /yolo_objects_raw JSON.
    Accepts items like:
      {label, cx, cy, w, h} OR {label, x1,y1,x2,y2} OR {label, cx,cy}.
    Returns list of dicts with keys: cx, cy, area, label.
    """
    out = []
    try:
        items = json.loads(msg_str)
    except Exception:
        return out

    if not isinstance(items, list):
        return out

    for it in items:
        if not isinstance(it, dict):
            continue
        label = it.get("label", "")
        # center
        if "cx" in it and "cy" in it:
            cx, cy = float(it["cx"]), float(it["cy"])
            # area
            if "w" in it and "h" in it:
                area = float(it["w"]) * float(it["h"])
            elif all(k in it for k in ("x1","y1","x2","y2")):
                w = abs(float(it["x2"]) - float(it["x1"]))
                h = abs(float(it["y2"]) - float(it["y1"]))
                area = max(1.0, w*h)
            else:
                area = 1.0
        elif all(k in it for k in ("x1","y1","x2","y2")):
            x1,y1,x2,y2 = map(float, (it["x1"], it["y1"], it["x2"], it["y2"]))
            cx, cy = 0.5*(x1+x2), 0.5*(y1+y2)
            area = max(1.0, abs(x2-x1)*abs(y2-y1))
        else:
            continue

        out.append({"label": label, "cx": cx, "cy": cy, "area": area})
    return out

def depth_at(u, v):
    """Median depth (meters) around pixel (u,v) with a small window."""
    global depth_img, depth_encoding
    if depth_img is None:
        return None
    h, w = depth_img.shape[:2]
    u0 = int(np.clip(u, 0, w-1))
    v0 = int(np.clip(v, 0, h-1))
    k = DEPTH_WIN//2

    vals = []
    for dv in range(-k, k+1, PIXEL_STEP):
        for du in range(-k, k+1, PIXEL_STEP):
            uu = np.clip(u0+du, 0, w-1)
            vv = np.clip(v0+dv, 0, h-1)
            d = depth_img[vv, uu]
            if depth_encoding == "16UC1":
                if d > 0:
                    vals.append(d * 0.001)  # mm -> m
            else:
                # float meters
                if np.isfinite(d) and d > 0:
                    vals.append(float(d))
    if not vals:
        return None
    return float(np.median(vals))

def deproject(u, v, Z, fx, fy, cx, cy):
    """Pixel (u,v) + depth Z -> 3D (X,Y,Z) in camera frame (meters)."""
    X = (u - cx) / fx * Z
    Y = (v - cy) / fy * Z
    return np.array([X, Y, Z], dtype=np.float32)

def hand_center_px(lm_list, w, h):
    """Average all landmark pixels to get a stable hand center."""
    xs = [int(l.x * w) for l in lm_list]
    ys = [int(l.y * h) for l in lm_list]
    return (int(np.mean(xs)), int(np.mean(ys)))

def direction_from_delta(dx, dy):
    """
    Given dx, dy in meters (target - hand) in camera frame:
      X>0 means target is to the RIGHT in image
      Y>0 means target is DOWN in image
    Return simple guidance text.
    """
    parts = []
    if abs(dx) > XY_DEADZONE_M:
        parts.append("right" if dx > 0 else "left")
    if abs(dy) > XY_DEADZONE_M:
        parts.append("down" if dy > 0 else "up")
    if not parts:
        return "centered"
    return " & ".join(parts)

# =========================
# Callbacks
# =========================

def rgb_callback(msg):
    global cam_rgb
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    with cam_lock:
        cam_rgb = frame

def depth_callback(msg):
    global depth_img, depth_encoding
    depth_encoding = msg.encoding  # "16UC1" or "32FC1"
    img = bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
    # Convert to numpy array (no copy when possible)
    depth_img = np.array(img, copy=False)

def camera_info_callback(msg):
    global K
    fx = msg.K[0]; fy = msg.K[4]; cx = msg.K[2]; cy = msg.K[5]
    K = (fx, fy, cx, cy)

def yolo_callback(msg):
    global target_uv, target_area, target_ready, LABEL_FILTER
    if target_ready:
        return
    objs = parse_yolo_objects(msg.data)
    if LABEL_FILTER:
        objs = [o for o in objs if o["label"] == LABEL_FILTER]
    if not objs:
        return
    best = max(objs, key=lambda o: o["area"])
    # update if larger than current best
    if best["area"] > target_area:
        with target_lock:
            target_area = best["area"]
            target_uv = (best["cx"], best["cy"])

def pose_callback(msg: PoseWithCovarianceStamped):
    # Placeholder: if you want "map/world" coords, you can transform
    # camera-frame points with TF using this pose and camera->base link.
    # For this task we keep everything in camera frame.
    pass

# =========================
# Main processing loop
# =========================

def run():
    global target_ready, hand3d, target3d

    rospy.init_node("depth_localization_node")

    # Params
    rgb_topic   = rospy.get_param("~rgb_topic",   RGB_TOPIC_DEFAULT)
    depth_topic = rospy.get_param("~depth_topic", DEPTH_TOPIC_DEFAULT)
    cinfo_topic = rospy.get_param("~camera_info_topic", CINFO_TOPIC_DEFAULT)
    yolo_topic  = rospy.get_param("~yolo_topic",  YOLO_TOPIC_DEFAULT)
    target_label_param = rospy.get_param("~target_label", "")
    if target_label_param:
        # only memorize largest bbox of this label
        global LABEL_FILTER
        LABEL_FILTER = target_label_param

    rospy.loginfo(f"Subscribing: rgb={rgb_topic}, depth={depth_topic}, cinfo={cinfo_topic}, yolo={yolo_topic}")

    rospy.Subscriber(rgb_topic,   Image,      rgb_callback,   queue_size=1, buff_size=2**24)
    rospy.Subscriber(depth_topic, Image,      depth_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber(cinfo_topic, CameraInfo, camera_info_callback, queue_size=1)
    rospy.Subscriber(yolo_topic,  String,     yolo_callback,  queue_size=1)
    rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, pose_callback, queue_size=1)

    global guidance_pub
    guidance_pub = rospy.Publisher("/hand_guidance", String, queue_size=1)

    # MediaPipe
    mp_hands = mp.solutions.hands
    mp_draw  = mp.solutions.drawing_utils
    mp_style = mp.solutions.drawing_styles

    warmup_counter = 0

    rate = rospy.Rate(30)
    with mp_hands.Hands(
        model_complexity=0,
        max_num_hands=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    ) as hands:
        while not rospy.is_shutdown():
            with cam_lock:
                frame = None if cam_rgb is None else cam_rgb.copy()

            if frame is None or K is None or depth_img is None:
                rate.sleep()
                continue

            h, w = frame.shape[:2]

            # Hand detection
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            results = hands.process(rgb)
            rgb.flags.writeable = True

            hand_uv = None
            if results.multi_hand_landmarks:
                lm = results.multi_hand_landmarks[0].landmark
                hand_uv = hand_center_px(lm, w, h)
                # draw landmarks
                mp_draw.draw_landmarks(
                    frame, results.multi_hand_landmarks[0],
                    mp_hands.HAND_CONNECTIONS,
                    mp_style.get_default_hand_landmarks_style(),
                    mp_style.get_default_hand_connections_style(),
                )
                cv2.circle(frame, hand_uv, 6, (0,255,255), -1)

            # During warmup, keep updating target with the largest bbox we see
            if not target_ready:
                warmup_counter += 1
                if warmup_counter >= WARMUP_FRAMES and target_uv is not None:
                    target_ready = True
                    rospy.loginfo(f"Locked target at {target_uv} (area {target_area:.1f})")

            # Draw target pixel if known
            if target_uv is not None:
                cv2.circle(frame, (int(target_uv[0]), int(target_uv[1])), 6, (0,165,255), -1)  # orange
                cv2.putText(frame, "TARGET", (int(target_uv[0])+8,int(target_uv[1])-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,165,255), 2, cv2.LINE_AA)

            guidance_text = "Waiting for target..." if not target_ready else "Show your hand"
            extra = ""

            if target_ready and hand_uv is not None:
                fx, fy, cx, cy = K
                # Depth at hand and target
                Zh = depth_at(hand_uv[0], hand_uv[1])
                Zt = depth_at(target_uv[0], target_uv[1]) if target_uv is not None else None

                if Zh is not None:
                    hand3d = deproject(hand_uv[0], hand_uv[1], Zh, fx, fy, cx, cy)
                    cv2.putText(frame, f"Hand Z={hand3d[2]:.2f}m", (10, h-40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)

                if Zt is not None and target_uv is not None:
                    target3d = deproject(target_uv[0], target_uv[1], Zt, fx, fy, cx, cy)
                    cv2.putText(frame, f"Target Z={target3d[2]:.2f}m", (10, h-15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2, cv2.LINE_AA)

                if hand3d is not None and target3d is not None:
                    delta = target3d - hand3d  # (dx, dy, dz) in camera frame (meters)
                    dx, dy, dz = float(delta[0]), float(delta[1]), float(delta[2])

                    # Lateral guidance (image plane)
                    dir2d = direction_from_delta(dx, dy)

                    # Close?
                    close = np.linalg.norm(delta) < CLOSE_THRESH_M
                    if close:
                        guidance_text = "Arrived ✅"
                    else:
                        guidance_text = f"Move {dir2d}"
                        extra = f" | ΔXYZ=({dx:+.2f},{dy:+.2f},{dz:+.2f})m"

                    # publish guidance
                    if guidance_pub:
                        guidance_pub.publish(guidance_text)

                    # draw vector on image
                    u1, v1 = int(hand_uv[0]), int(hand_uv[1])
                    u2, v2 = int(target_uv[0]), int(target_uv[1])
                    cv2.arrowedLine(frame, (u1,v1), (u2,v2), (0,255,0), 2, tipLength=0.2)

            # HUD
            hdr = f"Warmup {warmup_counter}/{WARMUP_FRAMES}" if not target_ready else "Tracking"
            cv2.putText(frame, hdr, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
            cv2.putText(frame, guidance_text + extra, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if "Move" in guidance_text else (255,255,0), 2, cv2.LINE_AA)

            cv2.imshow("Hand-to-Target Guidance (ROS + Depth + MediaPipe)", frame)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break

            rate.sleep()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
