#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import numpy as np
import json
import threading
import math
from tf.transformations import euler_from_quaternion

bridge = CvBridge()

# --- Global state ---
cam_depth = None
cam_intrinsics = {}
latest_detections = []
robot_pose = {"x":0.0, "y":0.0, "yaw":0.0}
permanent_objects = []
lock = threading.Lock()
pub = None

# --- Parameters ---
MIN_OBJ_DIST = 1  # meters: minimum distance between permanent chairs
PATCH_SIZE = 1      # pixels: median patch for depth smoothing (smaller for speed)
CENTER_PIXEL_THRESHOLD = 40  # pixels

# --- Track pending chairs ---
pending_chairs = {}  # key=(px,py) -> {'count': int, 'world_pos': (x,y)}

# --- ROS callbacks ---
def depth_callback(msg):
    global cam_depth
    with lock:
        cam_depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)/1000.0

def camera_info_callback(msg):
    global cam_intrinsics
    cam_intrinsics = {
        "fx": msg.K[0],
        "fy": msg.K[4],
        "cx": msg.K[2],
        "cy": msg.K[5]
    }

def yolo_callback(msg):
    global latest_detections
    try:
        objs = json.loads(msg.data)
        if not isinstance(objs, list):
            objs = []
    except:
        objs = []
    # Only chairs
    objs = [o for o in objs]
    # o.get('label') == 'chair'
    with lock:
        latest_detections = objs

def pose_callback(msg):
    global robot_pose
    pos = msg.pose.pose
    q = pos.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    with lock:
        robot_pose["x"] = pos.position.x
        robot_pose["y"] = pos.position.y
        robot_pose["yaw"] = yaw

# --- Processing thread ---
def process_thread():
    global cam_depth, cam_intrinsics, latest_detections, permanent_objects, pub, pending_chairs
    last_detections = []
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        with lock:
            if cam_depth is None or not latest_detections or not cam_intrinsics:
                rate.sleep()
                continue

            # Skip if detections haven't changed
            if latest_detections == last_detections:
                rate.sleep()
                continue

            depth_local = cam_depth  # avoid copying full depth
            fx = cam_intrinsics["fx"]
            fy = cam_intrinsics["fy"]
            cx0 = cam_intrinsics["cx"]
            cy0 = cam_intrinsics["cy"]
            detections = latest_detections.copy()
            pose = robot_pose.copy()
            last_detections = detections

        h, w = depth_local.shape[:2]
        image_center_x = w // 2
        current_frame_keys = set()

        for obj in detections:
            px = obj.get("cx")
            py = obj.get("cy")
            label = obj.get('label')
            if px is None or py is None:
                continue
            if abs(px - image_center_x) > CENTER_PIXEL_THRESHOLD:
                continue

            px = int(max(0, min(px, w-1)))
            py = int(max(0, min(py, h-1)))
            current_frame_keys.add((px, py))

            # Median depth patch (small 3x3)
            x0 = max(0, px-PATCH_SIZE)
            x1 = min(w-1, px+PATCH_SIZE)
            y0 = max(0, py-PATCH_SIZE)
            y1 = min(h-1, py+PATCH_SIZE)
            patch = depth_local[y0:y1+1, x0:x1+1]
            z = np.median(patch)
            if z <= 0:
                continue

            # Camera to world coordinates
            Xc = (px - cx0) * z / fx
            Yc = (py - cy0) * z / fy
            Zc = z
            world_x = pose["x"] + Zc * math.cos(pose["yaw"]) - Xc * math.sin(pose["yaw"])
            world_y = pose["y"] + Zc * math.sin(pose["yaw"]) + Xc * math.cos(pose["yaw"])

            # Update pending chair
            key = (px, py)
            if key not in pending_chairs:
                pending_chairs[key] = {'count': 1, 'world_pos': (world_x, world_y)}
            else:
                pending_chairs[key]['count'] += 1
                wx, wy = pending_chairs[key]['world_pos']
                pending_chairs[key]['world_pos'] = ((wx + world_x)/2, (wy + world_y)/2)

            # Promote to permanent if seen >1 frame and not too close to existing chairs
            if pending_chairs[key]['count'] >= 2:
                wx, wy = pending_chairs[key]['world_pos']
                if all(math.hypot(o['x']-wx, o['y']-wy) >= MIN_OBJ_DIST for o in permanent_objects):
                    permanent_objects.append({'x': wx, 'y': wy, 'label': f'{label}'})

        # Remove stale pending chairs
        for k in list(pending_chairs):
            if k not in current_frame_keys:
                del pending_chairs[k]

        # Publish permanent objects
        if pub:
            pub.publish(json.dumps(permanent_objects))

        rate.sleep()

def main():
    global pub
    rospy.init_node("depth_localization_node")
    rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback)
    rospy.Subscriber("/camera/depth/camera_info", CameraInfo, camera_info_callback)
    rospy.Subscriber("/yolo_objects_raw", String, yolo_callback)
    rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, pose_callback)
    pub = rospy.Publisher("/yolo_objects", String, queue_size=1)

    thread = threading.Thread(target=process_thread)
    thread.daemon = True
    thread.start()

    rospy.loginfo("Depth + Localization node running...")
    rospy.spin()

if __name__ == "__main__":
    main()