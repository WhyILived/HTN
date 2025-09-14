#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import threading
import json
import cv2
import time

bridge = CvBridge()
cam_rgb = None
cam_lock = threading.Lock()
CONF_THRESHOLD = 0.4
pub = None

# Load YOLOv8 model
yolo_model = YOLO("yolov8n.pt")

def rgb_callback(msg):
    global cam_rgb
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    with cam_lock:
        cam_rgb = frame.copy()

def yolo_thread():
    global cam_rgb, pub
    while not rospy.is_shutdown():
        with cam_lock:
            frame = cam_rgb.copy() if cam_rgb is not None else None

        if frame is None:
            time.sleep(0.05)
            continue

        results = yolo_model.predict(source=frame, conf=CONF_THRESHOLD, verbose=False)
        objs_to_send = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = yolo_model.names[cls_id]
                # if label != "chair":  # only send chairs
                #     continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2)//2, (y1 + y2)//2
                objs_to_send.append({"label": label, "cx": cx, "cy": cy})

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

        pub.publish(json.dumps(objs_to_send))
        cv2.imshow("YOLO RGB", frame)
        cv2.waitKey(1)
        time.sleep(0.01)

def main():
    global pub
    rospy.init_node("rgb_yolo_node")
    rospy.Subscriber("/camera/rgb/image_raw", Image, rgb_callback)
    pub = rospy.Publisher("/yolo_objects_raw", String, queue_size=1)

    thread = threading.Thread(target=yolo_thread)
    thread.daemon = True
    thread.start()

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()