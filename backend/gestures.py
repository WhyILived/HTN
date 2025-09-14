#!/usr/bin/env python3
import cv2
import math
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time

bridge = CvBridge()
cam_rgb = None
cam_lock = threading.Lock()

# --- Setup ---
mp_hands  = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_styles  = mp.solutions.drawing_styles

def rgb_callback(msg: Image):
    """ROS image callback -> OpenCV BGR frame"""
    global cam_rgb
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    with cam_lock:
        cam_rgb = frame

# Utility: distance between two landmarks
def dist(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

# Utility: finger extended? (tip farther from wrist than pip joint, and roughly straight)
def is_finger_extended(lm, tip, pip, mcp, wrist):
    straight = dist(lm[tip], lm[pip]) + dist(lm[pip], lm[mcp]) > dist(lm[tip], lm[mcp]) * 0.9
    away_from_wrist = dist(lm[tip], lm[wrist]) > dist(lm[pip], lm[wrist]) * 1.05
    return straight and away_from_wrist

# Classify a few simple gestures from landmarks
def classify_gesture(lm, handedness_label):
    # Landmark indices (MediaPipe Hands)
    WRIST = 0
    TH_TIP, TH_IP, TH_MCP = 4, 3, 2
    IX_TIP, IX_PIP, IX_MCP = 8, 6, 5
    MD_TIP, MD_PIP, MD_MCP = 12,10, 9
    RG_TIP, RG_PIP, RG_MCP = 16,14,13
    PK_TIP, PK_PIP, PK_MCP = 20,18,17

    fingers = {
        "index":  is_finger_extended(lm, IX_TIP, IX_PIP, IX_MCP, WRIST),
        "middle": is_finger_extended(lm, MD_TIP, MD_PIP, MD_MCP, WRIST),
        "ring":   is_finger_extended(lm, RG_TIP, RG_PIP, RG_MCP, WRIST),
        "pinky":  is_finger_extended(lm, PK_TIP, PK_PIP, PK_MCP, WRIST),
    }

    # Thumb extended heuristics: compare x for left/right hands
    is_right = (handedness_label == "Right")
    if is_right:
        thumb_extended = lm[TH_TIP].x < lm[TH_IP].x < lm[TH_MCP].x  # Right hand: thumb leftward
    else:
        thumb_extended = lm[TH_TIP].x > lm[TH_IP].x > lm[TH_MCP].x  # Left hand: thumb rightward

    count_extended = sum(fingers.values()) + (1 if thumb_extended else 0)

    if count_extended == 5:
        return "Open Palm 🖐️"
    if count_extended == 0:
        return "Fist ✊"
    if fingers["index"] and fingers["middle"] and not fingers["ring"] and not fingers["pinky"]:
        return "Peace ✌️"

    others_curled = (sum(fingers.values()) <= 1)
    if thumb_extended and others_curled:
        if lm[4].y < lm[0].y - 0.05:
            return "Thumbs Up 👍"
        elif lm[4].y > lm[0].y + 0.05:
            return "Thumbs Down 👎"

    return f"{count_extended} fingers"

def processing_loop(rgb_topic):
    """Main processing/render loop running off the latest frame from the subscriber."""
    with mp_hands.Hands(
        model_complexity=0,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    ) as hands:
        rate = rospy.Rate(60)  # UI loop rate (Hz)
        while not rospy.is_shutdown():
            with cam_lock:
                frame = cam_rgb.copy() if cam_rgb is not None else None

            if frame is None:
                # Wait for first message
                cv2.waitKey(1)
                rate.sleep()
                continue

            # BGR -> RGB for MediaPipe
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            results = hands.process(rgb)
            rgb.flags.writeable = True

            if results.multi_hand_landmarks and results.multi_handedness:
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_styles.get_default_hand_landmarks_style(),
                        mp_styles.get_default_hand_connections_style(),
                    )

                    lm = hand_landmarks.landmark
                    label = handedness.classification[0].label  # "Left" or "Right"
                    gesture = classify_gesture(lm, label)

                    h, w = frame.shape[:2]
                    x = int(lm[5].x * w)
                    y = int(lm[5].y * h) - 10
                    cv2.putText(frame, f"{label}: {gesture}", (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.putText(frame, f"Topic: {rgb_topic}", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, "Press 'q' to quit", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow("Hand Gesture Detection (ROS + MediaPipe)", frame)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                rospy.signal_shutdown("User requested quit")
                break

            rate.sleep()

    cv2.destroyAllWindows()

def main():
    rospy.init_node("hand_gesture_from_ros_rgb")

    # Choose default based on your pipeline; change if you use image_raw
    rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_color")
    rospy.loginfo(f"Subscribing to RGB topic: {rgb_topic}")

    rospy.Subscriber(rgb_topic, Image, rgb_callback, queue_size=1, buff_size=2**24)

    try:
        processing_loop(rgb_topic)
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
