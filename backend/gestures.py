import cv2
import math
import mediapipe as mp

# --- Setup ---
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_styles  = mp.solutions.drawing_styles

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
    # Thumb: 4 tip, 3 ip, 2 mcp; Index: 8 tip, 6 pip, 5 mcp; Middle: 12/10/9; Ring: 16/14/13; Pinky: 20/18/17
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

    # Thumb extended heuristics: compare x for left/right hands (because thumb sticks sideways)
    is_right = (handedness_label == "Right")
    if is_right:
        thumb_extended = lm[TH_TIP].x < lm[TH_IP].x < lm[TH_MCP].x  # Right hand: thumb leftward
    else:
        thumb_extended = lm[TH_TIP].x > lm[TH_IP].x > lm[TH_MCP].x  # Left hand: thumb rightward

    # Count extended fingers
    count_extended = sum(fingers.values()) + (1 if thumb_extended else 0)

    # Gesture rules (basic, tweak as needed)
    if count_extended == 5:
        return "Open Palm 🖐️"
    if count_extended == 0:
        return "Fist ✊"

    # Peace sign: index+middle extended, ring+pinky curled (thumb anything)
    if fingers["index"] and fingers["middle"] and not fingers["ring"] and not fingers["pinky"]:
        return "Peace ✌️"

    # Thumbs Up / Down: thumb extended, others mostly curled; use vertical direction
    others_curled = (sum(fingers.values()) <= 1)
    if thumb_extended and others_curled:
        # Compare thumb tip vs wrist in Y (image origin top-left, so smaller y == higher on screen)
        if lm[4].y < lm[0].y - 0.05:
            return "Thumbs Up 👍"
        elif lm[4].y > lm[0].y + 0.05:
            return "Thumbs Down 👎"

    # Fallback: show count
    return f"{count_extended} fingers"

def main():
    cap = cv2.VideoCapture(0)  # change to 1 if you have multiple cameras
    if not cap.isOpened():
        print("Could not open webcam.")
        return

    with mp_hands.Hands(
        model_complexity=0,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    ) as hands:

        while True:
            ok, frame = cap.read()
            if not ok:
                break

            # BGR -> RGB for MediaPipe
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            results = hands.process(rgb)
            rgb.flags.writeable = True

            gesture_texts = []

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
                    gesture_texts.append(f"{label}: {gesture}")

                    # Put label near index MCP
                    h, w = frame.shape[:2]
                    x = int(lm[5].x * w)
                    y = int(lm[5].y * h) - 10
                    cv2.putText(frame, f"{label}: {gesture}", (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

            # UI
            cv2.putText(frame, "Press 'q' to quit", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow("Hand Gesture Detection (OpenCV + MediaPipe)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
