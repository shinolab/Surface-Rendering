import pyrealsense2 as rs
import numpy as np
import mediapipe as mp
import cv2
import math

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            results = hands.process(color_image)

            color_image.flags.writeable = True

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        color_image,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())
                    
                    x_index = math.floor(hand_landmarks.landmark[8].x * 640)
                    y_index = math.floor(hand_landmarks.landmark[8].y * 480)
                    cv2.circle(color_image, (x_index, y_index), 10, (0, 0, 255))
                    # print(x_index, y_index)
                    # print(hand_landmarks.landmark[8].x, hand_landmarks.landmark[8].y)
                    finger_dis = depth_image[x_index][y_index]
                    print(finger_dis)

                # Flip the image horizontally for a selfie-view display.
                cv2.imshow('MediaPipe Hands', cv2.flip(color_image, 1))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
