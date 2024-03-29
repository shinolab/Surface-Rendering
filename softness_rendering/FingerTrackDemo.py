'''
Author: Mingxin Zhang m.zhang@hapis.u-tokyo.ac.jp
Date: 2023-04-05 17:40:31
LastEditors: Mingxin Zhang
LastEditTime: 2023-04-07 16:02:55
Copyright (c) 2023 by ${git_name}, All Rights Reserved. 
'''
import pyrealsense2 as rs
import numpy as np
import cv2
import mediapipe as mp
import math

# Initialization
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
pipeline = rs.pipeline()
config = rs.config()

W = 640
H = 480

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# Judge whether there is rgb input
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

# Decide resolutions for both depth and rgb streaming
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

try:
    with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            depth_frame = aligned_frames.get_depth_frame() # depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not depth_frame or not color_frame:
                continue

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

                    x_index = math.floor(hand_landmarks.landmark[8].x * W)
                    y_index = math.floor(hand_landmarks.landmark[8].y * H)
                    cv2.circle(color_image, (x_index, y_index), 10, (0, 0, 255))
                    # print(x_index, y_index)
                    # print(hand_landmarks.landmark[8].x, hand_landmarks.landmark[8].y)
                    finger_dis = 1000 * depth_frame.get_distance(x_index, y_index)
                    ang_x = math.radians((x_index - W / 2) / (W / 2) * (69 / 2))
                    ang_y = math.radians((y_index - H / 2) / (H / 2) * (42 / 2))
                    x_dis = math.tan(ang_x) * finger_dis
                    y_dis = math.tan(ang_y) * finger_dis
                    print('xyz coordinate: ', x_dis, y_dis, finger_dis)

            # Flip the image horizontally for a selfie-view display.
            cv2.imshow('MediaPipe Hands', cv2.flip(color_image, 1))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()