'''
Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
Date: 2022-11-22 22:42:58
LastEditors: Mingxin Zhang
LastEditTime: 2023-04-07 14:02:52
Copyright (c) 2022 by Mingxin Zhang, All Rights Reserved. 
'''

from pyautd3.link import SOEM
from pyautd3.link import Simulator
from pyautd3.gain import Focus
from pyautd3 import Controller, SilencerConfig, Clear, Synchronize, Stop, DEVICE_WIDTH, DEVICE_HEIGHT
from pyautd3.modulation import Static, Sine
import numpy as np
import ctypes
import platform
import os
import pyrealsense2 as rs
import cv2
import mediapipe as mp
import math
from multiprocessing import Process, Pipe
import time

# use cpp to get high precision sleep time
dll = ctypes.cdll.LoadLibrary
libc = dll(os.path.dirname(__file__) + '/../cpp/' + platform.system().lower() + '/HighPrecisionTimer.so') 


def run(subscriber, publisher):
    autd = Controller()

    # Multiple AUTD
    # The arrangement of the AUTDs:
    # 1 → 2
    #     ↓
    # 4 ← 3
    # (See from the upside)
    autd.geometry.add_device([-DEVICE_WIDTH / 2, DEVICE_HEIGHT / 2, 0.], [0., 0., 0.])
    autd.geometry.add_device([DEVICE_WIDTH / 2, DEVICE_HEIGHT / 2, 0.], [0., 0., 0.])
    autd.geometry.add_device([DEVICE_WIDTH / 2, -DEVICE_HEIGHT / 2, 0.], [0., 0., 0.])
    autd.geometry.add_device([-DEVICE_WIDTH / 2, -DEVICE_HEIGHT / 2, 0.], [0., 0., 0.])

    # link = Simulator().build()
    link = SOEM().high_precision(True).build()

    if not autd.open(link):
        print('Failed to open Controller')
        exit()

    autd.check_trials = 50

    autd.send(Clear())
    autd.send(Synchronize())

    print('================================== Firmware information ====================================')
    firm_info_list = autd.firmware_info_list()
    for firm in firm_info_list:
        print(firm)
    print('============================================================================================')

    center = autd.geometry.center + np.array([0., 0., 0.])

    m = Static(1.0)
    # m = Sine(150)

    radius = 1.0    # radius of STM
    step = 0.2      # step length (mm)
    stm_f = 6.0     # frequency of STM
    theta = 0
    height = 150.   # init height
    config = SilencerConfig.none()
    autd.send(config)

    print('press ctrl+c to finish...')

    subscriber.close()

    try:
        while True:
            # update the focus information
            p = radius * np.array([np.cos(theta), np.sin(theta), height])
            f = Focus(center + p)
            autd.send(m, f)

            # ... change the radius and height here
            if publisher.poll():
                height = publisher.recv()

            theta += step / radius
            size = 2 * np.pi * radius // step   # recalculate the number of points in a round
            time_step = (1 / stm_f) / size  # recalculate time step
            libc.HighPrecisionSleep(ctypes.c_float(time_step))  # cpp sleep function

    except KeyboardInterrupt:
        pass

    print('finish.')
    autd.send(Stop())
    publisher.close()

    autd.dispose()


def get_finger_distance(subscriber, publisher):
    # Initialization hand tracking
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

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
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)

    # Remove the background of objects more than clipping_distance_in_meters (user's set) meters away
    clipping_distance_in_meters = 1 #1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    publisher.close()

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
                        finger_dis = depth_image[y_index][x_index]
                        print(finger_dis)
                        subscriber.send(finger_dis)

                # Flip the image horizontally for a selfie-view display.
                cv2.imshow('MediaPipe Hands', cv2.flip(color_image, 1))

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()
        subscriber.close()


if __name__ == '__main__':
    subscriber, publisher = Pipe()

    p_main = Process(target=run, args=(subscriber, publisher))
    p_main.start()

    get_finger_distance(subscriber, publisher)

    publisher.close()
    subscriber.close()

    p_main.join()