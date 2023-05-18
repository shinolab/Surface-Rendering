'''
Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
Date: 2022-11-22 22:42:58
LastEditors: Mingxin Zhang
LastEditTime: 2023-05-18 10:33:01
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
    autd.geometry.add_device([-DEVICE_WIDTH / 2, DEVICE_HEIGHT / 2 + 12.5, 0.], [0., 0., 0.])
    autd.geometry.add_device([DEVICE_WIDTH / 2, DEVICE_HEIGHT / 2 + 12.5, 0.], [0., 0., 0.])
    autd.geometry.add_device([DEVICE_WIDTH / 2, -DEVICE_HEIGHT / 2 - 12.5, 0.], [0., 0., 0.])
    autd.geometry.add_device([-DEVICE_WIDTH / 2, -DEVICE_HEIGHT / 2 - 12.5, 0.], [0., 0., 0.])

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
    zero_radius = 1.0
    step = 0.2      # step length (mm)
    stm_f = 6.0     # frequency of STM
    theta = 0
    height = 200.   # init x, y, height
    x = 0.
    y = 0.
    zero_height = 200.
    config = SilencerConfig()
    autd.send(config)

    print('press ctrl+c to finish...')

    subscriber.close()

    try:
        while True:
            # update the focus information
            p = radius * np.array([np.cos(theta), np.sin(theta), 0])
            p += np.array([x, y, height])
            f = Focus(center + p)
            autd.send(m, f)

            # ... change the radius and height here
            if publisher.poll():
                height = publisher.recv()
                # height of D435i: 25mm
                # D435i depth start point: -4.2mm
                height = height - 4 - 4.2
            
            delta_height = zero_height - height
            radius = zero_radius + min(20, max(delta_height, 0)) * 0.25

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
    # Initialization
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    # Decide resolutions for both depth and rgb streaming
    config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)

    publisher.close()

    try:
        while True:
            frames = pipeline.wait_for_frames()

            depth_frame = frames.get_depth_frame() # depth_frame is a 640x480 depth image

            # Validate that both frames are valid
            if not depth_frame:
                continue

            W = depth_frame.get_width()
            H = depth_frame.get_height()

            # fixed center point
            finger_dis = 1000 * depth_frame.get_distance(math.floor(0.5 * W), math.floor(0.5 * H))  # meter to mm
            
            # rgb fov of D435i: 69° x 42°
            print('height: ', finger_dis)
            subscriber.send(finger_dis)

    finally:
        # Stop streaming
        pipeline.stop()
        subscriber.close()


if __name__ == '__main__':
    subscriber, publisher = Pipe()

    p_main = Process(target=run, args=(subscriber, publisher))
    p_main.start()

    get_finger_distance(subscriber, publisher)

    publisher.close()
    subscriber.close()

    p_main.join()