'''
Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
Date: 2022-11-22 22:42:58
LastEditors: Mingxin Zhang
LastEditTime: 2023-05-19 20:50:05
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

    # m = Static(1.0)
    m = Sine(108)

    radius = 3.0    # radius of STM
    step = 0.2      # step length (mm)
    stm_f = 5.0     # frequency of STM
    theta = 0
    height = 230.   # init x, y, height
    x = 0.
    y = 0.
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
                coordinate = publisher.recv()
                x = coordinate[0]
                y = coordinate[1]
                # height of D435i: 25mm
                # D435i depth start point: -4.2mm
                height = coordinate[2] - 4 - 4.2

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

    # Decide resolutions for both depth and rgb streaming
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)

    publisher.close()

    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Get aligned frames
            depth_frame = frames.get_depth_frame() # depth_frame is a 640x480 depth image

            # Validate that both frames are valid
            if not depth_frame:
                continue

            W = depth_frame.get_width()
            H = depth_frame.get_height()
            
            # Set the contact area height to 23cm
            filter = rs.threshold_filter(min_dist=0, max_dist=0.23)
            depth_frame = filter.process(depth_frame)
            depth_img = np.asanyarray(depth_frame.get_data())
            # Set the detect range
            depth_img = depth_img[int(H/2)-50:int(H/2)+50, int(W/2)-50:int(W/2)+50]
            
            mass_x, mass_y = np.where(depth_img > 0)
            if mass_x.size == 0 or mass_y.size == 0:
                continue

            # mass_x and mass_y are the list of x indices and y indices of mass pixels
            cent_x = int(np.average(mass_x))
            cent_y = int(np.average(mass_y))
            print(cent_x, cent_y)
            height = depth_img[cent_x, cent_y]

            # depth fov of D435i: 87° x 58°
            # rgb fov of D435i: 69° x 42°
            ang_x = math.radians((cent_x - 50) / (W / 2) * (87 / 2))
            ang_y = math.radians((cent_y - 50) / (H / 2) * (58 / 2))
            x_dis = math.tan(ang_x) * height
            y_dis = math.tan(ang_y) * height

            print('X:', x_dis, 'Y:', y_dis, 'Z:', height)
            subscriber.send([y_dis, x_dis, height])
            
            # put text and highlight the center
            cv2.circle(depth_img, (cent_y, cent_x), 5, (255, 255, 255), -1)

            depth_img = cv2.applyColorMap(cv2.convertScaleAbs(depth_img), cv2.COLORMAP_JET)
            cv2.imshow('Detect Area', cv2.flip(depth_img, 1))

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