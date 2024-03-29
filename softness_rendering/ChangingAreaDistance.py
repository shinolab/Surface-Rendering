'''
Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
Date: 2022-11-22 22:42:58
LastEditors: Mingxin Zhang
LastEditTime: 2023-08-16 16:52:06
Copyright (c) 2022 by Mingxin Zhang, All Rights Reserved. 
'''

from pyautd3.link import SOEM, OnLostFunc
from pyautd3.link import Simulator
from pyautd3.gain import Focus
from pyautd3 import Controller, AUTD3, SilencerConfig, Synchronize, Stop, DEVICE_WIDTH, DEVICE_HEIGHT
from pyautd3.modulation import Static, Sine
import numpy as np
import ctypes
import platform
import os
import pyrealsense2 as rs
import cv2
import math
from math import pi
from multiprocessing import Process, Pipe
import time
from datetime import timedelta

# use cpp to get high precision sleep time
dll = ctypes.cdll.LoadLibrary
libc = dll(os.path.dirname(__file__) + '/../cpp/' + platform.system().lower() + '/HighPrecisionTimer.so') 

def on_lost(msg: ctypes.c_char_p):
    print(msg.decode('utf-8'), end="")
    os._exit(-1)

def run(subscriber, publisher):
    W_cos = math.cos(math.pi/12) * DEVICE_WIDTH

    on_lost_func = OnLostFunc(on_lost)

    autd = (
        Controller.builder()
        .add_device(AUTD3.from_euler_zyz([W_cos - (DEVICE_WIDTH - W_cos), DEVICE_HEIGHT - 10 + 12.5, 0.], [pi, pi/12, 0.]))
        .add_device(AUTD3.from_euler_zyz([W_cos - (DEVICE_WIDTH - W_cos), -10 - 12.5, 0.], [pi, pi/12, 0.]))
        .add_device(AUTD3.from_euler_zyz([-W_cos + (DEVICE_WIDTH - W_cos),  12.5, 0.], [0., pi/12, 0.]))
        .add_device(AUTD3.from_euler_zyz([-W_cos + (DEVICE_WIDTH - W_cos), -DEVICE_HEIGHT - 12.5, 0.], [0., pi/12, 0.]))
        .advanced_mode()
        # .open_with(Simulator(8080))
        .open_with(SOEM().with_on_lost(on_lost_func))
        # .open_with(TwinCAT())
    )

    autd.send(Synchronize())

    print('================================== Firmware information ====================================')
    firm_info_list = autd.firmware_info_list()
    for firm in firm_info_list:
        print(firm)
    print('============================================================================================')

    center = autd.geometry.center + np.array([0., 0., 0.])

    m = Static()
    # m = Sine(108)

    radius = 3.0    # radius of STM
    zero_radius = 1.0
    zero_stm_f = 3.0
    zero_height = 230.
    # step = 0.2      # step length (mm)
    time_step = 0.0015
    stm_f = 1.0     # frequency of STM
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
            autd.send((m, f), timedelta(microseconds=0))

            # ... change the radius and height here
            if publisher.poll():
                coordinate = publisher.recv()
                x = coordinate[0]
                y = coordinate[1]
                # height of D435i: 25mm
                # D435i depth start point: -4.2mm
                height = coordinate[2] - 9 - 4.2

            delta_height = zero_height - height
            radius = zero_radius + min(30, max(delta_height, 0)) * 0.16
            stm_f = zero_stm_f + min(7, max(delta_height, 0) * 0.2)

            theta += 2 * np.pi * stm_f * time_step

            # theta += step / radius
            # size = 2 * np.pi * radius // step   # recalculate the number of points in a round
            # time_step = (1 / stm_f) / size  # recalculate time step
            libc.HighPrecisionSleep(ctypes.c_float(time_step))  # cpp sleep function

    except KeyboardInterrupt:
        pass

    print('finish.')
    autd.send(Stop())
    publisher.close()

    autd.close()


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
            height = depth_img[cent_x, cent_y]

            # depth fov of D435i: 87° x 58°
            # rgb fov of D435i: 69° x 42°
            ang_x = math.radians((cent_x - 50) / (W / 2) * (87 / 2))
            ang_y = math.radians((cent_y - 50) / (H / 2) * (58 / 2))
            x_dis = math.tan(ang_x) * height
            y_dis = math.tan(ang_y) * height

            delta_height = 230. - height
            radius = 1. + min(30, max(delta_height, 0)) * 0.16

            ang_r = math.atan(radius / height)
            print(ang_r)
            r_pixel = H * ang_r / math.radians(58 / 2)
            print(r_pixel)

            print('X:', x_dis, 'Y:', y_dis, 'Z:', height)
            subscriber.send([y_dis, x_dis, height])
            
            # put text and highlight the center
            cv2.circle(depth_img, (cent_y, cent_x), int(r_pixel), (255, 255, 255), -1)

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