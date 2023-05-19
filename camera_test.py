'''
Author: Mingxin Zhang m.zhang@hapis.u-tokyo.ac.jp
Date: 2023-04-05 17:40:31
LastEditors: Mingxin Zhang
LastEditTime: 2023-05-19 15:34:15
Copyright (c) 2023 by ${git_name}, All Rights Reserved. 
'''
import pyrealsense2 as rs
import numpy as np
import cv2
import math

# Initialization
pipeline = rs.pipeline()
config = rs.config()

W = 640
H = 480

# Decide resolutions for both depth and rgb streaming
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)

# Start streaming
profile = pipeline.start(config)

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

        # Flip the image horizontally for a selfie-view display.
        filter = rs.threshold_filter(min_dist=0, max_dist=0.23)
        depth_frame = filter.process(depth_frame)
        depth_img = np.asanyarray(depth_frame.get_data())
        depth_img = depth_img[int(H/2)-50:int(H/2)+50, int(W/2)-50:int(W/2)+50]
        
        mass_x, mass_y = np.where(depth_img > 0)
        if mass_x.size == 0 or mass_y.size == 0:
            continue

        # mass_x and mass_y are the list of x indices and y indices of mass pixels
        cent_x = int(np.average(mass_x))
        cent_y = int(np.average(mass_y))
        height = depth_img[cent_x, cent_y]

        x_index = W/2 - 50 + cent_x
        y_index = H/2 - 50 + cent_y

        # depth fov of D435i: 87° x 58°
        ang_x = math.radians((x_index - W / 2) / (W / 2) * (87 / 2))
        ang_y = math.radians((y_index - H / 2) / (H / 2) * (58 / 2))
        x_dis = math.tan(ang_x) * height
        y_dis = math.tan(ang_y) * height

        print(x_dis, y_dis, depth_img[cent_x, cent_y])
        
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