'''
File: geometry_viewer.py
Project: example
Created Date: 21/10/2022
Author: Shun Suzuki
-----
Last Modified: 21/10/2022
Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
-----
Copyright (c) 2022 Shun Suzuki. All rights reserved.

'''


from pyautd3 import Geometry, Controller, DEVICE_WIDTH, DEVICE_HEIGHT
from pyautd3.extra import GeometryViewer
from math import pi, cos

if __name__ == '__main__':

    W_cos = cos(pi/12) * DEVICE_WIDTH
    
    geometry = Geometry.Builder()\
        .add_device([W_cos - (DEVICE_WIDTH - W_cos), DEVICE_HEIGHT - 10, 0.], [pi, pi/12, 0.])\
        .add_device([W_cos - (DEVICE_WIDTH - W_cos),  - 10, 0.], [pi, pi/12, 0.])\
        .add_device([-W_cos + (DEVICE_WIDTH - W_cos), 0., 0.], [0., pi/12, 0.])\
        .add_device([-W_cos + (DEVICE_WIDTH - W_cos), -DEVICE_HEIGHT, 0.], [0., pi/12, 0.])\
        .build()
    
    GeometryViewer().window_size(800, 600).vsync(True).view(geometry)
