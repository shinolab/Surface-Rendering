'''
Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
Date: 2022-11-03 01:54:44
LastEditors: Mingxin Zhang
LastEditTime: 2022-12-08 17:48:45
Copyright (c) 2022 by Mingxin Zhang, All Rights Reserved. 
'''

from pyautd3 import Controller, SilencerConfig
from pyautd3.stm import FocusSTM
from pyautd3.modulation import Wav
import numpy as np

import os, sys
os.chdir(sys.path[0])

def surface_sound(autd: Controller, wavfile):
    config = SilencerConfig.none()
    autd.send(config)

    stm = FocusSTM(autd.sound_speed)
    radius = 4.0
    size = 100
    center = autd.geometry.center + np.array([0., 0., 150.])
    for i in range(size):
        theta = 2.0 * np.pi * i / size
        p = radius * np.array([np.cos(theta), np.sin(theta), 0])
        stm.add(center + p)

    div_ratio = int(163.84 * 1e6 / 44100)

    m = Wav('surface_sound_record/preprocessed/' + wavfile, div_ratio)

    stm.frequency = 6.0

    autd.send(m, stm)