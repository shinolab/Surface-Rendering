'''
Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
Date: 2022-11-10 16:50:39
LastEditors: Mingxin Zhang
LastEditTime: 2022-12-08 17:48:20
Copyright (c) 2022 by Mingxin Zhang, All Rights Reserved. 
'''

from pyautd3.link import SOEM
from pyautd3.link import Simulator
from pyautd3 import Controller, SilencerConfig, Clear, Synchronize, Stop
from pyautd3.stm import FocusSTM
from pyautd3.modulation import Sine, Static
import numpy as np


def sine(autd: Controller):
    config = SilencerConfig.none()
    autd.send(config)

    stm = FocusSTM(autd.sound_speed)
    radius = 4.0
    step = 0.2
    size = 2 * np.pi * radius // step
    center = autd.geometry.center + np.array([0., 0., 150.])
    for i in range(int(size)):
        theta = 2.0 * np.pi * i / size
        p = radius * np.array([np.cos(theta), np.sin(theta), 0])
        stm.add(center + p)

    # m = Sine(freq=108, amp=1, offset=0.5)
    m = Static(1.0)
    stm.frequency = 6.0
    autd.send(m, stm)

def run(autd: Controller):
    autd.send(Clear())
    autd.send(Synchronize())

    print('================================== Firmware information ====================================')
    firm_info_list = autd.firmware_info_list()
    for firm in firm_info_list:
        print(firm)
    print('============================================================================================')

    sine(autd)
    print('press enter to finish...')
    _ = input()
    print('finish.')
    autd.send(Stop())

    autd.dispose()


if __name__ == '__main__':
    autd = Controller()

    autd.geometry.add_device([0., 0., 0.], [0., 0., 0.])

    if_use_simulator = input('If use simulator? [y: simulator] or [n: AUTD]: ')

    if if_use_simulator == 'y':
        print('Use simulator')
        link = Simulator().port(50632).build()
    elif if_use_simulator == 'n':
        print('Use AUTD device')
        link = SOEM().high_precision(True).build()
    else:
        exit()

    if not autd.open(link):
        print(Controller.last_error())
        exit()

    autd.check_trials = 50

    run(autd)