'''
Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
Date: 2022-11-22 13:37:25
LastEditors: Mingxin Zhang
LastEditTime: 2022-12-08 17:49:08
Copyright (c) 2022 by Mingxin Zhang, All Rights Reserved. 
'''

from pyautd3.link import SOEM
from pyautd3.link import Simulator
from pyautd3.gain import Focus
from pyautd3 import Controller, SilencerConfig, Clear, Synchronize, Stop
from pyautd3.stm import GainSTM
from pyautd3.modulation import Static
import numpy as np
from pyautd3.modulation import Sine


def stm_gain(autd: Controller):
    config = SilencerConfig.none()
    autd.send(config)
    stm = GainSTM(autd)
    radius = 1.0
    # step = 0.2
    # size = 50 * 2 * np.pi * radius // step

    center = autd.geometry.center + np.array([0., 0., 150.])
    for i in range(1000):
        radius += 0.005
        # theta = step / radius
        theta = 50 * 2 * np.pi * i / 1000
        p = radius * np.array([np.cos(theta), np.sin(theta), 0])
        f = Focus(center + p)
        stm.add(f)

    m = Static(1.0)
    stm.frequency = 0.4
    autd.send(m, stm)

def run(autd: Controller):
    autd.send(Clear())
    autd.send(Synchronize())

    print('================================== Firmware information ====================================')
    firm_info_list = autd.firmware_info_list()
    for firm in firm_info_list:
        print(firm)
    print('============================================================================================')

    r_list = np.arange(5, 30.0, 0.5)
    stm_gain(autd)
    _ = input()
    print('finish.')
    autd.send(Stop())
    autd.dispose()


if __name__ == '__main__':
    autd = Controller()

    autd.geometry.add_device([0., 0., 0.], [0., 0., 0.])

    if_use_simulator = input('If use simulator? [y: simulator] or [n: AUTD]: ')

    # if if_use_simulator == 'y':
    #     print('Use simulator')
    #     link = Simulator().port(50632).build()
    # elif if_use_simulator == 'n':
    #     print('Use AUTD device')
    #     link = SOEM().high_precision(True).build()
    # else:
    #     exit()

    link = SOEM().high_precision(True).build()

    if not autd.open(link):
        print(Controller.last_error())
        exit()

    autd.check_trials = 50

    run(autd)