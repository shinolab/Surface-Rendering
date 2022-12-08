'''
Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
Date: 2022-11-22 22:42:58
LastEditors: Mingxin Zhang
LastEditTime: 2022-12-08 17:48:30
Copyright (c) 2022 by Mingxin Zhang, All Rights Reserved. 
'''

from pyautd3.link import SOEM
from pyautd3.link import Simulator
from pyautd3.gain import Focus
from pyautd3 import Controller, SilencerConfig, Clear, Synchronize, Stop
from pyautd3.modulation import Static
import numpy as np
import time

def run(autd: Controller):
    autd.send(Clear())
    autd.send(Synchronize())

    print('================================== Firmware information ====================================')
    firm_info_list = autd.firmware_info_list()
    for firm in firm_info_list:
        print(firm)
    print('============================================================================================')

    # r_list = np.arange(1.5, 4.0, 0.1)

    center = autd.geometry.center + np.array([0., 0., 150.])
    m = Static(1.0)
    radius = 1.0    # radius of STM
    step = 0.2      # step length (mm)
    stm_f = 20.0     # frequency of STM
    theta = 0
    config = SilencerConfig.none()
    autd.send(config)

    print('press ctrl+c to finish...')

    try:
        while True:
            # tic = time.time()
            # update the focus information
            p = radius * np.array([np.cos(theta), np.sin(theta), 0])
            f = Focus(center + p)
            autd.send(m, f)

            # ... change the radius and height here
            if radius < 6.0:
                radius += 0.005
            else:
                radius = 1.0

            theta += step / radius
            size = 2 * np.pi * radius // step   # recalculate the number of points in a round
            time_step = (1 / stm_f) / size  # recalculate time step
            # toc = time.time()
            # print(toc-tic)
            time.sleep(time_step)

    except KeyboardInterrupt:
        pass

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