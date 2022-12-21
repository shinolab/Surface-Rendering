'''
Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
Date: 2022-11-03 14:21:15
LastEditors: Mingxin Zhang
LastEditTime: 2022-12-21 17:09:11
Copyright (c) 2022 by Mingxin Zhang, All Rights Reserved. 
'''

from pyautd3 import Controller, SilencerConfig, Clear, Synchronize, Stop, DEVICE_WIDTH, DEVICE_HEIGHT
from pyautd3.link import SOEM
from pyautd3.link import Simulator
import surface_sound

def run(autd: Controller):
    samples = [
        ("wallpaper.wav"),
        ("carpet.wav"),
        ("fiber.wav"),
        ("rough_paper.wav"),
        ("sheep_skin.wav"),
        ("textile.wav"),
        ("modulation.wav"),
        ("plus.wav"),
        ("acctest.wav")
    ]

    autd.send(Clear())
    autd.send(Synchronize())

    print('================================== Firmware information ====================================')
    firm_info_list = autd.firmware_info_list()
    for firm in firm_info_list:
        print(firm)
    print('============================================================================================')

    while True:
        for i, (name) in enumerate(samples):
            print(f'[{i}]: {name}')
        print('[Other]: finish')

        idx = input('choose number: ')
        idx = int(idx) if idx.isdigit() else None
        if idx is None or idx >= len(samples):
            break
        (wavfile) = samples[idx]
        surface_sound.surface_sound(autd, wavfile)

        print('press enter to finish...')

        _ = input()

        print('finish.')
        autd.send(Stop())

    autd.dispose()


if __name__ == '__main__':
    autd = Controller()

    # Multiple AUTD
    num_autd = input('Choose the number of using AUTD: ')
    if num_autd == '4':
        autd.geometry.add_device([-DEVICE_WIDTH / 2, DEVICE_HEIGHT / 2, 0.], [0., 0., 0.])
        autd.geometry.add_device([DEVICE_WIDTH / 2, -DEVICE_HEIGHT / 2, 0.], [0., 0., 0.])
        autd.geometry.add_device([DEVICE_WIDTH / 2, DEVICE_HEIGHT / 2, 0.], [0., 0., 0.])
        autd.geometry.add_device([-DEVICE_WIDTH / 2, -DEVICE_HEIGHT / 2, 0.], [0., 0., 0.])
    elif num_autd == '1':
        autd.geometry.add_device([0., 0., 0.], [0., 0., 0.])
    else:
        exit()

    if_use_simulator = input('If use simulator? [y: simulator] or [n: AUTD]: ')

    if if_use_simulator == 'y':
        print('Use simulator')
        link = Simulator().build()
    elif if_use_simulator == 'n':
        print('Use AUTD device')
        link = SOEM().high_precision(True).build()
    else:
        exit()

    if not autd.open(link):
        print('Failed to open Controller')
        exit()

    autd.check_trials = 50

    run(autd)