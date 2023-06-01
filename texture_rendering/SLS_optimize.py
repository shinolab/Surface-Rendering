'''
Author: Mingxin Zhang m.zhang@hapis.u-tokyo.ac.jp
Date: 2023-04-12 01:47:50
LastEditors: Mingxin Zhang
LastEditTime: 2023-04-27 15:45:08
Copyright (c) 2023 by Mingxin Zhang, All Rights Reserved. 
'''

import pySequentialLineSearch
import numpy as np
import sys

# A dummy implementation of slider manipulation
def ask_human_for_slider_manipulation(optimizer):
    t_max = 0.0
    f_max = -sys.float_info.max

    for i in range(1000):
        
        t = float(i) / 999.0

        optmized_para = optimizer.calc_point_from_slider_position(t)

        # ...choose the best slider position

        if f_max is None or f_max < f:
            f_max = f
            t_max = t

    return t_max


def main():
    # Optimized parameters: f_STM, radius, f_wave, amplitude
    optimizer = pySequentialLineSearch.SequentialLineSearchOptimizer(num_dims=4)

    optimizer.set_hyperparams(kernel_signal_var=0.50,
                              kernel_length_scale=0.10,
                              kernel_hyperparams_prior_var=0.10)
    
    optimizer.set_gaussian_process_upper_confidence_bound_hyperparam(5.)

    for i in range(20):
        # slider_ends = optimizer.get_slider_ends()
        slider_position = ask_human_for_slider_manipulation(optimizer)
        optimizer.submit_feedback_data(slider_position)

        optimized_vector = optimizer.get_maximizer()
        # denormalization

        optimized_vector = optimized_vector.to(device)
        decoded_spec = decoder(optimized_vector)

        mse = nn.MSELoss()
        loss = mse(decoded_spec, target_spec).item()
        print("[#iters = " + str(i + 1) + "], slider_position: " + str(slider_position) + ", loss: " + str(loss))


if __name__ == '__main__':
    main()

