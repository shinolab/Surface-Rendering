<!--
 * @Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
 * @Date: 2022-12-08 22:29:47
 * @LastEditors: Mingxin Zhang
 * @LastEditTime: 2023-01-12 19:59:46
 * Copyright (c) 2022 by Mingxin Zhang, All Rights Reserved. 
-->

`spiral_demo.py` produces a spiral focus trajectory to obtain an STM with changing radius, so that the sensation of the changing contact area can be simulated. (But the step length of the STM is not controlled as a constant. Just a simple confirmatory pre-experiment.)

A more rigorous environment is designed in `changing_area_rending.py`. The step length can be controlled and the radius and the height can also be changed freely. 

Use `g++ -o HighPrecisionTimer.so -shared -fPIC HighPrecisionTimer.cpp` to generate the .so file.