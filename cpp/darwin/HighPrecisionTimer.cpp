/*
 * @Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
 * @Date: 2022-12-19 16:32:53
 * @LastEditors: Mingxin Zhang
 * @LastEditTime: 2022-12-19 18:50:07
 * Copyright (c) 2022 by Mingxin Zhang, All Rights Reserved. 
 */

#include <iostream>
#include <sys/time.h>

class HighPrecisionTimer
{
    public:
        float HighPrecisionSleep(float deltaTime);
};

float HighPrecisionTimer::HighPrecisionSleep(float deltaTime)
{
    timeval start, end;
    auto elapsedTime = 0.0;
    gettimeofday(&start, NULL);
    while (elapsedTime < deltaTime * 1e6) {
        gettimeofday(&end, NULL);
        elapsedTime = (double)(end.tv_usec - start.tv_usec);
    }
    return elapsedTime;
}

extern "C" {
    HighPrecisionTimer Timer;
    float HighPrecisionSleep(float deltaTime)
    {
        return Timer.HighPrecisionSleep(deltaTime);
    }
}