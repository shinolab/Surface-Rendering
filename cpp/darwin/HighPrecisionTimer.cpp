/*
 * @Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
 * @Date: 2022-12-19 16:32:53
 * @LastEditors: Mingxin Zhang
 * @LastEditTime: 2022-12-20 02:59:02
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
    double elapsedTime = 0.0;  // unit is us
    gettimeofday(&start, NULL);
    while (elapsedTime < deltaTime * 1e6) {
        gettimeofday(&end, NULL);
        elapsedTime = (double)(end.tv_usec - start.tv_usec);
        if (elapsedTime < 0) break;
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