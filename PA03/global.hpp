//
// Created by LEI XU on 4/9/19.
//

#ifndef RASTERIZER_GLOBAL_H
#define RASTERIZER_GLOBAL_H

typedef unsigned char u08;
#define MY_PI 3.1415926
#define TWO_PI (2.0* MY_PI)

inline float clamp01(float val)
{
    if(val > 1)
        val = 1;
    if(val < 0)
        val = 0;
    return val;
}

#endif //RASTERIZER_GLOBAL_H
