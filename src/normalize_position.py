#! /usr/bin/env python
'''
Normalizes the tag. Without this, from the perspective of the QC, the tag will
move when either pitching or rolling. By using some trig, the tag will appear
to be stationary when pitching and rolling.

Much of this information was sourced from:

http://ludep.com/tracking-algorithm-condsidering-the-inclination-of-the-drone

When using this, take care to set the origin of the image to be the center of
the image (i.e. both x and y of the image should be [x - 500px, y + 500px])

NOTE:
    Inputs:
    position: [-500px, +500px]
    tilt: the tilt of the bottom camera
    altitude: the altitude of the camera

    Output:
    position_actual: the calculated distance in mm

    Formulas used:
    cvt_alt         = altitude * tan(32 degrees/500px) / 1000
    position_m      = position * cvt_alt
    position_actual = position_mm / cos(tilt) - altitude * tan(tilt)

Created by: Josh Saunders and Amando Aranda
Date Created: 5/5/2016

Modified by:
Date Modified:
'''
import math

alpha   = math.radians(32)

def real_position(position, altitude, tilt):
    cvt_alt    = altitude * math.tan(alpha) / 500
    position_m = position * cvt_alt
    # position_actual
    return position_m / math.cos(tilt) - altitude * math.tan(tilt)
