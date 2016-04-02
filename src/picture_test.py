#!/usr/bin/env python
import subprocess

subprocess.call("rosrun image_view image_saver image:=/ardrone/front/image_raw", shell=True)
# i = 0
#
# while i < 100:
#     i += 1

subprocess.call("exit 1", shell=True)
