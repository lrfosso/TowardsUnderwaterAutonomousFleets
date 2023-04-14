import os as os
import numpy as np
import subprocess


def sin(x):
    return 1.5*np.sin(x)


i = 0
while(True):
    os.system("gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: {}, y:{}, z:{}'".format(0, 0, sin(i)))
    print(sin(i))
    i += 0.5
