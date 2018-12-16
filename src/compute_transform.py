#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np
from numpy.linalg import inv
from tf import transformations

#np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

# wide stereo to head plate
translation = transformations.translation_matrix([0.045, -0.060, 0.060])
rotation = transformations.quaternion_matrix([-0.499, 0.509, -0.501, 0.491])
t1 = inv(np.dot(translation, rotation))

print("\n Translation 1: \n", translation)
print("\n Rotation 1: \n", rotation)
print("\n Transform 1: \n", t1)

# asus to wide stereo
# good
t2 = np.matrix([
	[0.9996672531355477, 0.009193065957438276, 0.024101256123509184, 0.0840326747161547],
	[-0.009454262743529835, 0.9998975647280123, 0.010746020982991988, 0.07728556090590073],
	[-0.023999998425106264, -0.01097030488604532, 0.9996517656095557, 0.010264042110660615],
	[0.0, 0.0, 0.0, 1.0]
])

print("\n Transform 2: \n", t2)

t3 = inv(np.dot(t2, t1))
print("\n Transform 3: \n", t3)

roll, pitch, yaw = transformations.euler_from_matrix(t3, 'sxyz')
print("rpy: ", roll, pitch, yaw)