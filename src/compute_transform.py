#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np
from numpy.linalg import inv
from tf import transformations

#np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

# wide stereo right to head plate
#translation = transformations.translation_matrix([0.045, -0.060, 0.060])
#rotation = transformations.quaternion_matrix([-0.499, 0.509, -0.501, 0.491])
# wide stereo left to head plate
translation = transformations.translation_matrix([0.047, 0.030, 0.060])
rotation = transformations.quaternion_matrix([-0.000, 0.008, -0.010, 1.000])

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

t2 = np.matrix([
	[0.9995826332818657, 0.014527740286575754, -0.02497006214362608, -0.01654742947479654],
	[-0.014979949927871977, 0.9997254126657851, -0.018019444230711428, -0.07696587547085777],
	[0.024701423874934677, 0.018385973795017625, 0.9995257853733236, -0.017223286968161083],
	[0.0, 0.0, 0.0, 1.0]
])

t2 = np.matrix([
	[0.9999110884039305, 0.011700207106007759, 0.006396908671091978, -0.016854031347300933],
	[-0.011584618920277644, 0.9997741447123525, -0.01781729971568538, -0.07781838788308285],
	[-0.006603929992187336, 0.017741609801907567, 0.9998207956380473, -0.017922450086872804],
	[0.0, 0.0, 0.0, 1.0]
])

print("\n Transform: \n", t2)
roll, pitch, yaw = transformations.euler_from_matrix(t2, 'sxyz')
print("rpy: ", roll, pitch, yaw)

t3 = np.matrix([
	[0.999710061450946, -0.02366983747992207, 0.004419482709557995, 0.02615824124694793],
	[0.023650600136777705, 0.9997107974204255, 0.004355529151260891, 0.0003489315316723845],
	[-0.004521299250907412, -0.004249742897083252, 0.9999807485838877, 6.4853233546170095e-06],
	[0.0, 0.0, 0.0, 1.0]
])

t3 = np.matrix([
	[0.9997200905725844, -0.023617363708013147, 0.0014002274910678242, 0.026772908344689685],
	[0.023612558588165532, 0.9997155575776493, 0.0033542531404084277, 0.00017208290679281878],
	[-0.0014790478233541943, -0.0033202512996628906, 0.9999933941526029, 0.00017569481446118827],
	[0.0, 0.0, 0.0, 1.0]
])


print("\n Transform: \n", inv(t3))
roll, pitch, yaw = transformations.euler_from_matrix(inv(t3), 'sxyz')
print("rpy: ", roll, pitch, yaw)

#print("\n Transform 2: \n", inv(t2))

#t3 = inv(np.dot(inv(t2), t1))
#print("\n Transform 3: \n", t3)

#roll, pitch, yaw = transformations.euler_from_matrix(t3, 'sxyz')
#print("rpy: ", roll, pitch, yaw)