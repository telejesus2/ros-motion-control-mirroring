import numpy as np
import matplotlib.pyplot as plt
SHOULDER_OFFSET = 0.1198
SHOULDER_HEIGHT = 0.1519

a = np.load("./angles.npy")
ar = np.load("./angles_robot.npy")
print(len(a))
print(len(ar))

e = np.load("./tmp/ee.npy")
er = np.load("./tmp/ee_robot.npy")
ee_t  = np.load("./tmp/ee_tmp.npy")
ee2_t  = np.load("./tmp/ee2_tmp.npy")
print(len(e))
print(len(er))
print(len(ee_t))


n1 = 0
n2 = len(e)
xt = [ee_t[i,0] - np.sin(er[i,3]) * SHOULDER_OFFSET for i in range(n1,n2)]
zt = [ee_t[i,1] for i in range(n1,n2)]
x2t = [ee2_t[i,0] - np.sin(er[i,3]) * SHOULDER_OFFSET for i in range(n1,n2)]
z2t = [ee2_t[i,1] - SHOULDER_HEIGHT for i in range(n1,n2)]
x   = [e[i,0] - np.sin(er[i,3]) * SHOULDER_OFFSET for i in range(n1,n2)]
#x   = [e[i,0] for i in range(n1,n2)]
x_r = [er[i,0] for i in range(n1,n2)]
y   = [e[i,2] + np.sin(er[i,3]) * SHOULDER_OFFSET for i in range(n1,n2)]
y_r = [e[i,1] for i in range(n1,n2)]
z   = [e[i,1] for i in range(n1,n2)]
z_r = [e[i,2] for i in range(n1,n2)]
frames = [i for i in range(n1,n2)]


plt.plot(x2t, z2t, 'b', x_r, z_r, 'g')
plt.show()

base 		= [a[i,0] - np.pi for i in range(len(a))]
base_r 		= [ar[i,0] for i in range(len(a))]
shoulder	= [-a[i,1] - np.pi for i in range(len(a))]
shoulder_r	= [ar[i,1] for i in range(len(a))]
elbow		= [-a[i,2] for i in range(len(a))]
elbow_r		= [ar[i,2] for i in range(len(a))]
wrist1		= [-a[i,3] for i in range(len(a))]
wrist1_r	= [ar[i,3] for i in range(len(a))]
wrist3		= [a[i,4] + np.pi for i in range(len(a))]
wrist3_r	= [ar[i,5] for i in range(len(a))]
frames = [i for i in range(len(a))]


plt.plot(frames, elbow, 'r--', frames, elbow_r, 'bs')
#plt.show()
