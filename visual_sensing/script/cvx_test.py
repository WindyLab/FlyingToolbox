import cvxpy as cp
import numpy as np
import pdb
import random


from scipy.spatial.transform import Rotation as R

np.random.seed(1)
n = 16 ## number of observation
#p_base = np.random.randn(3, n)
p_base = \
[
[0,0,0],
[-0.05,0,0],
[-0.05,-0.05,0],
[0.05,0.05,0],

[0.05,   0.05,   -0.2],
[0.095,   0.02,  -0.2],
[0.11,-0.05,-0.2],
[-0.1,    0.05,   -0.2],

[-0.1,0,-0.3],
[-0.1,-0.11,-0.3],
[-0.09, 0.1, -0.3],
[0.09, 0.1, -0.3],

[0.1,0.1,-0.1],
[-0.1,0.1,-0.1],
[-0.1,-0.1,-0.1],
[0.1,-0.1,-0.1]
]
p_base = np.array(p_base)
p_vicon = \
[
[0.3167, -1.3337, 0.5392],
[0.3081,-1.370, 0.5591],
[0.2651,-1.3881,0.5421],
[0.3661,-1.2845, 0.5289],

[0.3675,-1.2837, 0.4876],
[0.3362,-1.2536, 0.4833],
[0.2641,-1.2526, 0.4861],
[0.3657,-1.4119, 0.4791],

[0.3155,-1.4080, 0.3914],
[0.2577,-1.3717, 0.3877],
[0.3841,-1.4056, 0.3934],
[0.3811,-1.2564, 0.3910],

[0.3975,-1.2563, 0.5316],
[0.3959,-1.4056, 0.5304],
[0.2349,-1.4145, 0.5429],
[0.2347,-1.2590, 0.5389]

]

p_vicon = np.array(p_vicon)
p_vicon = np.transpose(p_vicon)
p_base = np.transpose(p_base)
p_base[2,:]  = p_base[2,:] * (-1)
print(np.shape(p_vicon))

c_opt = cp.Variable((3, 3))
t_opt = cp.Variable((3, 1))

constraints = []
temp = c_opt @ p_base + t_opt @ np.ones((1, n), dtype=np.double)

obj = cp.Minimize(
    cp.sum(cp.norm(p_vicon - temp, axis=1)))
prob = cp.Problem(obj, constraints)
prob.solve()
print("status:", prob.status)
print(c_opt.value)
print(t_opt.value)
print(type(c_opt.value))
r = R.from_matrix(c_opt.value)
print(r.as_euler("zyx",degrees=True))