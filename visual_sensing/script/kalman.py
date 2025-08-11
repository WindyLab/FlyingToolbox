import numpy as np
import matplotlib.pyplot as plt
import scipy.io as scio
Q = np.diag([
    0.00001,  # variance of state px
    0.00001,  # variance of state py
    0.00001,  # variance of state pz
    0.0001,  # variance of state vx
    0.0001,  # variance of state vy
    0.0001  # variance of state  vz
]) ** 2  # predict state covariance

R = np.diag([
    0.0001,
    0.0001,
    0.0001
]) ** 2 

def motion_model(x):
    dt = 0.02
    F = np.eye(6,6)
    F[0,3] = dt
    F[1,4] = dt
    F[2,5] = dt
    x = F @ x
    return x,F

def observation_model(sPre):
    I  = np.eye(3)
    z = np.zeros((3,3))
    C = np.hstack((I,z))
    p_ob = C @ sPre
    return p_ob,C

def kf_estimation(sEst, PEst, y):
    #  Predict
    sPred,F = motion_model(sEst)
    PPred = F @ PEst @ F.T + Q
    fPred,C = observation_model(sPred)
    innov = y - fPred
    S = C @ PPred @ C.T + R
    A = np.linalg.inv(S)
    K = PPred @ C.T @ A
    sEst = sPred + K @ innov
    PEst = (np.eye(len(sEst)) - K @ C) @ PPred

    return sEst, PEst

import pdb

def check_cpp_res():
    pos_res = []
    path = 'data_out.txt'
    with open(path, 'r') as f:
        lines = f.read().splitlines()
        for l in lines:
            res = list(map(float, l.split(' ')))
            pos_res.append(res)
  #  print(pos_res)
    return pos_res



if __name__ == '__main__':
    pose_cpp_res = check_cpp_res()
    file_name = '/home/likai/Downloads/data_vision.mat'
    data = scio.loadmat(file_name)
    # print(data.keys())
    # print(data['pos_object_relative_x'])
    x = data['pos_object_relative_x']
    y = data['pos_object_relative_y']
    z = data['pos_object_relative_z']
    data_len = len(x)
    assert len(x) == len(y) and len(y) == len(z)
    t = np.array(x)[:,0]
    x = np.array(x)[:,1]
    y = np.array(y)[:,1]
    z = np.array(z)[:,1]

    pos = [x,y,z]
    #print(np.shape(pos))
   # print(pos)
    L = list(map(list, zip(*pos)))
    X = list(map(list, zip(*pose_cpp_res)))


    with open('data.txt', 'w') as fp:
        for item in L:
            print(item)
            item = str(item).strip('[').strip(']').replace(',','').replace('\'','')+'\n'
        #    item = str(item).strip('[').strip(']').replace('\'','')+'\n'
            fp.write(item)
   # exit(0)

    pos = np.transpose(pos)
    print(np.shape(pos))
    

    #print(cov_state)
    pos_init = [pos[0,0],pos[0,1],pos[0,2],0,0,0]
    cov_state = np.eye(6,6)
    for k in range(0,6):
        cov_state[k][k] = 0.1

    PEst = cov_state
    SEst = np.array(pos_init)
   # pdb.set_trace()
    states = SEst
    P = PEst
    num_iter = 830
    for num in range(0,num_iter-1):
        obervations = pos[num,:]
        SEst,PEst = kf_estimation(SEst,PEst,obervations)
        states = np.vstack((states,SEst))
        P = np.vstack((P,PEst))
        
    plt.figure(0)
    plt.plot(pos[:,2])
    plt.plot(states[:,2])
    plt.plot(X[2])
    plt.title("z")
    
    plt.figure(1)
    plt.plot(pos[:,0])
    plt.plot(states[:,0])
    plt.plot(X[0])

    plt.title("x")
    
    plt.figure(2)
    plt.plot(pos[:,1])
    plt.plot(states[:,1])
    plt.plot(X[1])

    plt.title("y")

    ################velocity#####################
    plt.figure(3)
    plt.plot(states[:,3])
    plt.title("vx")
    
    plt.figure(4)
    plt.plot(states[:,4])
    plt.title("vy")

    plt.figure(5)
    plt.plot(states[:,5])
    plt.title("vz")

    plt.show()