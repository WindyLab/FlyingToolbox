import numpy as np
import matplotlib.pyplot as plt

class tag:
    def __init__(self, center, width,ids):
        self.center = center 
        self.width = width
        self.co = {}
        self.ids = ids
    def get_cor(self):
        for i,p in enumerate(self.center):
            x0 = -self.width / 2 + p[0]
            y0 = self.width / 2 + p[1]
            
            x1 = self.width / 2 + p[0]
            y1 = self.width / 2 + p[1]
            
            x2 = self.width / 2 + p[0]
            y2 = -self.width / 2 + p[1]

            x3 = -self.width / 2 + p[0]
            y3 = -self.width / 2 + p[1]
            
            cor = [[x0,y0],[x1,y1],[x2,y2],[x3,y3]]
            self.co[self.ids[i]] = cor
        return self.co

if __name__ == "__main__":
    #####################
    center30 = [
        [0,0]
    ]
    ids30 = list(range(0,1))
    width30 = 30
    tag30 = tag(center30,width30,ids30)
    co30_target = tag30.get_cor()
    # print(co30_target)
    #######################
    center26 = [
        [141,  0],
        [0, -141],
        [-141, 0],
        [0,  141]
    ]
    width26 = 26
    ids26 = list(range(1,5))
    tag26 = tag(center26,width26,ids26)
    co26_target = tag26.get_cor()
    # print('\n')
    # print(co26_target)
    ##########################
    center28 = [
        [75, 168],
        [168, 75],
        [168, -75],
        [75, -168],
        [-75, -168],
        [-168, -75],
        [-168, 75],
        [-75, 168]
    ]
    width28 = 28
    ids28 = list(range(5,13))
    tag28 = tag(center28,width28,ids28)
    co28_target = tag28.get_cor()
    # print('\n')
    # print(co28_target)
    ##########################
    center22 = [
        [141,   141],
        [141,  -141],
        [-141,  141],
        [-141, -141]
    ]
    width22 = 22
    ids22 = list(range(13,17))
    tag22 = tag(center22,width22,ids22)
    co22_target = tag22.get_cor()
    # print('\n')
    # print(co22_target)
    #################################
    ids20_target = list(range(17,29))
    width20 = 20
    center20_target = [
        [-60,60], ##17
        [-20,60], ##18
        [20,60],   ##19
        [60,60], ##20
        [60,20],##21
        [60,-20],##22
        [60,-60],##23
        [20,-60],##24
        [-20,-60],##25
        [-60,-60],##26
        [-60,-20],##27
        [-60,20] ##28 
    ]
    tag_target20 = tag(center20_target,width20,ids20_target)
    co20_target = tag_target20.get_cor()
    # print('\n')
    # print(co20_target)
   # ax = plt.figure(figsize=(9,9))
    
    #plt.scatter(np.array(list(co20.values()))[:,:,0],np.array(list(co20.values()))[:,:,1])
    #plt.scatter(np.array(list(co26.values()))[:,:,0],np.array(list(co26.values()))[:,:,1])
    #plt.scatter(np.array(list(co29.values()))[:,:,0],np.array(list(co29.values()))[:,:,1])
    #plt.scatter(np.array(list(co20_target.values()))[:,:,0],np.array(list(co20_target.values()))[:,:,1])
    
   # plt.show()
    
    with open('pts3d_target.txt', 'w') as fp:
        for key in co30_target:
            fp.write('{')
            fp.write("\n  " + str(key) + ",\n  {\n")
            for i in range(0, 4):
                res = (co30_target[key][i][0], co30_target[key][i][1], 0)
                fp.write("    cv::Point3f " + str(res)+',\n')
            fp.write(' }\n')
            fp.write('},\n')
        for key in co26_target:
            fp.write('{')
            fp.write("\n  " + str(key) + ",\n  {\n")
            for i in range(0, 4):
                res = (co26_target[key][i][0], co26_target[key][i][1], 0)
                fp.write("    cv::Point3f " + str(res)+',\n')
            fp.write(' }\n')
            fp.write('},\n')
        for key in co28_target:
            fp.write('{')
            fp.write("\n  " + str(key) + ",\n  {\n")
            for i in range(0, 4):
                res = (co28_target[key][i][0], co28_target[key][i][1], 0)
                fp.write("    cv::Point3f " + str(res)+',\n')
            fp.write(' }\n')
            fp.write('},\n')
        for key in co22_target:
            fp.write('{')
            fp.write("\n  " + str(key) + ",\n  {\n")
            for i in range(0, 4):
                res = (co22_target[key][i][0], co22_target[key][i][1], 0)
                fp.write("    cv::Point3f " + str(res)+',\n')
            fp.write(' }\n')
            fp.write('},\n')