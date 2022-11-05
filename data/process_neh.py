from turtle import rt
import numpy as np


def load_txt(filename):
    data = np.loadtxt(filename,dtype=np.float64,delimiter=',')
    return data

if __name__=="__main__":
    
    np.set_printoptions(suppress=True)
    name = '/home/guoz/hc1011/pole-high/1023/1023polehigh.txt'  #根据文件名修改
    num_points = 15  #根据每次采集的点修改数量
    rtk_points = np.zeros((num_points + 1,3), dtype= float)  #多一个原点的数量 下面写入的时候会跳过原点
    index = 0
    
    with open(name, "r") as f:
        for line in f.readlines(): 
            line = line[:-1]
            if len(line.split(',')) == 4:
                rtk_points[index] = line.split(',')[1:]
                index += 1
            if len(line.split(',')) == 3:
                rtk_points[index] = line.split(',')
                index += 1
            continue
    # print(rtk_points)
    data = rtk_points
        
    # data = load_txt(name)
    # data.reshape((-1,3))
    # print(data.shape[0])
    
    print('--------before calculate-------\n')
    print(data)
    for i in range(data.shape[0]-1):
        data[i+1] = data[i+1] - data[0]
    print('--------after calculate-------\n')
    print(data)
    # data = data.tolist()
    # print(type(data[0][1]))
    
    with open('neh.txt','w') as f: 
        for i in data[1:]:
            f.write("%.8f"%i[0])
            # f.write(i[0])
            f.write(",")
            f.write("%.8f"%i[1])
            # f.write(i[1])
            f.write(",")
            f.write("%.8f"%i[2])
            # f.write(i[2])
            f.write('\r\n')
    
    # np.savetxt("./e_high_neh-2.txt", data,fmt ='%f')
