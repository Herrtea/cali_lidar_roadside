from ctypes import sizeof
import numpy as np

def save_x(filename):
    global lidar_points
    global name
    index = 0
    
    with open(filename, "r") as f:
        for line in f.readlines(): 
            line = line[:-1]
            # print(len(line.split("x:")))
            if len(line.split("x:")) == 2:
                lidar_points[index][0] = line.split("x:")[1]
                index += 1
            continue

def save_y(filename):
    global lidar_points
    global name
    index = 0
    
    with open(filename, "r") as f:
        for line in f.readlines(): 
            line = line[:-1]
            if len(line.split("y:")) == 2:
                lidar_points[index][1] = line.split("y:")[1]
                index += 1
            continue
        
def save_z(filename):
    global lidar_points
    global name
    index = 0
    
    with open(filename, "r") as f:
        for line in f.readlines(): 
            line = line[:-1]
            if len(line.split("z:")) == 2:
                lidar_points[index][2] = line.split("z:")[1]
                index += 1
            continue
    
    print("------the number of points is: ", index)
            


if __name__=="__main__":
    
    num_points = 15  #根据每次采集的点修改数量
    lidar_points = np.zeros((num_points,3), dtype = float)
    index = 0
    name = "/home/guoz/hc1011/pole-high/1023/clicked_point_1023.txt"  #根据每次的文件名进行修改
    
    save_x(name)
    save_y(name)
    save_z(name)
    print(lidar_points)
    
    with open('lidar.txt','w') as f: 
        for i in lidar_points:
            f.write(str(i[0]))
            f.write(",")
            f.write(str(i[1]))
            f.write(",")
            f.write(str(i[2]))
            f.write('\r\n')
               

