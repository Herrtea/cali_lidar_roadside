激光雷达标定流程（以2.4m处激光雷达为例）:
1. 根据rosbag截取pcd文件[board.pcd](data/low_lidar/1/board.pcd);

eg: 订阅/low/rslidar_points话题中的点云数据并保存
$ rosrun pcl_ros pointcloud_to_pcd /input:=/low/rslidar_points ./
参数./改为你要保存.pcd点云的目录./代表当前终端所在目录，点云最后的保存形式为 '时间戳.pcd'，如'1630559271.pcd',此处我改为board.pcd

2. 使用[select_pointcloud.cpp](src/select_pointcloud.cpp)的函数plot_and_choose()读取步骤1中截取的pcd文件并手动选择（shift+鼠标左键）标定板两个底部角点在激光雷达坐标系下的坐标，并将坐标存到[lidar.txt](data/low_lidar/lidar.txt)中;

3. 将从手持RTK中得到的标定板两个底部角点在东北天（全局）坐标系下的坐标存到[neh.txt](data/low_lidar/neh.txt)中;
注：lidar.txt与neh.txt中的点必须对应起来，即是同一个角点分别在激光雷达坐标系下和东北天（全局）坐标系下的坐标。

4. 用[calibration.cpp ](src/calibration.cpp)的函数read_lidar_txt()读取激光雷达坐标系下的角点坐标并存到lidar_points中，用read_neh_txt()读取东北天坐标系下的角点坐标并存到global_points;

5. 用[calibratio.cpp ](src/calibration.cpp)函数pose_estimation_3d3d()去计算从激光雷达坐标系到东北天坐标系的齐次变换矩阵transformation_matrix_low_to_global;

6. 用[calibratio.cpp ](src/calibration.cpp)函数RotationMatrix2RPY()计算步骤5中得到的齐次变换矩阵左上角3x3的旋转矩阵对应的欧拉角，用于后续雷达参数设置中的roll, pitch, yaw;

7.(可选) 用[calibratio.cpp ](src/calibration.cpp)在ROS订阅原始雷达信息，转换成PCL点云格式后利用齐次变换矩阵将点云变换到全局坐标系下，然后发布出去，该操作主要通过回调函数low_callback()完成。







使用方法：

## 0.编译

```
sh
mkdir -p calibration/src
cd calibration/src
catkin_init_workspace
将lidar_calibration文件夹拷贝到此
cd ..
catkin_make
```



## 1.首先进行数据采集：需要标定板和手持rtk

1.1打开雷达驱动。

通过在视野内移动，通过rviz来观察反光贴是否明显，选定位置后录制bag包（以备待查）和发布角点

```
sh
rosbag record -a -o 'E/ganzi'-'low/high'-num.bag
如 -o E-low-1.bag  代表针对E楼低处雷达的第一个标定点所记录的bag包

sh
rostopic echo /select_points
然后通过rviz中的select point来选定标定板角点 （GUI中间红色logo按钮）
记录下echo中发布出的角点坐标，此点坐标为激光雷达坐标系下的点坐标

```

1.2然后针对所选中的角点，通过手持rtk，记录此点在东北天坐标系下的坐标。（手持rtk在记录标定点之前要记录一个原点，即定下场地坐标系原点的坐标）

针对每个雷达的标定点记录不少于20个，尽量多一些。

## 2.处理数据

生成lidar.txt和neh.txt

2.1lidar.txt

将上述echo中记录的点处理成如下形式。下面是23个点的坐标，每一行形式为 x,y,z不要有多余的符号和行

```
13.674049377441406,9.223119735717773,-0.1662362813949585
14.281848907470703,11.402772903442383,-0.06360620260238647
18.206146240234375,9.986137390136719,0.9059762954711914
16.37517738342285,4.607728481292725,0.4766312837600708
15.032294273376465,1.521738886833191,0.1394050121307373
14.35738754272461,-1.3206753730773926,0.004349470138549805
14.044811248779297,-4.855583190917969,-0.09443831443786621
13.754976272583008,-8.478675842285156,-0.13654017448425293
13.366559982299805,-11.562169075012207,-0.27493906021118164
13.126972198486328,-14.007590293884277,-0.29687273502349854
12.982686996459961,-16.8930721282959,-0.3273189067840576
13.276081085205078,-20.929075241088867,-0.3001020550727844
17.053272247314453,-21.53081703186035,0.6741266250610352
12.526045799255371,-22.219619750976562,-0.4477289915084839
9.817468643188477,-21.17237663269043,-1.1395890712738037
8.228595733642578,-18.57329559326172,-1.4165115356445312
8.452448844909668,-14.515121459960938,-1.4075658321380615
7.938251972198486,-8.73154354095459,-1.4755463600158691
6.9940996170043945,8.058009147644043,-1.7302143573760986
15.61767292022705,16.281803131103516,0.24345409870147705
14.985424041748047,19.847251892089844,0.1344912052154541
12.225580215454102,21.147884368896484,-0.5059958696365356
7.710107326507568,21.235401153564453,-1.494884967803955
```

手持rtk，导出东北天坐标系下的坐标，具体导出方法见手持rtk操作文档。导出的坐标应为23+1，其中的1为原点坐标。将导出的23+1坐标处理为如上格式，第一个点务必为坐标系原点，文件命名为preneh.txt。然后运行data/文件夹中

```
python process_base.py
```

生成neh.txt文档，对其中的xyz填充逗号（或者更改calibration.cpp中read_neh_txt函数101行 ',' 改为' '）



## 3.生成转换矩阵

注释calibration.cpp366行ros::spin（） 再次catkin_make编译（每次对cpp文件有修改时都需要编译）

修改calibration.h文件，116和117行，将lidar.txt和neh.txt文件的绝对路径填入。（也可使用相对路径../data/lidar.txt）

编译后，在ws下，运行

```
source devel/setup.bash   （每开一个新的终端，都要运行此句）
rosrun lidar_calibration calibration
```

命令行中会输出旋转矩阵和对应的欧拉角



## 4.视觉验证

取消注释calibration.cpp366行ros::spin（）   将上述得到的转换矩阵填入transformation_matrix_low_to_global变量中，确保31行订阅的话题消息名和rosbag包中包含的雷达话题名相同。

再次catkin_make编译

开启主节点

```
roscore
```

运行节点

```
source devel/setup.bash   
rosrun lidar_calibration calibration
```

新开终端播放bag包

```
rosbag play --loop 包名
```

打开rviz

```
rviz
在gui中设置rviz的frame id为rslidar
点击左下角add，添加bag包中的点云话题和转换后的点云话题
```





data中的两个脚本辅助处理lidat.txt和neh.txt



float转str的时候会有一定的偏差  可以用“%.8f”%data的形式来处理