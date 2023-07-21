### 前言
本项目由HERO23赛季的雷达站负责同学开发，旨在提高RM中雷达站的相机（尤其是双相机）与雷达联合标定的自动化程度，项目中的取图部分使用了上交的[雷达站开源](https://github.com/COMoER/LCR_sjtu)，标定部分参考了Livox开源的[标定算法](https://github.com/Livox-SDK/livox_camera_lidar_calibration/tree/master)，在此一并进行感谢~
### 步骤1: 环境配置
(以下标定基于Ubuntu 64-bit 20.04环境，ROS noetic)
#### 1.1 安装雷达和相机驱动
（这里是**驱动和标定程序源码**，可以直接下载后catkin_make，SDK需要install就没放在里面）
安装ROS1环境，安装 [Livox SDK](https://github.com/Livox-SDK/Livox-SDK) 和mindvision相机SDK，如已安装可以跳过此步骤。
```bash
# 安装Livox_SDK
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
sudo ./third_party/apr/apr_build.sh
cd build && cmake ..
make
sudo make install
```
如安装
#### 1.2 安装依赖
如已安装可以跳过此步骤。

- [PCL 安装](http://www.pointclouds.org/downloads/linux.html)
- [Eigen 安装](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Ceres-solver 安装](http://ceres-solver.org/)
#### 1.3 下载源码，编译准备（见前面压缩包）
```
# 安装本项目
git clone https://github.com/DITINGmisa/HERO_LDStation_WS.git
cd HERO_LDStation_WS
catkin_make
```
#### 1.4 程序节点概括
此项目包括如下节点:

- cameraCalib - 标定相机内参（弃用）
- pcdTransfer - 将雷达点云rosbag转换成PCD文件
- cornerPhoto - 获得照片角点（弃用）
- getExt1 - 计算外参节点１，只优化外参
- getExt2 - 计算外参节点２，同时优化内参和外参
- projectCloud - 把雷达点云投影到照片上
- colorLidar - 雷达点云着色
- **cali_Entry.py - 标定主程序入口，运行这个就够了**
- BagRecorder.py - 使用service来控制rosbag的录制和停止（弃用）
- get_sensor_data.py - 获取相机和雷达数据
- MarkPhoto.py - 标注图像关键点
- MarkPCD.py - 标注点云关键点
- record.sh - 点云录制脚本

以下程序节点中如果想修改launch文件，需要到src/calibration/launch文件夹中找对应的launch文件。
py文件可以直接修改
### 步骤2: 标定准备和数据采集
这里和官方的教程不同，是同时拍摄两个相机的图像和点云，本方案中使用get_senser_data来拍摄照片，代码在GitHub，需要配合取图函数使用。
[Build software better, together](https://github.com/DITINGmisa/HERO_LDStation_WS)
运行前，需要先进入管理员模式并包含ros相关环境变量
**由于代码中有相对路径，故建议在HERO_LDStation_WS根目录下进行各种操作，也不要用roslaunch运行**
```bash
sudo -s
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun camera_lidar_calibration caliEntry.py # 直接rosrun，不要roslaunch
```
然后选1（只有相机由于需要root权限所以需要仅sudo -s，用其他选项都可以直接通过rosrun运行caliEntry.py）
#### 2.1 标定场景准备
本方案使用棋盘格标定板的四个角点来作为目标物，目标物上带有棋盘格，兼具相机标定功能。标定场景最好选择一个较为空旷明朗的环境（选一个自然而然的晴天~）这样方便识别标定板，并且保证雷达到标定板有３米以上。需要选取至少10个左右不同的角度和距离来摆放标定板(参考相机内参的标定物摆放)，左右位置和不同的偏向角度最好都有采集数据。
![1.bmp](https://cdn.nlark.com/yuque/0/2023/bmp/22969079/1689670465282-f9e09625-d85c-410c-806b-c7dd366b302f.bmp#averageHue=%2353717c&clientId=uedafee07-8532-4&from=drop&id=PIAoV&originHeight=2064&originWidth=3088&originalType=binary&ratio=1&rotation=0&showTitle=false&size=19120950&status=done&style=none&taskId=uddcdc465-6852-4268-8d88-2e532afcde3&title=)
不同角度位置摆放标定板。
#### 2.2 连接雷达
确保标定板角点在点云中，如果不确定的话，可以输入点云可视化的命令查看点云。
```bash
roslaunch livox_ros_driver livox_lidar_rviz.launch
```
需要录制rosbag时输入另一个命令。
```bash
roslaunch livox_ros_driver livox_lidar_msg.launch
```
注意根据[链接](https://github.com/Livox-SDK/livox_ros_driver)确认保存的rosbag数据格式是customMsg，后续程序中处理rosbag是对应的“livox custom msg format”格式。
#### 2.3 连接相机
首先需要调整镜头的光圈和对焦，使用630相机的话光圈在4-8之间，随后调整对焦，短焦镜头看的范围广距离近，主要看场地中间区域，需要保证近处10.5m，远处18.3m，都基本看清，长焦主要看敌方前哨站和飞坡区，需要看清18.3m处。
#### 2.4 采集照片和点云数据
##### 2.4.1 临时调整曝光参数
由于光圈较小，所以拍的照片会较暗，为了保证内参标定的质量，需要临时调整一下曝光参数，相机取图类是直接使用的上交雷达站开源中的函数，在preview界面按下t即可进行调整，按q保存并退出。
![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689669947273-afe66d45-4a96-4a7c-a53d-0bc676235196.png#averageHue=%237c9b99&clientId=uedafee07-8532-4&from=paste&height=970&id=u62df516d&originHeight=970&originWidth=1280&originalType=binary&ratio=1&rotation=0&showTitle=false&size=1413153&status=done&style=none&taskId=u3291434a-ddbf-40eb-8695-3d03b260583&title=&width=1280)
##### 2.4.2 拍摄照片和录制点云
按键定义：

- l：lidar，同时保存雷达和两相机数据
- s：shot，仅保存两相机数据
- d：delete，删除最新拍的数据（默认删最晚时间的）
- c：clear，删除所有旧数据
- t：transform，将bag转换到pcd，并保存对应的Image
- q：quit，退出标定程序

**标定数据存储文件夹在HERO_LDStation_WS/src/camera_lidar_calibration/scripts/cacali，**结构：

- bag：原始的bag文件
- cali_ImageL：用于相机雷达标定的左相机图
- cali_ImageR：用于相机雷达标定的右相机图
- ImageL：用于相机标定的左相机图
- ImageR：用于相机标定的右相机图
- param：用于存储相机内外参以及标注的角点信息
- pcd：用于相机雷达标定的点云图

**tips：**

1. l类的数据大概拍摄10张以上，剩下20张左右的可以用s来拍摄，最后使用t将bag转成pcd，同时与bag对应的数据另存到新文件夹以手动标注。
2. 每个位置会保存一张照片和20s左右的rosbag即可。
3. 数据采集完成后，照片和雷达rosbag放在会存储为同名文件存在不同文件夹下。

（保存bag由于是直接用os.system执行并杀死进程，现在会有报错，不过不影响bag的记录）
![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689734963671-4ac0e617-d8d4-401d-a2ae-3ea6c1f31701.png#averageHue=%238fb988&clientId=udec67dcc-3fce-4&from=paste&height=1050&id=u876af7e5&originHeight=1050&originWidth=1916&originalType=binary&ratio=1&rotation=0&showTitle=false&size=1146131&status=done&style=none&taskId=u49053a42-fabb-4cd7-8df2-9b48fd13b27&title=&width=1916)
可以根据bag的存储大小来简单辨认录制是否成功，一般20s的数据会在20mb以上，如果只有几kb的话就要检查自己的msg节点是否正常启动，是否正常发布客制化雷达信息。
![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689735178579-f6df5c14-c6de-456b-bce6-7f3aa902aadf.png#averageHue=%23e6e4e2&clientId=ua39fad26-7647-4&from=paste&height=50&id=u898930f4&originHeight=50&originWidth=324&originalType=binary&ratio=1&rotation=0&showTitle=false&size=5476&status=done&style=none&taskId=u127dcb2b-c986-4197-90d7-aa36fb0fd8d&title=&width=324)
### 步骤3**: **相机参数标定
#### 3.1 导入数据
将ImageL和ImageR文件夹中的图片在Windows端的Matlab进行双目标定。（用双目标定的好处是双目转换关系可以验证相机与雷达的转换关系是否正确）。
![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689771072417-f32af40f-e4fe-41b4-8c3a-dee421080e37.png#averageHue=%23efeeec&clientId=u465b3172-de8e-4&from=paste&height=716&id=u8c502391&originHeight=895&originWidth=1584&originalType=binary&ratio=1.25&rotation=0&showTitle=false&size=182981&status=done&style=none&taskId=u11e0b426-99d5-4b74-a332-5c3e45d7df5&title=&width=1267.2)
#### 3.2 执行计算
选择左右相机文件夹和标定板棋盘格大小；径向误差选择3维的，然后开始计算。
![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689772194379-852aa40a-8062-482d-a527-dba74c37e2a7.png#averageHue=%23caac72&clientId=u465b3172-de8e-4&from=paste&height=115&id=u1e590d06&originHeight=144&originWidth=580&originalType=binary&ratio=1.25&rotation=0&showTitle=false&size=15579&status=done&style=none&taskId=u770716fe-4ba5-47b9-96ab-f9da906d500&title=&width=464)
平均误差0.4左右的像素就可以接受，导出结果。
#### 3.3 保存数据
我们需要的有如下参数：

1. 两个相机的RadialDistortion、TangentialDistortion和IntrinsicMatrix（转置后才能用）。

![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689772376866-d79c811f-5d01-400b-ac08-cd0a6920f674.png#averageHue=%23f7f4f0&clientId=u465b3172-de8e-4&from=paste&height=230&id=u3184f001&originHeight=287&originWidth=494&originalType=binary&ratio=1.25&rotation=0&showTitle=false&size=35784&status=done&style=none&taskId=ufc373901-10f4-4a65-a3c1-36d0c38c480&title=&width=395.2)

2. RotationOfCamera2（转置后才能用）和TranslationOfCamera2

 ![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689772310148-9f9b73cf-1576-486a-9457-a35e13882312.png#averageHue=%23efe9e2&clientId=u465b3172-de8e-4&from=paste&height=31&id=u3b380c7f&originHeight=39&originWidth=473&originalType=binary&ratio=1.25&rotation=0&showTitle=false&size=5513&status=done&style=none&taskId=u5a47f328-9cf8-42ab-af27-4933832d86e&title=&width=378.4)
**tips：**
RadialDistortion、TangentialDistortion一般会表示为K和P，组成五维向量使用（K1 K2 P1 P2 K3）。
RotationOfCamera2（转置后才能用）和TranslationOfCamera2是相机2相对于相机1的转换矩阵，表示如下：
$p_2 = R_{21} p_1 + t_{21}$
其中

- p2为点p在相机2坐标系下的坐标
- p1为点p在相机1坐标系下的坐标
- R21为RotationOfCamera2
- t21为TranslationOfCamera2
### 步骤4**: **标注关键点
#### 4.1 写入内参
将上一步的两个相机的内参和畸变纠正参数以下图的格式保存在data/parameters/intrinsic.txt文件下。distortion下面对应５个畸变纠正参数，按顺序是k1和k2 (RadialDistortion)，p1和p2 (TangentialDistortion)，最后一个是k3。
![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689783744812-0c2d0236-d809-4ad9-999b-175339885b93.png#averageHue=%23f0f4f7&clientId=ued75b243-3bc4-4&from=paste&height=166&id=u0ae3063f&originHeight=166&originWidth=899&originalType=binary&ratio=1&rotation=0&showTitle=false&size=60748&status=done&style=none&taskId=u42903da4-2e70-4fb4-8b18-6770582729d&title=&width=899)
这里第1，5，6行的数据可以随便写，不会被程序读到；2，3，4，7行的数据间用一个空格隔开。
#### 4.2 获得照片中的角点坐标
如果步骤2正常操作，此时在cali_img_pathL和cali_img_pathR已经有了对应的图像，只需在caliEntry.py中选择2即可开始标定，如下图。
![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689785713860-d5a3d8a6-854f-4830-ba28-63701d67c010.png#averageHue=%2378aa86&clientId=ued75b243-3bc4-4&from=paste&height=920&id=u386bb684&originHeight=920&originWidth=2513&originalType=binary&ratio=1&rotation=0&showTitle=false&size=1644357&status=done&style=none&taskId=u0dd38d35-da86-47fb-9643-c31094bbc4c&title=&width=2513)
最左边的小窗口可以放大鼠标周围的图像来提高标注精准度，主图像会显示提示信息，标注顺序是从左上角起逆时针标注，命令行界面则会显示标注图像的信息和标注进度。
Tips：由于标定板边角处存在点云发散，边角有可能变成圆角，建议相机和点云标注时都统一稍微向内偏顺或逆时针标一些，以保证对应点在实际位置上更接近。
标注好的文件be like：
![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689785692019-fb7f7f96-791c-4671-a297-fe39d4220a72.png#averageHue=%23ebeff3&clientId=ued75b243-3bc4-4&from=paste&height=162&id=ub6addb96&originHeight=162&originWidth=451&originalType=binary&ratio=1&rotation=0&showTitle=false&size=39112&status=done&style=none&taskId=uc822f5cd-b998-474b-9200-1f7a9e0ee60&title=&width=451)
#### 4.3 获得点云中的角点坐标
获取点云使用的是Open3D库编写的，需要`pip install open3d`，Open3D占用空间较大，如果想安装在其他的环境中，可以更改caliEntry.py的第16行的python解释器路径。
![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689832848471-52ee6b6d-39a3-4e06-a38a-c85dd1cdfb81.png#averageHue=%23f3f7f9&clientId=uc3e474e1-495d-4&from=paste&height=670&id=ue663779a&originHeight=670&originWidth=1071&originalType=binary&ratio=1&rotation=0&showTitle=false&size=286893&status=done&style=none&taskId=u99a3ded2-fb45-4b20-94b1-c18bce3e3f9&title=&width=1071)
Open3D常用的快捷键如下，其他的可以按H键查看:
**Left button + drag : Rotate.**
**Ctrl + left button + drag : Translate.**
**Wheel button + drag : Translate.**
**Shift + left button + drag : Roll.**
**Wheel : Zoom in/out.**
**[ / ] : Increase/decrease field of view.**
**Q, Esc : Exit window.**
**H : Print help message.**
**Alt + Enter  : Toggle between full screen and windowed mode.**
**+/- : Increase/decrease point size.**
**Shift + +/-  : Increase/decrease picked point size..**
**Shift + mouse left button   : Pick a point and add in queue.**
**Shift + mouse right button  : Remove last picked point from queue.**
**基本流程：**

- **灵活通过鼠标左键、ctrl或shift+左键、和Alt加左键来拖动图片选择合适角度。**
- **通过+/-来调整点云到合适大小，shift+左键选择一个点，如果选错或者拖动时不可避免点到可以用shift+右键来撤销。**
- **shift + +/-调整被选中点到合适大小（一般建议直接最小）。**
- **选好四个点后拖动鼠标观察一下，如果无误就随便再选一个点，程序的回调函数检测到4个点以上时才会退出（防止第四个点没选好需要撤销）。**

![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689826670645-b01ddcd7-ae4c-4bc7-b89a-e113449fac06.png#averageHue=%23cdf889&clientId=uc3e474e1-495d-4&from=paste&height=1119&id=ud45ec33b&originHeight=1119&originWidth=1918&originalType=binary&ratio=1&rotation=0&showTitle=false&size=1453452&status=done&style=none&taskId=u1821664f-b2c3-4ffd-a437-4148e76c213&title=&width=1918)
标注好的数据be like：
![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689841600224-d7d940a8-5f7e-4407-8b37-abff9db8323c.png#averageHue=%23ecf0f5&clientId=uc3e474e1-495d-4&from=paste&height=211&id=u24096fc2&originHeight=211&originWidth=423&originalType=binary&ratio=1&rotation=0&showTitle=false&size=44967&status=done&style=none&taskId=u035687e7-380a-42eb-92b9-6d67967ac55&title=&width=423)
### 步骤５: 外参计算
#### 5.1 参数设置
如果按照上面的操作步骤进行，则参数的形式就是可以被后续程序识别的，如果格式不对，则需要对照着前面的截图更改一下相机内参文件、图像关键点文件和点云关键点文件。
#### 5.2 外参计算getExt1节点
在caliEntry.py中选4再选1。
终端中可以看到每次迭代运算的cost，外参结果以齐次矩阵的格式保存到cali/param/extrinsicL(R).txt下。
可以根据优化后的残差和重投影误差评估一下得到的外参。![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689842495707-0bb903a9-50be-488f-925f-487e8882a425.png#averageHue=%231d1616&clientId=uc3e474e1-495d-4&from=paste&height=699&id=u58b47cfc&originHeight=699&originWidth=803&originalType=binary&ratio=1&rotation=0&showTitle=false&size=86704&status=done&style=none&taskId=u20cb08c6-6b83-41db-bfcc-7d1a2d836f5&title=&width=803)
误差阈值可以自己在getExt1.launch中更改。重投影会把误差较大的数据打印在屏幕上，可以剔除异常标定数据后再重新进行优化计算。
#### 5.3 外参计算getExt２节点
在caliEntry.py中选4再选2。
getExt1节点只优化外参，而getExt2节点在计算的时候会将一开始计算的内参作为初值和外参一起优化。输入指令程序会得到一个新的内参和外参，并用新的参数来进行重投影验证。
一般使用getExt1节点即可，如果在外参初值验证过，并且异常值已经剔除后，优化还是有较大的残差，那么可以使用getExt2试一试。使用的前提需要保证标定数据量较大，并且要充分验证结果。
如果经过验证getExt2计算的结果确实更好，那么把新的内参更新在cali/param/intrinsic.txt中。
#### 5.4 简单验证
此时翻出我们珍藏已久的相机2（R）到相机1（L）的转换矩阵来简单验证一下，有如下关系：
![image.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689853569545-a3091bf2-3439-4bca-b50f-1a703eaaba7d.png#averageHue=%23fefefe&clientId=u943f1d48-969a-4&from=paste&height=716&id=u42db6902&originHeight=716&originWidth=1087&originalType=binary&ratio=1&rotation=0&showTitle=false&size=16210&status=done&style=none&taskId=u3da8de19-f2b1-4b02-a05d-8c94736a837&title=&width=1087)
设任意一点P在O0，O1，O2坐标系下的齐次坐标分别为p0，p1，p2，则有：
p2=M21p1=M20p0，p1=M10p0            ===>    M20p0 = M21M10p0
P为任意一点，故应有M20 = M21M10，为方便计算，使用M20(M10)-1 = M21
其中Mij表示从同一个点的齐次坐标从坐标系Oi向Oj的转换矩阵，例如M10和M20是雷达向左右相机的转换矩阵，M21是标定时保留的转换矩阵。
计算结果请在caliEntry.py选6。
未剔除大误差点的情况下：
![fe0e0c37f3504189ec596c0db352dfd1.png](https://cdn.nlark.com/yuque/0/2023/png/22969079/1689857726851-0d476567-64c6-475d-82c7-43a604e2313b.png#averageHue=%230f921e&clientId=u3a2bf121-04d5-4&from=drop&id=uad39df66&originHeight=161&originWidth=974&originalType=binary&ratio=1&rotation=0&showTitle=false&size=43380&status=done&style=none&taskId=u62d36aa9-8047-405b-84e7-17e825b0791&title=)
注意，Matlab中的双目标定的平移向量单位为mm，而标定程序计算出的单位则是m。可以看出误差不太大，基本在毫米级到厘米级之间。
至此，你的标定就初步完成啦，可以继续优化或者开始着手定位算法的研究了。
### 拓展: 结果验证与相关应用
获得外参后我们可以用两个常见的应用看一下融合的效果。第一个是将点云投影到照片上，第二个是点云的着色。

- 在projectCloud.launch文件中配置点云和照片的路径后，运行指令，将rosbag中一定数量的点投影到照片上并且保存成新的照片。
- 在colorLidar.launch文件中配置点云和照片的路径，运行指令，可以在rviz中检查着色的效果。
