import cv2
import sys
import os
import subprocess
import time
import signal


sys.path.append('/home/ditingmisa/RM/HERO_23_LDStation')

from radar_class.camera import Camera_Thread
img_pathL = '/home/ditingmisa/RM/HERO_LDStation_WS/src/camera_lidar_calibration/cali/ImageL/'
img_pathR = '/home/ditingmisa/RM/HERO_LDStation_WS/src/camera_lidar_calibration/cali/ImageR/'
cali_img_pathL = '/home/ditingmisa/RM/HERO_LDStation_WS/src/camera_lidar_calibration/cali/cali_ImageL/'
cali_img_pathR = '/home/ditingmisa/RM/HERO_LDStation_WS/src/camera_lidar_calibration/cali/cali_ImageR/'
project_pathL = None #'/home/ditingmisa/RM/HERO_LDStation_WS/src/lcamera_lidar_calibration/cali/projection/zerosL.png' 是否将雷达点云投影叠加到图像上
project_pathR = None #'/home/ditingmisa/RM/HERO_LDStation_WS/src/camera_lidar_calibration/cali/projection/zerosR.png' 是否将雷达点云投影叠加到图像上
bag_path = '/home/ditingmisa/RM/HERO_LDStation_WS/src/camera_lidar_calibration/cali/bag/'
cap1 = Camera_Thread(1)
cap2 = Camera_Thread(0)# 0R 1L to 1L 2R
c = ""
while c != 'q':
    flag1, frame1 = cap1.read()
    flag2, frame2 = cap2.read()
    # 新建窗口
    cv2.namedWindow("imgL", cv2.WINDOW_NORMAL)
    cv2.namedWindow("imgR", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("imgL", 640, 480)
    cv2.resizeWindow("imgR", 640, 480)
    if flag1:
        if project_pathL:
            # 混合图像
            pro = cv2.imread(project_pathL)
            frame1 = cv2.addWeighted(frame1, 0.5, pro, 0.5, 0)
        cv2.imshow("imgL", frame1)
    if flag2:
        if project_pathL:
            # 混合图像
            pro = cv2.imread(project_pathR)
            frame2 = cv2.addWeighted(frame2, 0.5, pro, 0.5, 0)
        cv2.imshow("imgR", frame2)
    c = chr(cv2.waitKey(1) & 0xFF)
    if c == 'l':
        # 存雷达点云和图像
        if bag_path:
            name = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime()) # 防止重名
            # 启动子进程
            cmd = str.format("%s '%s'" % ("/home/ditingmisa/RM/HERO_LDStation_WS/src/camera_lidar_calibration/script/record.sh",
                                      f"rosbag record /livox/lidar -O {bag_path}{name}.bag"))
            cc = str.format(f"rosbag record /livox/lidar -O {bag_path}{name}.bag")
            print(cmd)
            os.system(f"/bin/bash -c \"source /opt/ros/noetic/setup.bash && {cmd}\"")
        else:
            name = input("没有找到bag path. 请创建或更改脚本配置")
        print("save to ", f'{name}.bag')
        cv2.imwrite(img_pathL + f'{name}.bmp', frame1)
        cv2.imwrite(img_pathR + f'{name}.bmp', frame2)
        print("Saved")
    
    if c == 's':
        # 存图像
        name = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        print("save to ", img_pathL + f'{name}.bmp')
        cv2.imwrite(img_pathL + f'{name}.bmp', frame1)
        cv2.imwrite(img_pathR + f'{name}.bmp', frame2)
        print("Saved")

    if c == 'd':
        # 删除最后一张图像
        list = sorted(os.listdir(img_pathL))
        name = list[-1]
        try:
            os.remove(img_pathL + f'{name}')
            print("delete ", img_pathL + f'{name}')
            os.remove(img_pathR + f'{name}')
            print("delete ", img_pathR + f'{name}')
            if os.path.exists(bag_path + name.replace('.bmp', '.bag')):
                os.remove(bag_path + name.replace('.bmp', '.bag'))
                print("delete ", bag_path + name.replace('.bmp', '.bag'))
            print("Deleted")
        except:
            print("有无法对应的文件，可能已经删除，请检查或清空")

    if c == 'c':
        com = input("是否删除所有图像和bag文件?(y/n)")
        if not com == 'y':
            continue
        # 删除所有图像 
        for name in os.listdir(img_pathL):
            os.remove(img_pathL + f'{name}')
            print("delete ", img_pathL + f'{name}')
        for name in os.listdir(img_pathR):
            os.remove(img_pathR + f'{name}')
            print("delete ", img_pathR + f'{name}')
        for name in os.listdir(bag_path):
            os.remove(bag_path + f'{name}')
            print("delete ", bag_path + name)
        print("Deleted")
    
    if c == 't':
        # 转换bag文件
        import rospy
        rospy.set_param('/data_num', len(os.listdir(bag_path)))
        for i, bag in enumerate(sorted(os.listdir(bag_path))):
            if bag.endswith('.bag'):
                os.system(f"cp {bag_path}{bag} {bag_path}{i}.bag")
            x = bag.replace('.bag', '.bmp')
            if os.path.isfile(f'{img_pathL}{x}'):
                os.system(f"cp {img_pathL}{x} {cali_img_pathL}{i}.bmp")
            if os.path.isfile(f'{img_pathR}{x}'):
                os.system(f"cp {img_pathR}{x} {cali_img_pathR}{i}.bmp")
        os.system("roslaunch camera_lidar_calibration pcdTransfer.launch")
        
    if c == 'q':
        # 退出
        exit(0)



