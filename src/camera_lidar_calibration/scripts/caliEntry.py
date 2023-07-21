import os
import time
while True:
    time.sleep(0.5)
    print("1. 拍摄数据")
    print("2. 选择图像特征点")
    print("3. 选择点云特征点")
    print("4. 执行标定")
    print("5. 验证标定结果")
    print("6. 退出")
    cmd = input("请输入指令：")
    if cmd == '1':
        os.system("python3 src/camera_lidar_calibration/scripts/get_sensor_data.py")
    elif cmd == '2':
        os.system("python3 src/camera_lidar_calibration/scripts/MarkPhoto.py")
    elif cmd == '3':
        os.system("/home/ditingmisa/anaconda3/envs/yolo/bin/python3 src/camera_lidar_calibration/scripts/MarkPCD.py")
    elif cmd == '4':
        import rospy
        type = input("请选择标定类型：1.外参标定 2.内外参标定(需大量数据)")
        for s in ["L", "R"]:
            rospy.set_param('intrinsic_path', f"{os.getcwd()}/src/camera_lidar_calibration/cali/param/intrinsic{s}.txt")
            rospy.set_param('extrinsic_path', f"{os.getcwd()}/src/camera_lidar_calibration/cali/param/extrinsic{s}.txt")
            rospy.set_param('input_lidar_path', f"{os.getcwd()}/src/camera_lidar_calibration/cali/param/corner_lidar{s}.txt")
            rospy.set_param('input_photo_path',f"{os.getcwd()}/src/camera_lidar_calibration/cali/param/corner_photo{s}.txt")
            if type == '1':
                os.system(f"roslaunch camera_lidar_calibration getExt{type}.launch")
            elif type == '2':
                os.system(f"roslaunch camera_lidar_calibration getExt{type}.launch")
            else:
                print("输入错误")
                break
    elif cmd == '5':
        import numpy as np
        def read_matrix_from_file(filename):
            """
            从文件中读取矩阵数据
            :param filename: 文件名
            :return: 读取的矩阵
            """
            with open(filename, 'r') as f:
                matrix = np.zeros((4, 4))
                f.readline()
                for i in range(4):
                    line = f.readline()
                    line = line.strip().split('  ')
                    for j in range(len(line)):
                        matrix[i][j] = float(line[j])
            return np.array(matrix)

        # 从文件中读取矩阵A和B
        matrix_A = read_matrix_from_file('src/camera_lidar_calibration/cali/param/extrinsicR.txt') # M20
        matrix_B = read_matrix_from_file('src/camera_lidar_calibration/cali/param/extrinsicL.txt') # M10

        # 计算AB^{-1}
        result_matrix = np.dot(matrix_A, np.linalg.inv(matrix_B)) # M20 * M10^{-1}
        print("M20 * M10^{-1}的结果为：请与M21比较")
        print(result_matrix)

    elif cmd == '6':
        exit(0) 
