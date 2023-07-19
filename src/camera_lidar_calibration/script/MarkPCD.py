import open3d as o3d
import cv2

def pick_points(pcd):
    """
    手动选择四个点并获取其三维坐标
    :param pcd: 点云数据
    :return: 四个点的坐标
    """
    points = []
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()

    while True:
        vis.update_renderer()

        print("请选择四个点，按下 'Enter' 键确认选点，按下 'Backspace' 键撤销上一个点。")

        key = cv2.waitKey(1)
        if key == 13:  # 按下 'Enter' 键
            break
        elif key == 8:  # 按下 'Backspace' 键
            if points:
                points.pop()
                print("已撤销上一个点")

        if vis.poll_events():  # 处理 Open3D 的事件
            continue

        if vis.update_geometry():
            vis.update_renderer()

        if key != -1:
            index = vis.get_picked_points()[0]
            point = pcd.points[index]
            points.append([point[0], point[1], point[2]])
            print(f"已选择点 ({point[0]:.4f}, {point[1]:.4f}, {point[2]:.4f})")

    vis.destroy_window()

    return points

def save_points(points, filename):
    """
    将四个点的坐标顺序存储到文件中
    :param points: 四个点的坐标列表
    :param filename: 存储文件名
    """
    with open(filename, 'w') as f:
        for point in points:
            f.write(f"{point[0]:.4f} {point[1]:.4f} {point[2]:.4f}\n")

# 加载pcd文件
pcd = o3d.io.read_point_cloud('/home/ditingmisa/RM/env/ws_livox/src/livox_camera_calib/result/0.pcd')

# 手动选择四个点并获取其三维坐标
points = pick_points(pcd)

# 存储四个点的坐标到文件中
save_points(points, 'points.txt')
