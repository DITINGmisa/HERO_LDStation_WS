import open3d as o3d
import os

def pick_points(pcd):
    """
    手动选择四个点并获取其三维坐标
    :param pcd: 点云数据
    :return: 四个点的坐标
    """
    points = []
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    render_option = vis.get_render_option()
    render_option.point_size = 3.0
    
    # 获取视图控制对象
    view_control = vis.get_view_control()
    front = [0.0, 0.0, 1.0]# 设置视角为沿正Z轴方向
    up = [0.0, -1.0, 0.0]  # 设置Y轴为上方向
    view_control.set_front(front)
    view_control.set_up(up)
    vis.add_geometry(pcd)
    # 包装函数，将points和vis传递给pick_callback
    def pick_wrapper(vis):
        pick_callback(vis, points)

    vis.register_animation_callback(pick_wrapper)
    vis.run()

    return points
    

def pick_callback(vis, points):
    """
    回调函数：获取选点，当pick_points队列的长度为5时，销毁窗口
    """
    picked_points = vis.get_picked_points()
    for point_idx in picked_points:
        pass
    if len(picked_points) == 5:
        # 获取选中的点的坐标
        for i in range(4):
            point_idx = picked_points[i]
            point = pcd.points[point_idx]
            points.append(point)
            print(f"已选择点 ({point[0]:.4f}, {point[1]:.4f}, {point[2]:.4f})")
        # 手动选点完成，销毁窗口
        vis.destroy_window()


pcd_path = 'src/camera_lidar_calibration/cali/pcd'
output_path = 'src/camera_lidar_calibration/cali/param/corner_lidarL.txt'
result = []

# 加载pcd文件
for i in range(len(os.listdir(pcd_path))):
    pcd = o3d.io.read_point_cloud(f'src/camera_lidar_calibration/cali/pcd/{i}.pcd')
    # 手动选择四个点并获取其三维坐标
    points = pick_points(pcd)
    
    if not len(points) == 4:
        print(f"选点失败，请重新选点, 已选的点：{result}")
        break
    result.append(points)

assert len(result) == len(os.listdir(pcd_path)), f"选点失败，请重新选点, 已选的点：{result}"
# 存储四个点的坐标到文件中
with open(output_path, 'w') as f:
    for points in result:
        for point in points:
            f.write(f"{point[0]:.4f} {point[1]:.4f} {point[2]:.4f}\n")
os.system(f"cp {output_path} {output_path.replace('L.txt', 'R.txt')}")
