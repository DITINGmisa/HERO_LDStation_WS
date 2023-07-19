import rospy
import cv2

# 初始化ROS节点
rospy.init_node('trackbar_node')

# 设置ROS参数初始值
car_conf = rospy.get_param('car_conf')
car_nms = rospy.get_param('car_nms')
armor_conf = rospy.get_param('armor_conf')

def cb1(x):
    rospy.set_param('car_conf', x/100)

def cb2(x):
    rospy.set_param('car_nms', x/100)

def cb3(x):
    rospy.set_param('armor_conf', x/100)

# 创建窗口和Trackbar
window_name = 'Trackbar Example'
cv2.namedWindow(window_name)
cv2.resizeWindow(window_name, 600, 400)
cv2.createTrackbar('car_conf', window_name, 0, 100, cb1)
cv2.createTrackbar('car_nms', window_name, 0, 100, cb2)
cv2.createTrackbar('armor_conf', window_name, 0, 100, cb3)
cv2.setTrackbarPos('car_conf', window_name, int(car_conf*100))
cv2.setTrackbarPos('car_nms', window_name, int(car_nms*100))
cv2.setTrackbarPos('armor_conf', window_name, int(armor_conf*100))

while True:
    # 显示窗口
    cv2.imshow(window_name,0)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放窗口和资源
cv2.destroyAllWindows()
