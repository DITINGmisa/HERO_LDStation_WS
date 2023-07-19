'''
从畸变矫正后的图像中选择边缘点
'''
import cv2
import numpy as np
import os
from datetime import datetime


def __callback_1(event,x,y,flags,param):
    '''
    鼠标回调函数
    鼠标点击点：确认标定点并在图像上显示
    鼠标位置：用来生成放大图
    '''
    # using EPS and MAX_ITER combine
    stop_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                        30, 0.001)
    if event == cv2.EVENT_MOUSEMOVE:
        # 周围200*200像素放大图
        rect = cv2.getWindowImageRect(param["pick_winname"])
        img_cut = np.zeros((200,200,3),np.uint8)
        img_cut[max(-y+100,0):min(param["pick_img"].shape[0]+100-y,200),max(-x+100,0):min(param["pick_img"].shape[1]+100-x,200)] = \
        param["pick_img"][max(y-100,0):min(y+100,param["pick_img"].shape[0]),max(x-100,0):min(x+100,param["pick_img"].shape[1])]
        cv2.circle(img_cut,(100,100),1,(0,255,0),-1)
        cv2.imshow(param["zoom_winname"], img_cut)
        cv2.moveWindow(param["zoom_winname"],rect[0]-400,rect[1]+200)
        cv2.resizeWindow(param["zoom_winname"], 400,400)
    if event == cv2.EVENT_LBUTTONDOWN and not param["pick_flag"]:
        param["pick_flag"] = True
        print(f"pick ({x:d},{y:d})")
        # 亚像素精确化
        corner = cv2.cornerSubPix(param["pick_img_raw"],np.float32([x,y]).reshape(1,1,2),(5,5),(-1,-1),stop_criteria).reshape(2)
        param["pick_point"] = [corner[0],corner[1]]
        cv2.circle(param["pick_img"],(x,y),2,(0,255,0),1)

def locate_pick(frame_dir, num):
    '''
    手动四点标定

    :return: 四点坐标
    '''

    # 窗口下方提示标定哪个目标
    tips = ['left_top', 'left_bottom', 'right_bottom','right_top' ]
    frame = cv2.imread(os.path.join(frame_dir, f"{num}.bmp"))
    # 使用内参矩阵进行畸变矫正，存在img文件夹同级的文件夹中
    with open(frame_dir + '../parameters/intrinsic.txt', 'r') as f:
        f.readline()
        m = np.zeros((3, 3))
        for i in range(3):
            m[i][:] = f.readline().split(" ")
        f.readline()
        f.readline()
        d = np.zeros((1, 5))
        d[0][:] = f.readline().split(" ")
    frame = cv2.undistort(frame, m, d)

    # 标定目标提示位置
    tip_w = frame.shape[1]//2
    tip_h = frame.shape[0]-200

    # OpenCV窗口参数
    info = {}
    info["pick_img"] = frame.copy()
    info["pick_img_raw"] = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    info["pick_winname"] = "pick_corner"
    info["zoom_winname"] = "zoom_in"
    info["pick_flag"] = False
    info["pick_point"] = None # 回调函数中点击的点

    cv2.namedWindow(info["pick_winname"], cv2.WINDOW_NORMAL)
    cv2.resizeWindow(info["pick_winname"], 1280,780)
    cv2.setWindowProperty(info["pick_winname"],cv2.WND_PROP_TOPMOST,1)
    cv2.moveWindow(info["pick_winname"], 500,300)
    cv2.namedWindow(info["zoom_winname"], cv2.WINDOW_NORMAL)
    cv2.resizeWindow(info["zoom_winname"], 400, 400)
    cv2.setWindowProperty(info["zoom_winname"],cv2.WND_PROP_TOPMOST,1)
    cv2.setMouseCallback("pick_corner", __callback_1, info)

    pick_point = []
    while True:
        frame = cv2.imread(os.path.join(frame_dir, f"{num}.bmp"))
        # draw tips
        cv2.putText(frame,tips[len(pick_point)],(tip_w,tip_h),
                    cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),2)

        # draw the points having been picked
        for select_p in pick_point:
            cv2.circle(frame, (int(select_p[0]), int(select_p[1])), 1, (0, 255, 0), 2)

        # draw the connecting line following the picking order
        for p_index in range(1, len(pick_point)):
            cv2.line(frame, (int(pick_point[p_index - 1][0]), int(pick_point[p_index - 1][1])),
                    (int(pick_point[p_index][0]), int(pick_point[p_index][1])), (0, 255, 0), 2)

        cv2.imshow(info["pick_winname"], frame)

        if info["pick_flag"]: # 当在回调函数中触发点击事件
            pick_point.append(info["pick_point"])
            # draw the points having been picked
            for select_p in pick_point:
                cv2.circle(frame, (int(select_p[0]), int(select_p[1])), 1, (0, 255, 0), 2)

            # draw the connecting line following the picking order
            for p_index in range(1, len(pick_point)):
                cv2.line(frame, (int(pick_point[p_index - 1][0]), int(pick_point[p_index - 1][1])),
                        (int(pick_point[p_index][0]), int(pick_point[p_index][1])), (0, 255, 0), 2)
            # 四点完成，首尾相连
            if len(pick_point) == 4:
                cv2.line(frame, (int(pick_point[3][0]), int(pick_point[3][1])),
                        (int(pick_point[0][0]), int(pick_point[0][1])), (0, 255, 0), 2)

            cv2.imshow(info["pick_winname"], frame)
            # 将刚加入的pop出等待确认后再加入
            pick_point.pop()
            key = cv2.waitKey(0)
            if key == 13 & 0xFF: # 确认点加入
                pick_point.append(info["pick_point"])
                print(f"You have pick {len(pick_point):d} point.")

            if key == ord('z') & 0xFF: # 将上一次加入的点也删除（这次的也不要）
                if len(pick_point):
                    pick_point.pop()
                print("drop last")

            if key == ord('q') & 0xFF: # 直接退出标定，比如你来不及了
                cv2.destroyWindow(info["pick_winname"])
                cv2.destroyWindow(info["zoom_winname"])
                return False,None,None
            info["pick_flag"] = False
        else:
            cv2.waitKey(1)
        if len(pick_point) == 4:  # 四点全部选定完成
            print("four points have been picked.")
            cv2.destroyWindow(info["pick_winname"])
            cv2.destroyWindow(info["zoom_winname"])
            return pick_point

frame_dir,num,out_file = 'ImageL/', 10, 'corner_photo.txt'
points = []
for i in range(num):
    points.append(locate_pick(frame_dir, i))
with open(out_file, 'w') as f:
    for i in range(num):
        for j in range(4):
            f.write(f"{points[i][j][0]:.4f} {points[i][j][1]:.4f}\n")

