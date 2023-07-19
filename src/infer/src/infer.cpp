#include "car/car_infer.h"
#include "armor/armor_infer.h"
#include <ros/ros.h>
#include "infer/img_result.h"
#include <iostream>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>

// 推理文件，使用TensorRT8.5.2.2推理yolov5s-6.2检测车辆，随后使用


//定义名称列表
std::string get_name(int index, std::string object)
{
    std::vector<std::string> car_name = {"car_unknown", "car_red_0", "car_blue_0"};
    std::vector<std::string> armor_name = {"car_blue_1", "car_blue_2", "car_blue_3", "car_blue_4", "car_blue_5", "car_blue_6",
                                "car_red_1", "car_red_2", "car_red_3", "car_red_4", "car_red_5", "car_red_6"};
    if(object == "car")
        return car_name[index];
    else if(object == "armor")
    {
        if(index == -1)//未识别到有效装甲板
            return "car_unknown";
        else
            return armor_name[index];
    } 
    else
        std::cerr << "object error!" << std::endl;

}
//获取车辆检测信息
infer::car_item get_car(int cols, int rows, Yolo_car::Detection result) 
{
    float l, r, t, b;
    float r_w = Yolo_car::INPUT_W / (cols * 1.0);
    float r_h = Yolo_car::INPUT_H / (rows * 1.0);
    if (r_h > r_w) {
        l = result.bbox[0] - result.bbox[2] / 2.f;
        r = result.bbox[0] + result.bbox[2] / 2.f;
        t = result.bbox[1] - result.bbox[3] / 2.f - (Yolo_car::INPUT_H - r_w * rows) / 2;
        b = result.bbox[1] + result.bbox[3] / 2.f - (Yolo_car::INPUT_H - r_w * rows) / 2;
        l = l / r_w;
        r = r / r_w;
        t = t / r_w;
        b = b / r_w;
    } else {
        l = result.bbox[0] - result.bbox[2] / 2.f - (Yolo_car::INPUT_W - r_h * cols) / 2;
        r = result.bbox[0] + result.bbox[2] / 2.f - (Yolo_car::INPUT_W - r_h * cols) / 2;
        t = result.bbox[1] - result.bbox[3] / 2.f;
        b = result.bbox[1] + result.bbox[3] / 2.f;
        l = l / r_h;
        r = r / r_h;
        t = t / r_h;
        b = b / r_h;
    }
    l = (std::max)(0.f, l);
    r = (std::min)((float)cols - 1.f, r);
    t = (std::max)(0.f, t);
    b = (std::min)((float)rows - 1.f, b);
    infer::car_item car;
    car.cls = get_name((int)result.class_id, "car");
    car.conf = result.conf;
    car.x0 = l;
    car.y0 = t;
    car.x1 = r;
    car.y1 = b;
    return car;
}
//获取装甲板检测信息
infer::armor_item get_armor(int cols, int rows, int x, int y, int car_id, Yolo_armor::Detection result)
{
    float l, r, t, b;
    infer::armor_item armor;
    if(result.class_id == -1)
    {
        l = 0;
        r = 0;
        t = 0;
        b = 0;
    }
    else
    {
        float r_w = Yolo_armor::INPUT_W / (cols * 1.0);
        float r_h = Yolo_armor::INPUT_H / (rows * 1.0);
        if (r_h > r_w) {
            l = result.bbox[0] - result.bbox[2] / 2.f;
            r = result.bbox[0] + result.bbox[2] / 2.f;
            t = result.bbox[1] - result.bbox[3] / 2.f - (Yolo_armor::INPUT_H - r_w * rows) / 2;
            b = result.bbox[1] + result.bbox[3] / 2.f - (Yolo_armor::INPUT_H - r_w * rows) / 2;
            l = l / r_w;
            r = r / r_w;
            t = t / r_w;
            b = b / r_w;
        } else {
            l = result.bbox[0] - result.bbox[2] / 2.f - (Yolo_armor::INPUT_W - r_h * cols) / 2;
            r = result.bbox[0] + result.bbox[2] / 2.f - (Yolo_armor::INPUT_W - r_h * cols) / 2;
            t = result.bbox[1] - result.bbox[3] / 2.f;
            b = result.bbox[1] + result.bbox[3] / 2.f;
            l = l / r_h;
            r = r / r_h;
            t = t / r_h;
            b = b / r_h;
        }
        l = (std::max)(0.f, l);
        r = (std::min)((float)cols - 1.f, r);
        t = (std::max)(0.f, t);
        b = (std::min)((float)rows - 1.f, b);   
    }
    armor.idx = car_id;
    armor.cls = result.class_id;
    armor.conf = result.conf;
    armor.x = l + x;
    armor.y = t + y;
    armor.w = r - l;
    armor.h = b - t;
    return armor;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "image_inference");
    ros::NodeHandle nh;

    //自定义msg
    ros::Publisher topic_publisher1 = nh.advertise<infer::img_result>("/infer/img1", 10);
    ros::Publisher topic_publisher2 = nh.advertise<infer::img_result>("/infer/img2", 10);
    infer::img_result img_results[2];//存储两个相机的检测结果
    
    // 定义car检测器
    TRTInfer_car car_infer(0);
    std::string car_engine = "";
    ros::param::get("yolov5_engine_car", car_engine);//获取模型路径
    float car_conf = 0.0;
    ros::param::get("car_conf", car_conf);//获取置信度阈值
    float car_nms = 0.0;
    ros::param::get("car_nms", car_nms);//获取nms阈值
    car_infer.initModule(car_engine, 2, 4);// TODO: 需要随模型修改

    // 定义armor检测器
    TRTInfer_armor armor_infer(0);
    std::string armor_engine = "";
    ros::param::get("yolov5_engine_armor", armor_engine);//获取模型路径
    std::string enemy_color = "";
    ros::param::get("enemy_color", enemy_color);//获取敌方颜色
    float armor_conf = 0.0;
    ros::param::get("armor_conf", armor_conf);//获取置信度阈值
    armor_infer.initModule(armor_engine, 4, 12);// TODO: 需要随模型修改

    // 创建共享内存
    int shm_fd1 = shm_open("/shared_memory1", O_RDWR | O_CREAT, 0666);
    if (shm_fd1 == -1) {
        std::cerr << "Failed to create shared memory1" << std::endl;
        return -1;
    }
    int shm_fd2 = shm_open("/shared_memory2", O_RDWR | O_CREAT, 0666);
    if (shm_fd2 == -1) {
        std::cerr << "Failed to create shared memory2" << std::endl;
        return -1;
    }

    // 设置共享内存大小
    static const int cols = 3088;
    static const int rows = 2064;
    static const int channels = 3;
    size_t shm_size = cols * rows * channels;
    if (ftruncate(shm_fd1, shm_size) == -1) {
        std::cerr << "Failed to set shared memory1 size" << std::endl;
        shm_unlink("/shared_memory1");
        return -1;
    }
    if (ftruncate(shm_fd2, shm_size) == -1) {
        std::cerr << "Failed to set shared memory2 size" << std::endl;
        shm_unlink("/shared_memory2");
        return -1;
    }

    // 映射共享内存到进程地址空间
    void* shm_ptr1 = mmap(NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd1, 0);
    if (shm_ptr1 == MAP_FAILED) {
        std::cerr << "Failed to map shared memory1" << std::endl;
        shm_unlink("/shared_memory1");
        return -1;
    }
    // 映射共享内存到进程地址空间
    void* shm_ptr2 = mmap(NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd2, 0);
    if (shm_ptr2 == MAP_FAILED) {
        std::cerr << "Failed to map shared memory2" << std::endl;
        shm_unlink("/shared_memory2");
        return -1;
    }

    // 此处默认batch_size为2，可根据实际情况修改，模型的batch_size也要相应修改
    std::vector<cv::Mat> imgs_buffer(2);
    std::vector<cv::Mat> batch_roi;//存放roi图像
    std::vector<int> img_idx;//存放图像索引
    std::vector<std::vector<Yolo_car::Detection>> car_results(2);//存放车辆检测结果
    std::vector<Yolo_armor::Detection> armor_results;//存放装甲板检测结果
    std::string fps;
    while(ros::ok()) 
    {
        auto time1 = std::chrono::system_clock::now();
        // 循环读取共享内存中的图像数据
        for(int b = 0; b < 2; b++) 
        {
            cv::Mat img(rows, cols, CV_8UC3);
            if(b == 0)
                memcpy(img.data, shm_ptr1, shm_size);
            else
                memcpy(img.data, shm_ptr2, shm_size);
            if (img.empty()) continue;
            imgs_buffer[b] = img;
        }
        auto time2 = std::chrono::system_clock::now();
        
        // Run inference
        ros::param::get("car_conf", car_conf);//获取置信度阈值
        ros::param::get("car_nms", car_nms);//获取nms阈值
        ros::param::get("armor_conf", armor_conf);//获取置信度阈值
        car_infer.Inference(imgs_buffer, car_results, car_conf, car_nms, batch_roi, img_idx, enemy_color,false);
        armor_infer.Inference(batch_roi, armor_results, armor_conf, enemy_color, false);
        // std::cout<<"armor_conf: "<<armor_conf<<std::endl;
        // std::cout<<car_results[0].size()<<" "<<car_results[1].size()<<" "<<armor_results.size()<<std::endl;
        assert(armor_results.size() == car_results[0].size() + car_results[1].size());
        assert(armor_results.size() == img_idx.size());
        auto time3 = std::chrono::system_clock::now();

        // 后处理
        for(int b = 0; b < 2; b++)
        {
            for(int i =0; i < car_results[b].size(); i++)
                img_results[b].carlist.push_back(get_car(cols, rows, car_results[b][i]));
        }
        for(int i = 0; i < armor_results.size(); i++)
        {
            int img_id = (int)img_idx[i]/1000;
            int car_id = (int)img_idx[i]%1000;
            // std::cout<<"img_id: "<<img_id<<" car_id: "<<car_id<<std::endl;
            // std::cout<<img_results[img_id].carlist.size()<<std::endl;
            // std::cout<<armor_results[i].class_id<<std::endl;
            img_results[img_id].carlist[car_id].conf = armor_results[i].conf;
            img_results[img_id].carlist[car_id].cls = get_name(armor_results[i].class_id, "armor");
            int w = img_results[img_id].carlist[car_id].x1 - img_results[img_id].carlist[car_id].x0;
            int h = img_results[img_id].carlist[car_id].y1 - img_results[img_id].carlist[car_id].y0;
            int l = img_results[img_id].carlist[car_id].x0;
            int t = img_results[img_id].carlist[car_id].y0;
            img_results[img_id].armorlist.push_back(get_armor(w, h, l, t, car_id, armor_results[i]));
        }

        // 发布消息
        topic_publisher1.publish(img_results[0]);
        topic_publisher2.publish(img_results[1]);
        
        // 清空数据
        batch_roi.clear();
        img_idx.clear();
        car_results[0].clear();
        car_results[1].clear();
        // car_results1.clear();
        armor_results.clear();
        img_results[0].carlist.clear();
        img_results[0].armorlist.clear();
        img_results[1].carlist.clear();
        img_results[1].armorlist.clear();
        auto time4 = std::chrono::system_clock::now();
        //fps 以3位小数显示
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "fps: " << (float)1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(time4 - time1).count() \
        << "\timg get time: " << std::chrono::duration_cast<std::chrono::milliseconds>(time2 - time1).count() << "ms  " \
        << "\tinfer time: " << std::chrono::duration_cast<std::chrono::milliseconds>(time3 - time2).count() << "ms  " \
        << "\tpost time: " << std::chrono::duration_cast<std::chrono::milliseconds>(time4 - time3).count() << "ms  \r";
        // std::cout << "--------------------------" << std::endl;
        ros::param::set("/infer/fps", (float)1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(time4 - time1).count());
        // fps = std::to_string(1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(time4 - time1).count());
    }

    // Destroy the engine
    car_infer.unInitModule();
    armor_infer.unInitModule();

    // 解除映射并关闭共享内存
    munmap(shm_ptr1, shm_size);
    munmap(shm_ptr2, shm_size);
    close(shm_fd1);
    close(shm_fd2);
    shm_unlink("/shared_memory1");
    shm_unlink("/shared_memory2");

    return 0;
}
