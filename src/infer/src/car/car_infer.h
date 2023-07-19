#ifndef CAR_INFER_H
#define CAR_INFER_H

#include <iostream>
#include <chrono>
#include <cmath>
#include "../yolov5/cuda_utils.h"
#include "../yolov5/logging.h"
#include "../yolov5/utils.h"
#include "../yolov5/calibrator.h"
#include "../yolov5/preprocess.h"
#include "car_common.hpp"
#include "yololayer_car.h"

using namespace Yolo_car;
using namespace nvinfer1;
/**
 * @brief TRT推理
 * 高性能TRT YOLOX推理模块
 */
class TRTInfer_car
{
private:
    const char *INPUT_BLOB_NAME = "images";
    const char *OUTPUT_BLOB_NAME = "output";
    static const int batch_size = 2;
    static const int num_classes = Yolo_car::CLASS_NUM;
    static const int input_h = Yolo_car::INPUT_H;
    static const int input_w = Yolo_car::INPUT_W;

private:
    Logger gLogger;
    IRuntime *runtime;
    ICudaEngine *engine;
    IExecutionContext *context;
    float *buffers[2];
    static const int output_size = Yolo_car::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo_car::Detection) / sizeof(float) + 1;  
    int inputIndex = -1;
    int outputIndex = -1;
    uint8_t *img_host = nullptr;
    uint8_t *img_device = nullptr;
    cudaStream_t stream = nullptr;
    float *output;
    int max_input_size = 3088*3088;

private:
    int inter_frame_compensation = 0;
    bool _is_inited = false;

private:
    // 执行推理
    void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* output, int batchSize) {
        // infer on the batch asynchronously, and DMA output back to host
        context.enqueue(batchSize, buffers, stream, nullptr);
        CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * this->output_size * sizeof(float), cudaMemcpyDeviceToHost, stream));
        cudaStreamSynchronize(stream);
    }
    // car的nms，跨类别，检测敌方和unkown
    void car_nms_crossClass(std::vector<Yolo_car::Detection>& res, float *output, float conf_thresh, float nms_thresh = 0.5, std::string enemy_color = "all");

public:
    /**
     * @brief 构造函数
     * @param device
     * 使用的GPU索引
     */
    TRTInfer_car(const int device);
    ~TRTInfer_car();

    /**
     * @brief 初始化TRT模型
     * @param engine_file_path
     * engine路径
     * @param batch_size
     * 推理时使用的batch_size,输入图片数量不可大于此设置值，此设定值不可大于构建引擎时应用的maxBatchSize，最佳设定值为maxBatchSize/2
     * @param num_classes
     * num_classes设定值，类别数量
     */
    bool initModule(const std::string engine_file_path, const int batch_size, const int num_classes);
    /**
     * @brief 反初始化TRT模型，释放显存
     */
    void unInitModule();
    /**
     * @brief 执行推理
     * @param frames
     * 需要推理的图像序列，图像数量决定推理时batch_size，不可大于初始化模型时指定的batch_size
     * @param results
     * 推理结果
     * @param confidence_threshold
     * 置信度阈值
     * @param nms_threshold
     * 非极大值抑制阈值
     * 为了少一步逻辑判断，这里写了两个版本
     * @param batch_roi
     * 用于存储batch中每张图片的车辆roi，传给装甲板检测器检测
     * @param img_idx
     * 用于存储batch中每张图片的索引，表示每个装甲板检测结果对应的图片
     */
    void InferenceBase(std::vector<cv::Mat> &frames, std::vector<Yolo_car::Detection>& results, float confidence_threshold, float nms_threshold);
    void Inference(std::vector<cv::Mat> &frames, std::vector<std::vector<Yolo_car::Detection>>& results, float confidence_threshold, float nms_threshold, std::vector<cv::Mat> &batch_roi, std::vector<int> &img_idx, std::string enemy_color, bool debug);
};


#endif //CAR_INFER_H