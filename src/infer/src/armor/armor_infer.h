#ifndef ARMOR_INFER_H
#define ARMOR_INFER_H

#include <iostream>
#include <chrono>
#include <cmath>
#include "../yolov5/cuda_utils.h"
#include "../yolov5/logging.h"
#include "../yolov5/utils.h"
#include "../yolov5/calibrator.h"
#include "../yolov5/preprocess.h"
#include "armor_common.hpp"
#include "yololayer_armor.h"

using namespace Yolo_armor;
using namespace nvinfer1;
/**
 * @brief TRT推理
 * 高性能TRT YOLOX推理模块
 */
class TRTInfer_armor
{
private:
    const char *INPUT_BLOB_NAME = "images";
    const char *OUTPUT_BLOB_NAME = "output";
    static const int batch_size = 4;
    static const int num_classes = Yolo_armor::CLASS_NUM;
    static const int input_h = Yolo_armor::INPUT_H;
    static const int input_w = Yolo_armor::INPUT_W;

private:
    Logger gLogger;
    IRuntime *runtime;
    ICudaEngine *engine;
    IExecutionContext *context;
    float *buffers[2];
    static const int output_size = Yolo_armor::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo_armor::Detection) / sizeof(float) + 1;  
    int inputIndex = -1;
    int outputIndex = -1;
    uint8_t *img_host = nullptr;
    uint8_t *img_device = nullptr;
    cudaStream_t stream = nullptr;
    float *output;
    int max_input_size = 640*480;

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
    // 装甲板nms，只需要输出一个置信度最高的敌方装甲板
    Yolo_armor::Detection armor_nms_one(float *output, float conf_thresh, std::string enemy_color);
    

public:
    /**
     * @brief 构造函数
     * @param device
     * 使用的GPU索引
     */
    TRTInfer_armor(const int device);
    ~TRTInfer_armor();

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
     * @param enemy_color
     * 敌方颜色，blue,red或者all
     * 实际识别装甲板用不着nms_threshold，因为只看一个装甲板，所以直接用置信度最高的敌方就行
     */
    void InferenceBase(std::vector<cv::Mat> &frames, std::vector<Yolo_armor::Detection>& results, float confidence_threshold, float nms_threshold);
    void Inference(std::vector<cv::Mat> &frames, std::vector<Yolo_armor::Detection>& results, float confidence_threshold, std::string enemy_color, bool debug);
};

#endif //ARMOR_INFER_H