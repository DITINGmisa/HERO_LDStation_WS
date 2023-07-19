#ifndef YOLOV5_CAR_H
#define YOLOV5_CAR_H

#include <iostream>
#include <chrono>
#include <cmath>
#include "../yolov5/cuda_utils.h"
#include "../yolov5/logging.h"
#include "../yolov5/utils.h"
#include "../yolov5/calibrator.h"
#include "../yolov5/preprocess.h"
#include "car_common.hpp"

#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.15
#define BATCH_SIZE 2
#define MAX_IMAGE_INPUT_SIZE_THRESH 3088*3088 // ensure it exceed the maximum size in the input images !
const char* INPUT_BLOB_NAME = "images";
const char* OUTPUT_BLOB_NAME = "output";
static Logger gLogger;
// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo_car::INPUT_H;
static const int INPUT_W = Yolo_car::INPUT_W;
static const int CLASS_NUM = Yolo_car::CLASS_NUM;
static const int OUTPUT_SIZE = Yolo_car::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo_car::Detection) / sizeof(float) + 1;  // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1


static int get_width(int x, float gw, int divisor = 8) {
    return int(ceil((x * gw) / divisor)) * divisor;
}

static int get_depth(int x, float gd) {
    if (x == 1) return 1;
    int r = round(x * gd);
    if (x * gd - int(x * gd) == 0.5 && (int(x * gd) % 2) == 0) {
        --r;
    }
    return std::max<int>(r, 1);
}

ICudaEngine* build_engine(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt, float& gd, float& gw, std::string& wts_name);
ICudaEngine* build_engine_p6(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt, float& gd, float& gw, std::string& wts_name);
void APIToModel(unsigned int maxBatchSize, IHostMemory** modelStream, bool& is_p6, float& gd, float& gw, std::string& wts_name);
void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* output, int batchSize);
bool parse_args(int argc, char** argv, std::string& wts, std::string& engine, bool& is_p6, float& gd, float& gw, std::string& img_dir);

#endif //YOLOV5_CAR_H