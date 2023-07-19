# include "armor_infer.h"

Yolo_armor::Detection TRTInfer_armor::armor_nms_one(float *output, float conf_thresh, std::string enemy_color) 
{
    // 装甲板检测需要输出置信度最高的一个框，所以这里只取了一个框
    int det_size = sizeof(Yolo_armor::Detection) / sizeof(float);
    Yolo_armor::Detection max;
    max.conf = -1;
    max.class_id = -1;
    for(int i = 0; i < 4; i++)
        max.bbox[i] = 0.0;
    for (int i = 0; i < output[0] && i < Yolo_armor::MAX_OUTPUT_BBOX_COUNT; i++) 
    {
        if (output[1 + det_size * i + 4] <= conf_thresh) continue;
        Yolo_armor::Detection det;
        memcpy(&det, &output[1 + det_size * i], det_size * sizeof(float));
        if (det.conf > max.conf) {
            max = det;
        }
    }
    return max;
}

TRTInfer_armor::TRTInfer_armor(const int device)
{
    cudaSetDevice(device);
}

TRTInfer_armor::~TRTInfer_armor()
{
}


bool TRTInfer_armor::initModule(const std::string engine_file_path, const int batch_size, const int num_classes)
{
    assert(batch_size > 0 && num_classes > 0);
    assert(this->batch_size == batch_size);
    assert(this->num_classes == num_classes);
    char *trtModelStream{nullptr};
    size_t size{0};
    std::ifstream file(engine_file_path, std::ios::binary);
    if (file.good())
    {
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
    }
    else
    {
        this->gLogger.log(ILogger::Severity::kERROR, "Engine bad file");
        return false;
    }
    this->runtime = createInferRuntime(this->gLogger);
    assert(runtime != nullptr);
    this->engine = this->runtime->deserializeCudaEngine(trtModelStream, size);
    assert(this->engine != nullptr);
    this->context = this->engine->createExecutionContext();
    assert(context != nullptr);
    delete trtModelStream;
    
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    this->inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    this->outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    
    if (this->inputIndex == -1 || this->outputIndex == -1)
    {
        this->gLogger.log(ILogger::Severity::kERROR, "Uncorrect Input/Output tensor name");
        delete context;
        delete engine;
        return false;
    }
    CUDA_CHECK(cudaStreamCreate(&this->stream));
    CUDA_CHECK(cudaMalloc((void**)&this->buffers[this->inputIndex], this->batch_size * 3 * this->input_h * this->input_w * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)&this->buffers[this->outputIndex], this->batch_size * this->output_size * sizeof(float)));
    // prepare input data cache in pinned memory 
    CUDA_CHECK(cudaMallocHost((void **)&this->img_host, this->max_input_size * 3));
    CUDA_CHECK(cudaMalloc((void **)&this->img_device, this->max_input_size * 3));
    this->output = (float *)malloc(this->batch_size * this->output_size * sizeof(float));
    this->_is_inited = true;
    return true;
}

void TRTInfer_armor::InferenceBase(std::vector<cv::Mat> &frames, std::vector<Yolo_armor::Detection>& results, float confidence_threshold, float nms_threshold)
{
    if(!this->_is_inited)
    {
        this->gLogger.log(ILogger::Severity::kERROR,"Module not inited !");
    }
    static float prob[this->batch_size * this->output_size];
    int fcount = 0;
    std::vector<cv::Mat> imgs_buffer(this->batch_size);
    for (int f = 0; f < (int)frames.size(); f++) 
    {
        fcount++;
        if (fcount < this->batch_size && f + 1 != (int)frames.size()) continue;
        auto start1 = std::chrono::system_clock::now();
        float *buffer_idx = (float*)this->buffers[inputIndex];
        for (int b = 0; b < fcount; b++) {
            cv::Mat img = frames[f - fcount + 1 + b];
            if (img.empty()) continue;
            imgs_buffer[b] = img;
            size_t size_image = img.cols * img.rows * 3;
            size_t size_image_dst = this->input_h * this->input_w * 3;
            //copy data to pinned memory
            memcpy(this->img_host, img.data, size_image);
            //copy data to device memory
            CUDA_CHECK(cudaMemcpyAsync(this->img_device, this->img_host, size_image, cudaMemcpyHostToDevice, this->stream));
            preprocess_kernel_img(this->img_device, img.cols, img.rows, buffer_idx, this->input_h, this->input_w, this->stream);
            buffer_idx += size_image_dst;
        }
        // Run inference
        auto start = std::chrono::system_clock::now();
        doInference(*this->context, this->stream, (void**)this->buffers, prob, this->batch_size);
        auto end = std::chrono::system_clock::now();
        std::cout << "armor inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        std::vector<std::vector<Yolo_armor::Detection>> batch_res(fcount);
        for (int b = 0; b < fcount; b++) {
            auto& res = batch_res[b];
            armor_nms(res, &prob[b * this->output_size], confidence_threshold, nms_threshold);
        }
        for (int b = 0; b < fcount; b++) {
            auto& res = batch_res[b];
            cv::Mat img = imgs_buffer[b];
            for (size_t j = 0; j < res.size(); j++) 
            {
                cv::Rect r = armor_get_rect(img, res[j].bbox);
                results.push_back(res[j]);
                cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }
            cv::imwrite("_" + std::to_string(f - fcount + 1 + b) + ".jpg", img);
        }
        auto end1 = std::chrono::system_clock::now();
        std::cout << "armor all time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count() << "ms" << std::endl;
        fcount = 0;
    }
}

void TRTInfer_armor::Inference(std::vector<cv::Mat> &frames, std::vector<Yolo_armor::Detection>& results, float confidence_threshold, std::string enemy_color, bool debug)
{
    if(!this->_is_inited)
    {
        this->gLogger.log(ILogger::Severity::kERROR,"Module not inited !");
    }
    static float prob[this->batch_size * this->output_size];
    int fcount = 0;
    std::vector<cv::Mat> imgs_buffer(this->batch_size);
    for (int f = 0; f < (int)frames.size(); f++) 
    {
        fcount++;
        if (fcount < this->batch_size && f + 1 != (int)frames.size()) continue;
        float *buffer_idx = (float*)this->buffers[inputIndex];
        for (int b = 0; b < fcount; b++) {
            cv::Mat img = frames[f - fcount + 1 + b];
            if (img.empty()) continue;
            imgs_buffer[b] = img;
            size_t size_image = img.cols * img.rows * 3;
            size_t size_image_dst = this->input_h * this->input_w * 3;
            //copy data to pinned memory
            memcpy(this->img_host, img.data, size_image);
            //copy data to device memory
            CUDA_CHECK(cudaMemcpyAsync(this->img_device, this->img_host, size_image, cudaMemcpyHostToDevice, this->stream));
            preprocess_kernel_img(this->img_device, img.cols, img.rows, buffer_idx, this->input_h, this->input_w, this->stream);
            buffer_idx += size_image_dst;
        }
        // Run inference
        if(debug)
        {
            auto start = std::chrono::system_clock::now();
            doInference(*this->context, this->stream, (void**)this->buffers, prob, this->batch_size);
            auto end = std::chrono::system_clock::now();
            std::cout << "armor inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        }
        else
        {
            doInference(*this->context, this->stream, (void**)this->buffers, prob, this->batch_size);
        }        
        std::vector<std::vector<Yolo_armor::Detection>> batch_res(fcount);
        for (int b = 0; b < fcount; b++) 
        {
            Yolo_armor::Detection res = armor_nms_one(&prob[b * this->output_size], confidence_threshold, enemy_color);
            results.push_back(res);
            if(debug)
            {
                cv::Rect r = armor_get_rect(imgs_buffer[b], res.bbox);
                cv::rectangle(imgs_buffer[b], r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(imgs_buffer[b], std::to_string((int)res.class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                cv::imshow("armor", imgs_buffer[b]);
                cv::Mat img = imgs_buffer[b];
                // cv::imwrite("./imgs/" + std::to_string(f - fcount + 1 + b) + ".jpg", imgs_buffer[b]);
                cv::waitKey(1);
            }
        }
        fcount = 0;
    }
}

void TRTInfer_armor::unInitModule()
{
    this->_is_inited = false;
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(img_device));
    CUDA_CHECK(cudaFreeHost(img_host));
    CUDA_CHECK(cudaFree(this->buffers[this->inputIndex]));
    CUDA_CHECK(cudaFree(this->buffers[this->outputIndex]));
    // Destroy the engine
    this->context->destroy();
    this->engine->destroy();
    this->runtime->destroy();
}

int main(int argc, char** argv)
{
    std::string img_dir = argv[1];
    std::string engine_file = argv[2];
    int batch_size = std::atoi(argv[3]);
    int num_classes = std::atoi(argv[4]);
    std::vector<cv::String> img_names;
    cv::glob(img_dir, img_names);
    std::vector<cv::Mat> imgs;
    for (int i = 0; i < img_names.size(); i++)
    {
        cv::Mat img = cv::imread(img_names[i]);
        imgs.push_back(img);
    }
    TRTInfer_armor trt_infer(0);
    trt_infer.initModule(engine_file, batch_size, num_classes);
    std::vector<Yolo_armor::Detection> results;
    trt_infer.InferenceBase(imgs, results, 0.5, 0.4);
    trt_infer.unInitModule();
    return 0;
}