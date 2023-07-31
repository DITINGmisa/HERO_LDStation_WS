# include "car_infer.h"


TRTInfer_car::TRTInfer_car(const int device)
{
    cudaSetDevice(device);
}

TRTInfer_car::~TRTInfer_car()
{
}

void TRTInfer_car::car_nms_crossClass(std::vector<Yolo_car::Detection>& res, float *output, float conf_thresh, float nms_thresh, std::string enemy_color) 
{
    int det_size = sizeof(Yolo_car::Detection) / sizeof(float);
    std::map<float, std::vector<Yolo_car::Detection>> m;
    // 只检测敌方和未知，0 unkown，1 red，2 blue， 3 base
    float ignore_cls = 3;
    if(enemy_color == "red")
        ignore_cls = 2;
    else if(enemy_color == "blue")
        ignore_cls = 1;
    else
        ignore_cls = 3;
    for (int i = 0; i < output[0] && i < Yolo_car::MAX_OUTPUT_BBOX_COUNT; i++) 
    {
        if (output[1 + det_size * i + 4] <= conf_thresh || output[1 + det_size * i + 5] == 3 || output[1 + det_size * i + 5] == ignore_cls)
            continue;
        Yolo_car::Detection det;
        memcpy(&det, &output[1 + det_size * i], det_size * sizeof(float));
        if (m.count(0) == 0) m.emplace(0, std::vector<Yolo_car::Detection>());
        m[0].push_back(det);
    }
    // std::cout<<"m.size:"<<m.size()<<std::endl;
    for (auto it = m.begin(); it != m.end(); it++) 
    {
        //std::cout << it->second[0].class_id << " --- " << std::endl;
        auto& dets = it->second;
        // std::cout<<"dets.size:"<<dets.size()<<std::endl;
        std::sort(dets.begin(), dets.end(), car_cmp);
        for (size_t m = 0; m < dets.size(); ++m) {
            auto& item = dets[m];
            res.push_back(item);
            for (size_t n = m + 1; n < dets.size(); ++n) {
                if (car_iou(item.bbox, dets[n].bbox) > nms_thresh) {
                    dets.erase(dets.begin() + n);
                    --n;
                }
            }
            // std::cout<<"dets.size:"<<dets.size()<<std::endl;
        }
    }
    // std::cout<<"res.size:"<<res.size()<<std::endl;
}

bool TRTInfer_car::initModule(const std::string engine_file_path, const int batch_size, const int num_classes)
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

void TRTInfer_car::InferenceBase(std::vector<cv::Mat> &frames, std::vector<Yolo_car::Detection>& results, float confidence_threshold, float nms_threshold)
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
        std::cout << "car inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        std::vector<std::vector<Yolo_car::Detection>> batch_res(fcount);
        for (int b = 0; b < fcount; b++) {
            auto& res = batch_res[b];
            car_nms(res, &prob[b * this->output_size], confidence_threshold, confidence_threshold);
            std::cout<<"b:"<<b<<" res.size:"<<res.size()<<std::endl;
            for(int i=0;i<res.size();i++)
            {
                std::cout<<"class_id:"<<res[i].class_id<<" bbox:"<<res[i].bbox[0]<<","<<res[i].bbox[1]<<","<<res[i].bbox[2]<<","<<res[i].bbox[3]<<std::endl;
            }
        }
        for (int b = 0; b < fcount; b++) {
            auto& res = batch_res[b];
            cv::Mat img = imgs_buffer[b];
            for (size_t j = 0; j < res.size(); j++) 
            {
                cv::Rect r = car_get_rect(img, res[j].bbox);
                results.push_back(res[j]);
                cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }
            cv::Mat m;
            cv::resize(img, m, cv::Size(640, 480));
            if (b == 0)
                    cv::imshow("imgR", m);
                else
                    cv::imshow("imgL", m);
            cv::waitKey(1);
            cv::imwrite("_" + std::to_string(f - fcount + 1 + b) + ".jpg", img);
        }
        auto end1 = std::chrono::system_clock::now();
        std::cout << "car all time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count() << "ms" << std::endl;
        fcount = 0;
    }
}

// 专为雷达站设计，一次只传size为2的vector
void TRTInfer_car::Inference(std::vector<cv::Mat> &frames, std::vector<std::vector<Yolo_car::Detection>>& results, float confidence_threshold, float nms_threshold, std::vector<cv::Mat> &batch_roi, std::vector<int> &img_idx, std::string enemy_color, bool debug)
{
    if(!this->_is_inited)
    {
        this->gLogger.log(ILogger::Severity::kERROR,"Module not inited !");
    }
    static float prob[this->batch_size * this->output_size];
    std::vector<cv::Mat> imgs_buffer(this->batch_size);
    float *buffer_idx = (float*)this->buffers[inputIndex];
    for (int b = 0; b < this->batch_size; b++) 
    {
        cv::Mat img = frames[b].clone();
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
        std::cout << "car inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    }
    else
    {
        doInference(*this->context, this->stream, (void**)this->buffers, prob, this->batch_size);
    }
    
    std::vector<std::vector<Yolo_car::Detection>> batch_res(this->batch_size);
    cv::Mat img_to_draw; 
    for (int b = 0; b < this->batch_size; b++) 
    {
        auto& res = batch_res[b];
        car_nms_crossClass(res, &prob[b * this->output_size], confidence_threshold, nms_threshold, enemy_color);
        // std::cout<<"b:"<<b<<" res.size:"<<res.size()<<std::endl;
        // for(int i=0;i<res.size();i++)
        // {
        //     std::cout<<"class_id:"<<res[i].class_id<<" bbox:"<<res[i].bbox[0]<<","<<res[i].bbox[1]<<","<<res[i].bbox[2]<<","<<res[i].bbox[3]<<std::endl;
        // }
        if(debug)
            img_to_draw = imgs_buffer[b].clone();
        for (size_t j = 0; j < res.size(); j++) 
        {
            results[b].push_back(res[j]);
            cv::Rect r = car_get_rect(imgs_buffer[b], res[j].bbox);
            // std::cout<<b<<" "<<r.x<<" "<<r.y<<" "<<r.width<<" "<<r.height<<std::endl;
            cv::Mat roi = cv::Mat(imgs_buffer[b], r);
            batch_roi.push_back(roi.clone());
            img_idx.push_back((int)b*1000+j);//一张图检测出的车辆数量不会超过1000
            if(debug)
            {
                cv::rectangle(img_to_draw, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(img_to_draw, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0xFF, 0xFF, 0xFF), 3);
                cv::Mat m;
                cv::resize(img_to_draw, m, cv::Size(640, 480));
                if (b == 0)
                    cv::imshow("imgR", m);
                else
                    cv::imshow("imgL", m);
                cv::waitKey(1);
            }
        }
    }
}

void TRTInfer_car::unInitModule()
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
// 使用方法： rosrun infer car_demo [path to engine] [path to imgDir] [batch_size] [num_classes]
int main(int argc, char** argv)
{
    std::string engine_file = argv[1];
    std::string img_dir = argv[2];
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
    TRTInfer_car trt_infer(0);
    trt_infer.initModule(engine_file, batch_size, num_classes);
    std::vector<Yolo_car::Detection> results;
    trt_infer.InferenceBase(imgs, results, 0.5, 0.4);
    trt_infer.unInitModule();
    return 0;
}