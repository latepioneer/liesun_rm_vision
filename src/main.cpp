#include <opencv2/opencv.hpp>
#include <iostream>
#include "my_thread.h"

#include <string>
using namespace std;
using namespace cv;

// int main()
// {
//     Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntime");
//     std::string model_path = "../../armor_classify/saved_model/my_model.onnx";

//     Ort::SessionOptions session_options;
//     session_options.SetIntraOpNumThreads(1);
//     session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
//     Ort::Session session(env, model_path.c_str(), session_options);

//     Ort::AllocatorWithDefaultOptions allocator;
//     auto input_name = session.GetInputNameAllocated(0, allocator);
//     std::string input_name_str = input_name.get();

//     auto output_name = session.GetOutputNameAllocated(0, allocator);
//     std::string output_name_str = output_name.get();

//     cv::Mat img = cv::imread("/home/wtl/codefield/armor_classify/47.jpg", cv::IMREAD_GRAYSCALE);
//     cv::resize(img, img, cv::Size(28, 20));
//     img.convertTo(img, CV_32F, 1.0 / 255.0);

//     std::vector<float> input_tensor_values;
//     input_tensor_values.assign(img.begin<float>(), img.end<float>());
//     auto input_type_info = session.GetInputTypeInfo(0);
//     auto input_shape_info = input_type_info.GetTensorTypeAndShapeInfo();

//     Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
//     std::vector<int64_t> input_dims = {1, 28 * 20};
//     Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
//         memory_info, input_tensor_values.data(), input_tensor_values.size(),
//         input_dims.data(), input_dims.size());

//     const std::vector<const char *> input_names = {input_name_str.c_str()};
//     const std::vector<const char *> output_names = {output_name_str.c_str()};

//     auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_names.data(), &input_tensor, 1, output_names.data(), 1);

//     float *output_data = output_tensors[0].GetTensorMutableData<float>();
//     int max_index = std::max_element(output_data, output_data + 8) - output_data;
//     size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

//     // 计算最大概率
//     auto max_prob_iter = std::max_element(output_data, output_data + output_size);
//     float max_prob = *max_prob_iter;
//     if (max_prob > 0.9)
//     {
//         std::cout << "result: " << max_index << std::endl;
//     }

//     // float *output_data = output_tensors[0].GetTensorMutableData<float>();
//     // size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
//     // std::cout << "推理结果: " << output_data[0] << std::endl;

//     return 0;
// }
int main()
{

    Task task;
    if (!task.init())
        cout << "connect error" << endl;
    thread t1(&Task::camera_task, &task);
    thread t2(&Task::get_armor_task, &task);
    // thread t3(&Task::get_uart_task, &task);
    // thread t4(&Task::send_uart_task, &task);
    //  thread t5(&Task::InterGyroPose, &task);
    t1.join();
    t2.join();
    // t3.join();
    // t4.join();
    //  t5.join();
    //   Task task(2);
    //   task.camera_task();
    //   task.get_armor_task();

    // task.enqueue([&task]
    //              { task.camera_task(); });

    // task.enqueue([&task]
    //              { task.get_armor_task(); });
    // comm_service com;
    // com.CommOpen("/dev/ttyACM0");
    // com.CommInit(115200, 0, 8, 1, 'N');
    // ReceivePacket rdata;
    // com.CommRecv(rdata, sizeof(ReceivePacket));
    // cout << rdata.q[0] << endl;
    return 0;
}
