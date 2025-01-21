#include "classify.h"
#include <filesystem>

void DataSet::numPreprocessing(std::vector<cv::Point2f> points, const cv::Mat *img)
{
    cv::Point2f src_points[4];
    std::copy(points.begin(), points.end(), src_points);
    cv::Mat rotation = cv::getPerspectiveTransform(src_points, dst_points);
    cv::Mat imgarmor(100, 100, img->type()); // 创建目标矩阵为 100x100 大小
    cv::warpPerspective(*img, imgarmor, rotation, imgarmor.size());
    cv::Rect roi(5, 0, 90, 100);
    cv::Mat roi_imgarmor = imgarmor(roi);
    cv::Mat gray_img;
    cv::cvtColor(roi_imgarmor, gray_img, cv::COLOR_BGR2GRAY);
    cv::threshold(gray_img, armor_num_img, 20, 255, cv::THRESH_BINARY);
}

bool DataSet::imagesSave(std::vector<cv::Point2f> points, const cv::Mat *img, std::string num_kind, std::string address)
{
    numPreprocessing(points, img);
    imshow("1",armor_num_img);
    // 检查路径并创建文件夹
    if (num_kind == "null")
        return false;
    std::string full_address = address + "/" + num_kind;
    if (!std::filesystem::exists(full_address))
    {
        if (!std::filesystem::create_directories(full_address))
        {
            return false; // 如果创建失败，返回 false
        }
    }

    // 生成图像文件名
    std::string imgname = full_address + "/" + std::to_string(num++) + ".jpg";

    // 达到保存次数上限时返回 false
    if (num >= 100)
    {
        return false;
    }

    // 保存图像并返回保存成功与否
    return cv::imwrite(imgname, this->armor_num_img);
}

int Classify::predit(std::vector<cv::Point2f> points, cv::Mat *img)
{
    numPreprocessing(points, img);
    cv::resize(armor_num_img, armor_num_img, cv::Size(28, 20));
    armor_num_img.convertTo(armor_num_img, CV_32F, 1.0 / 255.0);
    std::vector<float> input_tensor_values;
    input_tensor_values.assign(armor_num_img.begin<float>(), armor_num_img.end<float>());
    auto input_type_info = session.GetInputTypeInfo(0);
    auto input_shape_info = input_type_info.GetTensorTypeAndShapeInfo();

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    std::vector<int64_t> input_dims = {1, 28 * 20};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_tensor_values.data(), input_tensor_values.size(),
        input_dims.data(), input_dims.size());

    const std::vector<const char *> input_names = {input_name_str.c_str()};
    const std::vector<const char *> output_names = {output_name_str.c_str()};

    auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_names.data(), &input_tensor, 1, output_names.data(), 1);

    float *output_data = output_tensors[0].GetTensorMutableData<float>();
    int max_index = std::max_element(output_data, output_data + 8) - output_data;
    size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

    // 计算最大概率
    auto max_prob_iter = std::max_element(output_data, output_data + output_size);
    float max_prob = *max_prob_iter;
    if(max_prob>0.95)
    {
        return max_index;
    }
    else return 8;
}