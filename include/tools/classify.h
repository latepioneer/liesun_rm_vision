#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <onnxruntime_cxx_api.h>

class DataSet
{
private:
    int num = 0; // 保存的图像数量
    cv::Point2f dst_points[4] = {
        cv::Point2f(0, 0),
        cv::Point2f(0, 100),
        cv::Point2f(100, 0),
        cv::Point2f(100, 100),
    };

protected:
    /**
     * @brief 存储装甲板数字图片
     */
    cv::Mat armor_num_img;
    /**
     * @brief 装甲板数字预处理
     * @param points 装甲板四个点位
     * @param img 原图像
     */
    void numPreprocessing(std::vector<cv::Point2f> points, const cv::Mat *img);

public:
    /**
     * @brief 处理后的图像保存
     * @param address 保存地址，默认为当前文件夹下创建的imgs文件夹
     * @return 0表示失败，1表示成功
     */
    bool imagesSave(
        std::vector<cv::Point2f> points, const cv::Mat *img,
        std::string num_kind, std::string address = "../imgs");
};

class Classify : public DataSet
{
private:
    Ort::Env env;                                                              // ONNX Runtime 环境
    std::string model_path = "../../armor_classify/saved_model/my_model.onnx"; // 模型路径
    Ort::Session session;                                                      // 会话对象
    Ort::AllocatorWithDefaultOptions allocator;
    std::string input_name_str, output_name_str;

public:
    // 构造函数
    Classify()
        : env(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntime"),
          session(env, model_path.c_str(), Ort::SessionOptions{nullptr})

    {
        auto input_name = session.GetInputNameAllocated(0, allocator);
        input_name_str = input_name.get();

        auto output_name = session.GetOutputNameAllocated(0, allocator);
        output_name_str = output_name.get();
    }
    int predit(std::vector<cv::Point2f> points, cv::Mat *img);
};