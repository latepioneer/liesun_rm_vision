#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

class Camera
{
private:
    void *handle;
    static const cv::Mat cameraMatrix;
    static const cv::Mat distCoeffs;
    cv::VideoCapture video; // 移除初始化，使用构造函数初始化

    friend class ArmorDetector;
    friend class Task;

private:
    void camera_init();

public:
    Camera(); // 构造函数
    cv::Mat get_pic();
    ~Camera(); // 析构函数
};
