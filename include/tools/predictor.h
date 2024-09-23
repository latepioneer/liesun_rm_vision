#pragma once
#include <opencv2/opencv.hpp>

class CoordPredictor
{
private:
    std::shared_ptr<cv::KalmanFilter> KF;
    int DP = 6; // 状态向量维度
    int MP = 3; // 测量向量维度
    int CP = 0; // 控制向量维度
    double dt;  // 时间间隔

    cv::Point3f dataPoints; // 存储预测点

public:
    CoordPredictor();
    /*
        @param LKF预测
    */
    cv::Mat predictAndUpdate(const cv::Mat &measurement);
    cv::Point3f predict(const cv::Point3f armor_xyz);
};