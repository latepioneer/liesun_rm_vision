#pragma once
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <chrono>

class Gyropose
{
public:
    /*@brief 四元数*/
    float q_0, q_1, q_2, q_3;

public:
    std::chrono::steady_clock::time_point receive_time;
    Gyropose(float *imu_data);
    Gyropose();
};

class Mat_time
{
private:
public:
    cv::Mat img;
    Gyropose gyro_pose;
    std::chrono::steady_clock::time_point start_time;
    Mat_time() {};
    Mat_time(cv::Mat *img);
    void copyTo(Mat_time &frame);
};