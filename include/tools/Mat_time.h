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

class Mat_time : public cv::Mat
{
private:
public:
    cv::Mat img;
    Gyropose gyro_pose;
    std::chrono::steady_clock::time_point start_time;
    Mat_time() {};
    Mat_time(cv::Mat *img);
    /*@param 获得该图像的处理时间*/
    void get_process_time();
    void get_img();
};

class Point_time : public cv::Point3f
{
private:
    std::chrono::steady_clock::time_point local_time;

public:
    cv::Point3f world_point;

public:
    Point_time(cv::Point3f world_point):world_point(world_point){}
};

cv::Point3f cameratowoprld(Eigen::Quaternionf q, cv::Point3f, cv::Point3f trans_offset);