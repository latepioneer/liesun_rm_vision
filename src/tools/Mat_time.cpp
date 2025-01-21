#include "Mat_time.h"

Gyropose::Gyropose(float *imu_data)
{
    q_0 = imu_data[0];
    q_1 = imu_data[1];
    q_2 = imu_data[2];
    q_3 = imu_data[3];
    receive_time = std::chrono::steady_clock::now();
}

Gyropose::Gyropose()
{
    q_0 = 0;
    q_1 = 0;
    q_2 = 0;
    q_3 = 0;
}

Mat_time::Mat_time(cv::Mat *img)
{
    this->img = *img;
    this->start_time = std::chrono::steady_clock::now();
}

void Mat_time::copyTo(Mat_time &frame)
{
    img.copyTo(frame.img);
    frame.gyro_pose = gyro_pose;
}