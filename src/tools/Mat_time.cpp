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

cv::Point3f cameratowoprld(Eigen::Quaternionf q, cv::Point3f point, cv::Point3f trans_offset)
{
    point += trans_offset;
    Eigen::Quaternionf p(0, point.z, -point.x, -point.y);

    Eigen::Quaternionf result = q * p * q.inverse();
    return cv::Point3f(result.x(), result.y(), result.z());
}