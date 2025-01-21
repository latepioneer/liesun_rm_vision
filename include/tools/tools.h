#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

cv::Point3f camera2world(Eigen::Quaternionf q, cv::Point3f camera_point, cv::Point3f trans_offset);
cv::Point3f world2camera(Eigen::Quaternionf q, cv::Point3f world_point, cv::Point3f trans_offset);