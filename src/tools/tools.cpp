#include "tools.h"

cv::Point3f camera2world(Eigen::Quaternionf q1,cv::Point3f point,cv::Point3f trans_offset)
{
    point+=trans_offset;
    Eigen::Quaternionf q2(0,point.z,-point.x,-point.y);
    Eigen::Quaternionf q3 = q1*q2*q1.inverse();
    return cv::Point3f(q3.x(),q3.y(),q3.z());   
}

cv::Point3f world2camera(Eigen::Quaternionf q1,cv::Point3f point,cv::Point3f trans_offset)
{
    Eigen::Quaternionf q2(0,point.x,point.y,point.z);
    Eigen::Quaternionf q3 = q1.inverse()*q2*q1;
    return cv::Point3f(-q3.y(),-q3.z(),q3.x())-trans_offset;
}