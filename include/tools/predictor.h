#pragma once
#include <opencv2/opencv.hpp>
#include "Mat_time.h"
#include <Eigen/Dense>

/*@brief 装甲板中心位置预测*/
class CoordPredictor
{
private:
    std::shared_ptr<cv::KalmanFilter> KF;
    int DP = 6; // 状态向量维度
    int MP = 3; // 测量向量维度
    int CP = 0; // 控制向量维度
    double dt;  // 时间间隔

    double bullet_speed = 14.5;

    double k = 0.0402;
    double g = 9.75;
    int iter_num = 20;
    std::chrono::steady_clock::time_point last_t;
public:
    cv::Point3f cam2gyro = cv::Point3f(-30,-20.0,0);//相机坐标系
    cv::Point3f gun2cam = cv::Point3f(50.,0,-40);//世界坐标系
    cv::Point2f pitch_time;
    cv::Point3f world_coord;
    cv::Point3f correct_coord;


public:
    CoordPredictor();
    /*
        @param LKF预测
    */
    cv::Point3f predict(cv::Point3f coord, Gyropose gyro_pose);
    cv::Point3f predict(cv::Point3f coord,std::chrono::steady_clock::time_point timestamp);
    cv::Point3f predictNextpoint(cv::Mat result,float dt);
    void initState(cv::Point3f coord,Gyropose gyro_pose);
    double getflytime(double angle,cv::Point3f correct_spin,double T,double dt);
    cv::Point2f compensate(cv::Point3f correct_spin,float dt);
};