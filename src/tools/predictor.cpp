#include "predictor.h"
#include "tools.h"

CoordPredictor::CoordPredictor() : KF(std::make_shared<cv::KalmanFilter>())
{
    if (!KF)
    {
        std::cerr << "Failed to create KalmanFilter object." << std::endl;
        return;
    }
    assert(DP > 0 && MP > 0); // 确保 DP 和 MP 都大于 0
    KF->init(DP, MP, CP);
    dt = 1; // 初始化时间间隔
    // 初始化状态转移矩阵 F(x,y,z,vx,vy,vz)
    cv::setIdentity(KF->transitionMatrix, cv::Scalar::all(1));
    // 初始化测量矩阵 H
    KF->measurementMatrix = (cv::Mat_<float>(MP, DP) << 1, 0, 0, 0, 0, 0,
                             0, 0, 1, 0, 0, 0,
                             0, 0, 0, 0, 1, 0);

    // 初始化过程噪声协方差矩阵 Q
    KF->processNoiseCov = (cv::Mat_<float>(DP, DP) << 1, 0, 0, 0, 0, 0,
                           0, 100, 0, 0, 0, 0,
                           0, 0, 1, 0, 0, 0,
                           0, 0, 0, 100, 0, 0,
                           0, 0, 0, 0, 1, 0,
                           0, 0, 0, 0, 0, 100);

    // 初始化测量噪声协方差矩阵 R
    KF->measurementNoiseCov = (cv::Mat_<float>(MP, MP) << 1, 0, 0,
                               0, 1, 0,
                               0, 0, 1);

    // 初始化状态估计协方差矩阵 P
    cv::setIdentity(KF->errorCovPost, cv::Scalar::all(1));

    // 初始化状态估计向量 x
    KF->statePost = cv::Mat::zeros(DP, 1, CV_32F);
}

cv::Point3f CoordPredictor::predict(cv::Point3f coord, Gyropose gyro_pose)
{
    Eigen::Quaternionf  q(gyro_pose.q_0,gyro_pose.q_1,gyro_pose.q_2,gyro_pose.q_3);
    world_coord = camera2world(q,coord,cam2gyro);
    return predict(world_coord,gyro_pose.receive_time);
}


cv::Point3f  CoordPredictor::predict(cv::Point3f coord,std::chrono::steady_clock::time_point timestamp)
{
    cv::Mat correct_state;
    std::chrono::duration<float> duration = timestamp - last_t;
    float dt = duration.count();
    //std::cout<<dt<<std::endl;
    last_t = timestamp;
    cv::Mat measurement = (cv::Mat_<float>(3, 1) << coord.x, coord.y, coord.z);
    KF->transitionMatrix.at<float>(0,1) = dt;
    KF->transitionMatrix.at<float>(2,3) = dt;
    KF->transitionMatrix.at<float>(4,5) = dt;
    KF->predict();
    correct_state = KF->correct(measurement);
    correct_coord = cv::Point3f(correct_state.at<float>(0, 0), correct_state.at<float>(2, 0), correct_state.at<float>(4, 0));
    //std::cout<<correct_coord<<std::endl;
    pitch_time = compensate(correct_coord,dt);
    //std::cout<<"pitch_time"<<pitch_time<<std::endl;
    cv::Point3f world_next = predictNextpoint(correct_state,pitch_time.y+dt);
    return world_next;
}

cv::Point3f CoordPredictor::predictNextpoint(cv::Mat result,float dt)
{
    float t = dt;
    float x = result.at<float>(0,0)+t*result.at<float>(1,0);
    float y = result.at<float>(2,0)+t*result.at<float>(3,0);
    float z = result.at<float>(4,0)+t*result.at<float>(5,0);
    return cv::Point3f(x,y,z);
}

void CoordPredictor::initState(cv::Point3f coord,Gyropose gyro_pose)
{
    Eigen::Quaternionf  q(gyro_pose.q_0,gyro_pose.q_1,gyro_pose.q_2,gyro_pose.q_3);
    world_coord = camera2world(q,coord,cam2gyro);
    cv::Mat state = (cv::Mat_<float>(6, 1) << world_coord.x, 0, world_coord.y, 0, world_coord.z, 0);
    KF->statePost = state;
    last_t  = gyro_pose.receive_time;
}


double CoordPredictor::getflytime(double angle,cv::Point3f correct_spin,double T,double dt)
{
    double x = sqrt(correct_spin.x*correct_spin.x+correct_spin.y*correct_spin.y)/1000.0;
    return (exp(k*x)-1.0)/(k*bullet_speed*cos(angle));
}

cv::Point2f CoordPredictor::compensate(cv::Point3f correct_spin,float dt)
{
    correct_spin -= gun2cam;
    double dy,angle,y_actual;
    double t_actual = 0.0;
    double y_temp = correct_spin.z/1000.0;
    double y = y_temp;
    double x = sqrt(correct_spin.x*correct_spin.x+correct_spin.y*correct_spin.y)/1000.0;
    for(int i = 0;i < iter_num;i++)
    {
        angle = atan2(y_temp,x);
        t_actual = getflytime(angle,correct_spin,t_actual,dt);
        y_actual = bullet_speed*t_actual*sin(angle)-0.5*g*t_actual*t_actual;
        dy = y-y_actual;
        y = y_temp-dy;
        if(abs(dy)<0.001) break;
    }
    return cv::Point2f(angle/M_PI*180,t_actual);
}