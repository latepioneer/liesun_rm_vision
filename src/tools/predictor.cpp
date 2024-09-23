#include "predictor.h"

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
    KF->transitionMatrix = (cv::Mat_<float>(DP, DP) << 1, 0, 0, dt, 0, 0,
                            0, 1, 0, 0, dt, 0,
                            0, 0, 1, 0, 0, dt,
                            0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 1);

    // 初始化测量矩阵 H
    KF->measurementMatrix = (cv::Mat_<float>(MP, DP) << 1, 0, 0, 0, 0, 0,
                             0, 1, 0, 0, 0, 0,
                             0, 0, 1, 0, 0, 0);

    // 初始化过程噪声协方差矩阵 Q
    KF->processNoiseCov = (cv::Mat_<float>(DP, DP) << 1, 0, 0, 0, 0, 0,
                           0, 1, 0, 0, 0, 0,
                           0, 0, 1, 0, 0, 0,
                           0, 0, 0, 100, 0, 0,
                           0, 0, 0, 0, 100, 0,
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

cv::Mat CoordPredictor::predictAndUpdate(const cv::Mat &measurement)
{
    // 预测步骤
    cv::Mat prediction = KF->predict();
    // 更新步骤
    cv::Mat residual = measurement - KF->measurementMatrix * prediction;
    cv::Mat S = KF->measurementMatrix * KF->errorCovPost * KF->measurementMatrix.t() + KF->measurementNoiseCov;
    cv::Mat K = KF->errorCovPost * KF->measurementMatrix.t() * S.inv();
    cv::Mat updatedState = prediction + K * residual;
    KF->errorCovPost = (cv::Mat::eye(DP, DP, CV_32F) - K * KF->measurementMatrix) * KF->errorCovPost;

    // 更新状态
    return KF->statePost = updatedState;
}

cv::Point3f CoordPredictor::predict(const cv::Point3f armor_xyz)
{
    cv::Mat armor = (cv::Mat_<float>(3, 1) << armor_xyz.x, armor_xyz.y, armor_xyz.z);
    cv::Mat armor_predict = this->predictAndUpdate(armor);
    this->dataPoints.x = armor_predict.at<float>(0, 0);
    this->dataPoints.y = armor_predict.at<float>(1, 0);
    this->dataPoints.z = armor_predict.at<float>(2, 0);
    return this->dataPoints;
}
