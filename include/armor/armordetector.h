#pragma once
#include <opencv2/opencv.hpp>
#include <cmath>
#include "armorbox.h"
#include "camera.h"
#include "Mat_time.h"
#include "classify.h"
#include "predictor.h"
#include <Eigen/Dense>

#define red_blue_diff 60
#define blue_red_diff 160

using namespace std;

enum Color
{
    BLUE = 0,
    GREEN = 1,
    RED = 2
};

/**
 *@class ArmorDetector
 *@brief 装甲板探测
 */
class ArmorDetector
{
private:
    Classify classify;

    float roi_enlarge = 3;
    int lost_count = 0;
    cv::Rect roi_rect;///roi区域矩阵
    cv::Rect roi_temp;//暂存上一次识别装甲板的roi
    cv::Point offset;

    cv::Point3f last_world_point;
public:
    float yaw, pitch;
    LightBlobs lightblobs;
    ArmorBoxes armorboxes;
    ArmorState state = ArmorState::LOST;

    ArmorBox target;//目标装甲板
    ArmorBox last_target;//上一次目标装甲板
    cv::Mat roi;
    Mat_time src;

    CoordPredictor  predictor;

private:
    /**
     * @brief 查看是否为同一个装甲板的灯条
     * @param light_blob_i 左灯条
     * @param light_blob_j 右灯条
     * @return
     */
    bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j);
    bool getBestArmor(ArmorBoxes &armorboxes);
    /**
     * @brief 计算灯条中心的距离
     * @param 左灯条
     * @param 右灯条
     */
    double distance(cv::Point2f first, cv::Point2f second); // 计算灯条中心的距离
    /**
     * @brief 查看是否为上一次锁定的装甲板
     */
    bool ifOldArmor();

    void get_roi(float ratio);
    bool setRoi(Mat_time _src, cv::Rect &roi);

public:
    cv::Mat img_preprocess(int color); // 图像预处理
    bool find_light(cv::Mat binary);                 // 寻找灯条
    /**
     * @brief 灯条与装加板相匹配
     * @param[out] 灯条集合
     * @param[out] 装甲板集合
     * @return 1为匹配成功，0为失败
     */
    bool matchArmorBoxes(LightBlobs &lightblobs, ArmorBoxes &armorboxes);
    cv::Point3f pnp(ArmorBox armor);
    void getArmorNum(ArmorBoxes &armor_boxes);

    bool run(Mat_time _src,cv::Point2f &pitch_yaw);

    bool findArmorBox(ArmorBox &box);

    cv::Point2f getPitchYaw(ArmorBox temp_target,Gyropose gyro_pose);
    cv::Point2f getPitchYaw(cv::Point3f world_point,Gyropose gyro_pose);
    cv::Point2f calPredict(cv::Point3f world_predict,Gyropose gyro_pose);

};

