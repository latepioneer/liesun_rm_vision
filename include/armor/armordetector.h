#pragma once
#include <opencv2/opencv.hpp>
#include <cmath>
#include "armorbox.h"
#include "camera.h"
#include "Mat_time.h"
#include "classify.h"
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

public:
    float yaw, pitch;
    LightBlobs lightblobs;
    ArmorBoxes armorboxes;
    ArmorState state = ArmorState::LOST;
    /**
     * @brief 记录丢失的帧数
     */
    int lost_count;
    /**
     * @brief 记录跟随的帧数
     */
    int lock_count;
    /**
     * @brief 选取roi区域，加快处理图像的时间
     */
    cv::Mat ROI;
    /**
     * @brief 记录上一次锁定的装甲板
     */
    ArmorBox last_target;

private:
    /**
     * @brief 查看是否为同一个装甲板的灯条
     * @param light_blob_i 左灯条
     * @param light_blob_j 右灯条
     * @return
     */
    bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j);
    void getBestArmor(ArmorBoxes &armorboxes);
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

public:
    cv::Mat img_preprocess(cv::Mat *img, int color); // 图像预处理
    void find_light(cv::Mat binary);                 // 寻找灯条
    /**
     * @brief 灯条与装加板相匹配
     * @param[out] 灯条集合
     * @param[out] 装甲板集合
     * @return 1为匹配成功，0为失败
     */
    bool matchArmorBoxes(LightBlobs &lightblobs, ArmorBoxes &armorboxes, cv::Mat *img);
    void find_armor(cv::Mat *img); // 寻找合适的装甲板
    cv::Point3f pnp(ArmorBox armor);

    void getPitchYaw(cv::Point3f world_point, Gyropose gyro_pose);
    void getArmorNum(ArmorBoxes &armor_boxes, cv::Mat *img);
};

cv::Point3f camera_to_world(Eigen::Quaternionf q1, cv::Point3f point, cv::Point3f trans_offset);
