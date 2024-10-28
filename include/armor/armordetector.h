#pragma once
#include <opencv2/opencv.hpp>
#include <cmath>
#include "armorbox.h"
#include "camera.h"
#include "Mat_time.h"
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

class ArmorDetector
{
private:
    // ArmorBox last_target;

public:
    float yaw, pitch;
    LightBlobs lightblobs;
    ArmorBoxes armorboxes;
    ArmorState state = ArmorState::LOST;
    int lose_count;

private:
    bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j);
    void getBestArmor(ArmorBoxes &armorboxes);
    double distance(cv::Point2f first, cv::Point2f second); // 计算灯条中心的距离
    /*@brief 查看是否为上一次锁定的装甲板*/
    bool ifOldArmor();

public:
    cv::Mat img_preprocess(cv::Mat *img, int color); // 图像预处理
    void find_light(cv::Mat binary);                 // 寻找灯条
    /*
    @brief 灯条与装加板相匹配
    @param[out] 灯条集合
    @param[out] 装甲板集合
    @return 1为匹配成功，0为失败
    */
    bool matchArmorBoxes(LightBlobs &lightblobs, ArmorBoxes &armorboxes);
    void find_armor(); // 寻找合适的装甲板
    cv::Point3f pnp(ArmorBox armor);

    void getPitchYaw(cv::Point3f world_point, Gyropose gyro_pose);
    // void getArmorNum(ArmorBoxes& armor_boxes);
};

cv::Point3f camera_to_world(Eigen::Quaternionf q1, cv::Point3f point, cv::Point3f trans_offset);
