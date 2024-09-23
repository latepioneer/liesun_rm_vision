#pragma once
#include <opencv2/opencv.hpp>
#include <cmath>
#include "armorbox.h"
#include "camera.h"

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
public:
    LightBlobs lightblobs;
    ArmorBoxes armorboxes;

private:
    double distance(cv::Point2f first, cv::Point2f second); // 计算灯条中心的距离
public:
    cv::Mat img_preprocess(cv::Mat *img, int color); // 图像预处理
    void find_light(cv::Mat binary);                 // 寻找灯条
    void find_armor();                               // 寻找合适的装甲板
    cv::Point3f pnp(ArmorBox armor);
    // void getArmorNum(ArmorBoxes& armor_boxes);
};