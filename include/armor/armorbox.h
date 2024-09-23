#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

#define light_min_wh_ratio 0.1  // 灯条最小宽高比
#define light_max_wh_ratio 0.25 // 灯条最大宽高比
#define light_min_ch_ratio 1    // 灯条中心距离与高度之比最小值
#define light_max_ch_ratio 4.2  // 灯条中心距离与高度之比最大值
#define light_max_cdif_ratio 1  // 中心距离之差和高度之比
#define light_angle_dif 20      // 两灯之间的夹角

enum
{
    BIG_ARMOR = 1,
    SMALL_ARMOR = 2
};

class LightBlob
{
private:
    cv::RotatedRect rrect;           // 灯条的旋转矩阵
    std::vector<cv::Point2f> points; // 点位顺序
    double width, height;

    friend class ArmorDetector;
    friend class ArmorBox;

public:
    LightBlob(cv::RotatedRect rrect, double width, double height);
    // LightBlob& operator=(const LightBlob& other);
};
typedef std::vector<LightBlob> LightBlobs;

class ArmorBox
{
public:
    cv::RotatedRect rect;      // 装甲板旋转矩形
    LightBlobs light_Blobs[2]; // 装甲板的左右灯条 [0]左 [1]右
    std::vector<cv::Point2f> points;
    int id; // 装甲板id
    int type;

public:
    ArmorBox(LightBlob left, LightBlob right);
    bool operator>(const ArmorBox &box) const;
};
typedef std::vector<ArmorBox> ArmorBoxes;