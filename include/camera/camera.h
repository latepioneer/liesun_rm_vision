#pragma once

#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>

#define Frame_Rate 165          // 相机帧率
#define Gain 5                  // 增益
#define Gamma true              // 伽马矫正
#define Exposure_Time 2000      // 曝光时间
#define Pixel_Format 0x01080009 // 像素格式为Bayer RG 8

using namespace std;

class Camera
{
private:
    void *handle;
    int nRet;
    MV_CC_DEVICE_INFO_LIST stDeviceList; // 该结构体包含两个信息，1是连接设备数量，2是连接设备的信息
    MV_CC_DEVICE_INFO pDeviceInfo;
    MV_FRAME_OUT stOutFrame;            // 图像结构体
    MV_CC_PIXEL_CONVERT_PARAM CvtParam; // 像素转换结构体

    void camera_init();

    int num = 0;

public:
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1187.11280, 0, 373.96157,
                            0, 1187.32272, 280.15269,
                            0, 0, 1); // 相机内参矩阵
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.08197, 0.62288, -0.00087, 0.00131, -3.26992);

    Camera();
    void PrintDeviceINfo();
    /*
    @brief
    @return 1表示开启成功，0表示开启失败*/
    bool start_cam();
    /*
    @brief 关闭相机
    */
    void close_cam();
    /*
        @brief 获取一帧图像
        @param Mat* 图像的存放地址
    */
    bool get_pic(cv::Mat *srcimg);
    // void myCalibrateCamera();//求相机的内参矩阵以及畸变系数
    // void take_img(Mat* img);
    ~Camera();
};