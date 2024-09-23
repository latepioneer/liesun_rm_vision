#include "camera.h"

const cv::Mat Camera::cameraMatrix = (cv::Mat_<double>(3, 3) << 1148.94897, 0, 326.97111,
                                      0, 1136.90969, 276.29967,
                                      0, 0, 1);

const cv::Mat Camera::distCoeffs = (cv::Mat_<double>(1, 5) << -0.02406, -0.44315, -0.00163, -0.00534, 1.56468);

Camera::Camera() : video(0)
{
    camera_init();
}

void Camera::camera_init()
{
    video.set(cv::CAP_PROP_EXPOSURE, 2000);
    video.set(cv::CAP_PROP_FRAME_WIDTH, 1000);
    // video.set(cv::CAP_PROP_FPS, 200);

    if (video.isOpened())
    {
        std::cout << "镜头图像的宽度" << video.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
        std::cout << "镜头图像的高度" << video.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
        std::cout << "镜头图像的曝光" << video.get(cv::CAP_PROP_EXPOSURE) << std::endl;
        std::cout << "帧率为:" << video.get(cv::CAP_PROP_FPS) << std::endl;
    }
    else
    {
        std::cout << "链接失败" << std::endl;
    }
}

cv::Mat Camera::get_pic()
{
    cv::Mat img;
    video >> img;
    return img;
}

Camera::~Camera()
{
    ;
}