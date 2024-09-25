#include "camera.h"
#include <iostream>
#include <fstream>

/*
    @brief   相机参数初始化
*/
void Camera::camera_init()
{
    MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", Frame_Rate);
    MV_CC_SetFloatValue(handle, "ExposureTime", Exposure_Time);
    MV_CC_SetFloatValue(handle, "Gain", Gain);
    MV_CC_SetBoolValue(handle, "GammaEnable", Gamma);
    MV_CC_SetEnumValue(handle, "PixelFormat", Pixel_Format);
}

/*
    @brief camera类的构造函数
*/
Camera::Camera()
{
    nRet = MV_OK;
    handle = NULL;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST)); // 全部初始化成0
    memset(&pDeviceInfo, 0, sizeof(MV_CC_DEVICE_INFO));
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
    memset(&CvtParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
}

bool Camera::start_cam()
{
    nRet = MV_CC_Initialize(); // 初始化SDK资源
    if (MV_OK != nRet)
    {
        printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
        return 0;
    }

    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList); // 查看指定传输协议的所有设备
    if (MV_OK != nRet)
    {
        printf("Enum Devices fail! nRet [0x%x]\n", nRet);
        return 0;
    }

    if (stDeviceList.nDeviceNum != 0)
    {
        memcpy(&pDeviceInfo, stDeviceList.pDeviceInfo, sizeof(MV_CC_DEVICE_INFO));
        if (NULL == &pDeviceInfo)
        {
            return 0;
        }
    }
    else
    {
        printf("Find No Devices!\n");
        return 0;
    }
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]); // ch:选择设备并创建句柄 | en:Select device and create handle
    if (MV_OK != nRet)
    {
        printf("Create Handle fail! nRet [0x%x]\n", nRet);
        return 0;
    }

    nRet = MV_CC_OpenDevice(handle); // 打开设备
    if (MV_OK != nRet)
    {
        printf("Open Device fail! nRet [0x%x]\n", nRet);
        return 0;
    }

    camera_init(); // 初始化相机
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
        return 0;
    }
    return 1;
}

void Camera::PrintDeviceINfo()
{
    if (NULL == &pDeviceInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
    }
    else if (pDeviceInfo.nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName: %s\n", pDeviceInfo.SpecialInfo.stUsb3VInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pDeviceInfo.SpecialInfo.stUsb3VInfo.chSerialNumber);
        printf("Device Number: %d\n\n", pDeviceInfo.SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else
    {
        printf("Not support.\n");
    }
}

void Camera::close_cam()
{
    int nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK == nRet)
        cout << "Stopped Grabbing !\n";
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
    }
}

/*
    @brief 获取一帧图像
    @param Mat* 图像的存放地址
 */
void Camera::get_pic(cv::Mat *srcimg)
{
    // 获取图像缓冲区
    int ret = MV_CC_GetImageBuffer(handle, &stOutFrame, 10);
    if (ret != MV_OK)
    {
        std::cerr << "Failed to get image buffer, error code: " << std::hex << ret << std::endl;
        return;
    }

    // 设置转换参数
    CvtParam.nWidth = stOutFrame.stFrameInfo.nWidth;              // 图像宽度
    CvtParam.nHeight = stOutFrame.stFrameInfo.nHeight;            // 图像高度
    CvtParam.enSrcPixelType = stOutFrame.stFrameInfo.enPixelType; // 源像素格式
    CvtParam.pSrcData = stOutFrame.pBufAddr;                      // 输入数据缓存
    CvtParam.nSrcDataLen = stOutFrame.stFrameInfo.nFrameLen;      // 输入数据长度
    CvtParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;         // 目标像素格式

    static std::vector<uint8_t> buffer(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 4 + 2048);
    CvtParam.pDstBuffer = buffer.data();     // 输出数据缓存
    CvtParam.nDstBufferSize = buffer.size(); // 输出缓冲区大小

    // 转换像素格式
    ret = MV_CC_ConvertPixelType(handle, &CvtParam);
    if (ret != MV_OK)
    {
        std::cerr << "Failed to convert pixel type, error code: " << ret << std::endl;
        MV_CC_FreeImageBuffer(handle, &stOutFrame); // 释放图像缓冲区
        return;
    }

    // 创建 OpenCV Mat 对象
    *srcimg = cv::Mat(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC3, buffer.data());

    // 释放图像缓冲区
    if (stOutFrame.pBufAddr != NULL)
    {
        MV_CC_FreeImageBuffer(handle, &stOutFrame);
    }
}

//
// void Camera::take_img(Mat* img) {
//    string src = "E:\\Codefield\\opencv\\halcon_opencv\\picture\\";
//    string filename = src + to_string(num) + ".jpg"; // Constructing filename
//    imwrite(filename, *img);  // Saving the image
//    num++;
//}
//
// void Camera::myCalibrateCamera()
//{
//    vector<Mat> imgs;
//    string src = "E:\\Codefield\\opencv\\halcon_opencv\\picture\\";
//    string imageName;
//    ifstream fin(src+"calibdata.txt");
//    if (!fin)                              //检测是否读取到文件
//    {
//        cerr << "没有找到文件" << endl;
//        return ;
//    }
//    while (getline(fin, imageName))
//    {
//        Mat img = imread(src+imageName);
//        imgs.push_back(img);
//    }
//    cout << imgs.size()<<endl;
//    Size board_size = Size(9, 6);//方格标定板内角点数目（行，列）
//    vector<vector<Point2f>> imgsPoints;//像素下每个交叉点的坐标
//    for (int i = 0; i < imgs.size(); i++)
//    {
//        Mat img1 = imgs[i];
//        Mat gray1;
//        cvtColor(img1, gray1, COLOR_BGR2GRAY);
//        vector<Point2f> img1_points;
//
//        bool success = findChessboardCorners(gray1, board_size, img1_points);
//        if (!success)
//        {
//            cout << "can not find the corners " <<i<< endl;
//            continue;
//        }
//        else
//        {
//            find4QuadCornerSubpix(gray1, img1_points, Size(5, 5));
//            cv::drawChessboardCorners(img1, board_size, img1_points, success); //将角点连线
//            cout  << i << endl;
//            cv::imshow("Camera calibration", img1);
//            cv::waitKey(0); //等待按键输入
//        }
//        imgsPoints.push_back(img1_points);
//    }
//
//    Size squareSize = Size(2.65, 2.65);//棋盘每个方格的真实尺寸
//    vector<vector<Point3f>> objectPoints;//棋盘每个方格交叉点的真实坐标
//    for (int i = 0; i < imgsPoints.size(); i++)
//    {
//        vector<Point3f> tempPointSet;
//        for (int j = 0; j < board_size.height; j++)
//        {
//            for (int k = 0; k < board_size.width; k++)
//            {
//                Point3f realPoint;
//                realPoint.x = j * squareSize.width;
//                realPoint.y = k * squareSize.height;
//                realPoint.z = 0;
//                tempPointSet.push_back(realPoint);
//            }
//        }
//        objectPoints.push_back(tempPointSet);
//    }
//
//    vector<int> point_number;//初始化每张图片的角点数量，假设每付图像中都可以看见完整的标定板
//    for (int i = 0; i < imgsPoints.size(); i++)
//    {
//        point_number.push_back(board_size.width * board_size.height);
//    }
//
//    Size imageSize;
//    imageSize.width = imgs[0].cols;
//    imageSize.height = imgs[0].rows;
//    vector<Mat> rvecs;//旋转向量
//    vector<Mat> tvecs;//平移向量
//
//    calibrateCamera(objectPoints, imgsPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, 0);
//    cout << "相机的内参矩阵=" << cameraMatrix << endl;
//    cout << "相机的畸变系数=" << distCoeffs << endl;
//    cout << imgs.size() << endl;
//    for (int i = 0; i < imgs.size(); i++)
//    {
//        Mat img1 ;
//        undistort(imgs[i], img1, cameraMatrix, distCoeffs);
//        imshow("pic", img1);
//        waitKey(0);
//    }
//}

Camera::~Camera()
{
}