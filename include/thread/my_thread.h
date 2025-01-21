#pragma once
#include <thread>
#include <vector>
#include <queue>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <future>

#include "camera.h"
#include "armordetector.h"
#include "predictor.h"
#include "uart.h"
#include "Mat_time.h"
#include "classify.h"

#define enemy_color RED // 蓝0红2
#define GYRO_BUFFER_NUM 15

/*
    @brief 任务集合
*/
class Task
{
public:
    void camera_task();
    void get_armor_task();
    void send_uart_task();
    void get_uart_task();
    bool InterGyroPose(Mat_time &frame);
    void ArmorConsumer(ArmorDetector &detector,Mat_time frame);
    bool init();

private:
    /* 线程锁 */
    std::mutex pic_mtx;       // 图像收集
    std::mutex send_data_mtx; // 发送下位机
    std::mutex get_data_mtx;  // 接受下位机

    std::queue<Mat_time> pic_buffer;
    std::vector<Gyropose> get_data_buffer;
    std::queue<SendPacket> send_data_buffer;

    Camera cam;                  // 摄像头

    comm_service uartr;
    comm_service uarts;

    DataSet dataset;
};
