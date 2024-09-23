#pragma once
#include <thread>
#include <iostream>
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

#define enemy_color RED // 蓝0红2

/*
    @brief 线程池
*/
class ThreadPool
{
public:
    ThreadPool(int numThreads) : stop(false)
    {
        for (int i = 0; i < numThreads; ++i)
        {
            threads.emplace_back([this]
                                 {
                    while (true)
                    {
                        std::function<void()> task;
                        {
                            std::unique_lock<std::mutex> lock(this->mtx);
                            this->condition.wait(lock, [this] {
                                return !this->tasks.empty() || this->stop;
                                });
                            if (this->stop && this->tasks.empty())
                                return;
                            task = std::move(this->tasks.front());
                            this->tasks.pop();
                        }
                        task();
                    } });
        }
    }

    ~ThreadPool()
    {
        {
            std::unique_lock<std::mutex> lock(mtx);
            stop = true;
        }
        condition.notify_all();
        for (std::thread &t : threads)
        {
            t.join();
        }
    }

    template <class F, class... Args>
    auto enqueue(F &&f, Args &&...args)
        -> std::future<typename std::result_of<F(Args...)>::type>
    {
        using return_type = typename std::result_of<F(Args...)>::type;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...));

        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(mtx);
            if (stop)
                throw std::runtime_error("enqueue on stopped ThreadPool");

            tasks.emplace([task]()
                          { (*task)(); });
        }
        condition.notify_one();
        return res;
    }

private:
    std::vector<std::thread> threads;
    std::queue<std::function<void()>> tasks;
    std::mutex mtx;
    std::condition_variable condition;
    bool stop;
};

/*
    @brief 任务集合;
*/
class Task
{
public:
    void camera_task();
    void get_armor_task();
    void send_uart_task();
    void get_uart_task();

    cv::Point3f get_armor_xyz(cv::Mat *img);

private:
    /* 线程锁 */
    std::mutex pic_mtx;       // 图像收集
    std::mutex send_data_mtx; // 发送下位机
    std::mutex get_data_mtx;  // 接受下位机

    std::queue<cv::Mat> pic_buffer;
    std::queue<std::vector<float>> send_data_buffer;
    std::queue<std::vector<float>> get_data_buffer;

    Camera cam;
    ArmorDetector armordetector;
    CoordPredictor coorpredictor;
    // ArmorDetector armordetector;
};