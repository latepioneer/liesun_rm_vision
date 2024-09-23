#include <opencv2/opencv.hpp>
#include <iostream>
#include "my_thread.h"
using namespace std;
using namespace cv;

int main()
{
    Task task;
    ThreadPool pool(2); // 创建一个具有2线程的线程池

    pool.enqueue([&task]
                 { task.camera_task(); });

    pool.enqueue([&task]
                 { task.get_armor_task(); });

    
    return 0;
}
