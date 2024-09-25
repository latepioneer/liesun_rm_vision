#include <opencv2/opencv.hpp>
#include <iostream>
#include "my_thread.h"
using namespace std;
using namespace cv;

class data
{
    char src[13];
    unsigned char crc;
} teat_data;

int main()
{
    Task task;
    ThreadPool pool(2); // 创建一个具有2线程的线程池

    pool.enqueue([&task]
                 { task.camera_task(); });

    pool.enqueue([&task]
                 { task.get_armor_task(); });
    /*
    comm_service com;
    com.CommOpen("/dev/ttyACM0");
    com.CommInit(115200, 0, 8, 1, 'N');
    cout << "1";
    char src[16] = {0};
    com.CommRecv(src, 100);
    cout << (unsigned int)src[11];*/

    return 0;
}
