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
    if (!task.init())
        cout << "connect error" << endl;
    thread t1(&Task::camera_task, &task);
    thread t2(&Task::get_armor_task, &task);
    thread t3(&Task::get_uart_task, &task);
    thread t4(&Task::send_uart_task, &task);
    // thread t5(&Task::InterGyroPose, &task);
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    // t5.join();
    //  Task task(2);
    //  task.camera_task();
    //  task.get_armor_task();

    // task.enqueue([&task]
    //              { task.camera_task(); });

    // task.enqueue([&task]
    //              { task.get_armor_task(); });
    // comm_service com;
    // com.CommOpen("/dev/ttyACM0");
    // com.CommInit(115200, 0, 8, 1, 'N');
    // ReceivePacket rdata;
    // com.CommRecv(rdata, sizeof(ReceivePacket));
    // cout << rdata.q[0] << endl;
    return 0;
}
