#include "my_thread.h"
#include <chrono>

bool Task::init()
{
    if (!cam.start_cam())
        return 0;
    if (!uartr.CommOpen("/dev/ttyUSB1"))
        return 0;
    if (uartr.CommInit(115200, 0, 8, 1, 'N') == -1)
        return 0;
    if (!uarts.CommOpen("/dev/ttyUSB0"))
        return 0;
    if (uarts.CommInit(115200, 0, 8, 1, 'N') == -1)
        return 0;
    return 1;
}

void Task::camera_task()
{
    while (1)
    {
        //cout<<"camera"<<endl;
        if (cv::waitKey(1) == 27)
        {
            cam.close_cam();
        }
        cv::Mat img;
        while(!cam.get_pic(&img))
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        cv::Mat rimg;
        if (!img.empty())
            resize(img, rimg, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        Mat_time timg(&rimg);
        imshow("1",rimg);
        pic_mtx.lock();
        pic_buffer.push(timg);
        if (pic_buffer.size() >= 6)
        {
            pic_buffer.pop(); // Remove oldest image to make space
        }
        pic_mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Task::get_armor_task()
{
    //
    ArmorDetector detector;
    
    while(1)
    {
        cout<<"armor_task"<<endl;
        Mat_time frame;
        while (!InterGyroPose(frame))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        ArmorConsumer(detector,frame);
       
    }
}

void Task::ArmorConsumer(ArmorDetector &detector,Mat_time frame)
{
    //cout<<"count"<<endl;
    cv::Point2f pitch_yaw;
    detector.run(frame,pitch_yaw);
    send_data_mtx.lock();
    SendPacket send_data(detector.state,detector.target.id,pitch_yaw.x,pitch_yaw.y);
    cout<<send_data.pitch<<" "<<send_data.yaw<<endl;
    send_data_buffer.push(send_data);
    send_data_mtx.unlock();
}

bool Task::InterGyroPose(Mat_time &frame)
{
    // cout << "InterGyroPose: Checking data buffer..." << endl;
    if(get_data_buffer.size() < GYRO_BUFFER_NUM)
        return false;
    get_data_mtx.lock();
    std::vector<Gyropose> temp_gyro_buffer;
    temp_gyro_buffer.resize(GYRO_BUFFER_NUM);
    for (int i = 0; i < GYRO_BUFFER_NUM; i++)
        temp_gyro_buffer[i] = get_data_buffer[i];
    get_data_mtx.unlock();
    // cout << "InterGyroPose: Waiting for picture..." << endl;
    if (pic_buffer.empty())
        return false;
    // cout << "InterGyroPose:Match..." << endl;
    pic_mtx.lock();
    if(!pic_buffer.empty())
    {
        // pic_mtx.lock();
    Mat_time temp = pic_buffer.front();
    pic_buffer.pop();
    // pic_mtx.unlock();

    int64 delta[GYRO_BUFFER_NUM] = {0};
    for (int i = 0; i < GYRO_BUFFER_NUM; i++)
        delta[i] = std::chrono::duration_cast<chrono::microseconds>(temp_gyro_buffer[i].receive_time - temp.start_time).count();
    if(delta[0] > 0)
    {
    }
    else if (delta[GYRO_BUFFER_NUM - 1] >= 0)
    {
        ///cout << "图像时间小于4元书" << endl;
        int min_index = 0;
        int64 min_value = abs(delta[0]);
        for (int i = 1; i < GYRO_BUFFER_NUM; i++)
        {
            if (abs(delta[i]) < min_value)
            {
                min_value = abs(delta[i]);
                min_index = i;
            }
        }
        frame = temp;
        frame.gyro_pose = temp_gyro_buffer[min_index];

        pic_mtx.unlock();
        return true;
    }
    else if (delta[GYRO_BUFFER_NUM - 1] < 0)
    {
        // cout << "图像时间大于4元书" << endl;
        if (delta[GYRO_BUFFER_NUM - 1] > -1000)
        {
            frame = temp;
            frame.gyro_pose = temp_gyro_buffer[GYRO_BUFFER_NUM - 1];

            pic_mtx.unlock();
            return true;
        }
        pic_mtx.unlock();
        return false;
    }
}
pic_mtx.unlock();
    return false;
}

void Task::get_uart_task()
{
    while (1)
    {
        // ReceivePacket rdata;
        //cout << "uart" << endl;

        // if (uart.CommRecv(rdata))
        // {

        //     get_data_mtx.lock();
        //     Gyropose new_post(rdata.q);
        //     get_data_buffer.push_back(new_post);
        //     if (get_data_buffer.size() > GYRO_BUFFER_NUM)
        //         get_data_buffer.erase(get_data_buffer.begin());
        //     get_data_mtx.unlock();
        // }
        // else
        // {
        //     continue;
        // }
        uint8_t JY61P_Recive_Data[11];
        float q[4] = {0};
        if(uartr.CommRecv(JY61P_Recive_Data,q))
        {
            get_data_mtx.lock();
            Gyropose new_post(q);
            get_data_buffer.push_back(new_post);
            ///cout<<q[0]<<"  "<<q[1]<<"    "<<q[2]<<"    "<<q[3]<<endl;
            //cout<<q[0]<<endl;
            if (get_data_buffer.size() > GYRO_BUFFER_NUM)
                get_data_buffer.erase(get_data_buffer.begin());
            get_data_mtx.unlock();
        }
        else
        {
            continue;
        }
        this_thread::sleep_for(chrono::microseconds(50));
    }
}

void Task::send_uart_task()
{
    int a = 0;
    while (1)
    {
        // cout<<a<<endl;
        // a++;
        // SendPacket test;
        // test.id = a;
        // uarts.CommSend(&test);
        if (!send_data_buffer.empty())
        {
            send_data_mtx.lock();
            uarts.CommSend(&send_data_buffer.front());
            cout<<send_data_buffer.front().pitch<<" "<<send_data_buffer.front().yaw<<endl;
            send_data_buffer.pop();
            send_data_mtx.unlock();
        }
        this_thread::sleep_for(chrono::microseconds(500));
    }
}
