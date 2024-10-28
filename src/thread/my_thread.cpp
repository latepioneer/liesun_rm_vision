#include "my_thread.h"
#include <chrono>

bool Task::init()
{
    if (!cam.start_cam())
        return 0;
    if (!uart.CommOpen("/dev/ttyACM0"))
        return 0;
    if (uart.CommInit(115200, 0, 8, 1, 'N') == -1)
        return 0;
    return 1;
}

void Task::camera_task()
{
    while (1)
    {
        /// cout << "camera_task" << endl;
        if (cv::waitKey(1) == 27)
        {
            cam.close_cam();
        }
        cv::Mat img;
        cam.get_pic(&img);
        if (!img.empty())
            resize(img, img, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        // cv::imshow("2", img);
        Mat_time timg(&img);
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
    while (1)
    {
        // cout << "get_armor_task" << endl;
        if (cv::waitKey(1) == 27)
        {
            cam.close_cam();
        }
        while (pic_buffer.empty())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        pic_mtx.lock();
        Mat_time img = pic_buffer.front();
        pic_buffer.pop();
        pic_mtx.unlock();
        /// cout << "get_armor_task: Processing image..." << endl;
        while (!InterGyroPose(img))
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        cv::Point3f point = get_armor_xyz(&img.img);
        if (!armordetector.armorboxes.empty())
        {
            armordetector.lose_count = 0;
            armordetector.state = ArmorState::SHOOT;
            Point_time tpoint(point);
            armordetector.getPitchYaw(tpoint.world_point, img.gyro_pose);
            send_data_mtx.lock();
            SendPacket sdata( ArmorState::SHOOT, 1,armordetector.pitch, armordetector.yaw);
            cout << "try" << endl;
            send_data_buffer.push(sdata);
            send_data_mtx.unlock();
        }
        else if (armordetector.state != ArmorState::LOST)
        {
            armordetector.lose_count++;
            if(armordetector.lose_count == 70)
            {
                armordetector.state = ArmorState::LOST;
                send_data_mtx.lock();
                SendPacket sdata( armordetector.state, 1,armordetector.pitch, armordetector.yaw);
                cout << "try" << endl;
                send_data_buffer.push(sdata);
                send_data_mtx.unlock();
            }
        }
        armordetector.armorboxes.clear();
        armordetector.lightblobs.clear();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

cv::Point3f Task::get_armor_xyz(cv::Mat *img)
{
    cv::Mat binary = armordetector.img_preprocess(img, enemy_color);
    armordetector.find_light(binary);
    // cout << armordetector.lightblobs.size() << endl;
    armordetector.find_armor();

    if (!armordetector.armorboxes.empty())
    {
        cv::Point3f armor = coorpredictor.predict(armordetector.pnp(armordetector.armorboxes[0]));
        // cout << "predict:" << armor.x << "       " << armor.y << "     " << armor.z << endl;
        vector<cv::Point2f> imagePoints;
        vector<cv::Point3f> objectPoints;
        objectPoints.push_back(armor);
        cv::Mat rvec1 = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec1 = cv::Mat::zeros(3, 1, CV_64FC1);
        projectPoints(objectPoints, rvec1, tvec1, cam.cameraMatrix, cam.distCoeffs, imagePoints);
        cv::circle(*img, imagePoints[0], 10, cv::Scalar(255, 255, 0), 1);
        imshow("1", *img);
        return armor;
    }
}

bool Task::InterGyroPose(Mat_time &frame)
{
    // cout << "InterGyroPose: Checking data buffer..." << endl;
    if (get_data_buffer.size() < GYRO_BUFFER_NUM)
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

    Mat_time temp = pic_buffer.front();
    // pic_buffer.pop();
    // pic_mtx.unlock();

    int64 delta[GYRO_BUFFER_NUM] = {0};
    for (int i = 0; i < GYRO_BUFFER_NUM; i++)
        delta[i] = std::chrono::duration_cast<chrono::microseconds>(temp_gyro_buffer[i].receive_time - temp.start_time).count();
    if (delta[GYRO_BUFFER_NUM - 1] >= 0)
    {
        // cout << "图像时间小于4元书" << endl;
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

void Task::get_uart_task()
{
    while (1)
    {
        ReceivePacket rdata;
        // cout << "1" << endl;

        if (uart.CommRecv(rdata))
        {

            get_data_mtx.lock();
            Gyropose new_post(rdata.q);
            //cout << rdata.q[0] << endl;
            get_data_buffer.push_back(new_post);
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
    while (1)
    {
        if (!send_data_buffer.empty())
        {
            cout << "send_uart" << endl;
            send_data_mtx.lock();
            uart.CommSend(&send_data_buffer.front());
            send_data_buffer.pop();
            send_data_mtx.unlock();
        }
        this_thread::sleep_for(chrono::microseconds(500));
    }
}
