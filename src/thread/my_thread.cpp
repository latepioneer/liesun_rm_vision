#include "my_thread.h"

void Task::camera_task()
{
    while (1)
    {
        cv::Mat img = cam.get_pic();
        pic_mtx.lock();
        pic_buffer.push(img);
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
        while (pic_buffer.empty())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        pic_mtx.lock();
        cv::Mat img = pic_buffer.front();
        pic_buffer.pop();
        get_armor_xyz(&img);
        imshow("1", img);
        cv::waitKey(1);
        pic_mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

cv::Point3f Task::get_armor_xyz(cv::Mat *img)
{
    cv::Mat binary = armordetector.img_preprocess(img, enemy_color);
    armordetector.find_light(binary);
    armordetector.find_armor();
    if (!armordetector.armorboxes.empty())
    {
        //cout << "ready" << endl;
        cv::Point3f armor = coorpredictor.predict(armordetector.pnp(armordetector.armorboxes[0]));
        vector<cv::Point2f> imagePoints;
        vector<cv::Point3f> objectPoints;
        objectPoints.push_back(armor);
        cv::Mat rvec1 = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec1 = cv::Mat::zeros(3, 1, CV_64FC1);
        projectPoints(objectPoints, rvec1, tvec1, Camera::cameraMatrix, Camera::distCoeffs, imagePoints);
        cv::circle(*img, imagePoints[0], 10, cv::Scalar(255, 255, 0), 1);
        armordetector.armorboxes.clear();
        return armor;
    }
}
