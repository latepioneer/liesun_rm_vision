#include "armordetector.h"

double ArmorDetector::distance(cv::Point2f first, cv::Point2f second)
{
    return sqrt(pow(first.x - second.x, 2) + pow(first.y - second.y, 2));
}

/*
    @brief 图像预处理
    @param img 需要处理的图像
    @param color 识别颜色
    @return 筛选后的二值图片
*/
cv::Mat ArmorDetector::img_preprocess(cv::Mat *img, int color)
{
    vector<cv::Mat> channels;
    split(*img, channels);
    cv::Mat img_B = channels.at(BLUE);
    cv::Mat img_G = channels.at(GREEN);
    cv::Mat img_R = channels.at(RED);
    cv::Mat color_binary;
    if (color == BLUE)
    {
        cv::Mat blue_mask, blue_threshold, blue_condition, final_blue_mask;
        subtract(img_B, img_R, blue_mask);
        threshold(blue_mask, blue_threshold, blue_red_diff, 255, cv::THRESH_BINARY);
        compare(img_B, img_G, blue_condition, cv::CMP_GT);
        bitwise_and(blue_threshold, blue_condition, final_blue_mask);
        final_blue_mask.copyTo(color_binary);
    }
    else if (color == RED)
    {
        cv::Mat red_mask, red_threshold, red_condition, final_red_mask;
        subtract(img_R, img_B, red_mask);
        threshold(red_mask, red_threshold, red_blue_diff, 255, cv::THRESH_BINARY);
        compare(img_R, img_G, red_condition, cv::CMP_GT);
        bitwise_and(red_threshold, red_condition, final_red_mask);
        final_red_mask.copyTo(color_binary);
    }

    // 帧间差分计算
    // cv::Mat diffbinary;
    // if (!prevbinary->empty()) {
    //	absdiff(color_binary, *prevbinary, diffbinary);
    // }

    // 闭运算来更新灯条
    morphologyEx(color_binary, color_binary, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    // color_binary.copyTo(*prevbinary);
    return color_binary;
}

/*
    @brief 寻找合适的灯条
    @param binary 经过预处理的图片
*/
void ArmorDetector::find_light(cv::Mat binary)
{
    vector<vector<cv::Point>> contours;
    findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // 寻找轮廓的最大四点
    for (int n = 0; n < contours.size(); n++)
    {
        if (contours[n].size() < 5)
            continue; // 判断面积
        cv::RotatedRect box = minAreaRect(contours[n]);
        float width = box.size.width > box.size.height ? box.size.height : box.size.width;
        float height = box.size.width < box.size.height ? box.size.height : box.size.width;
        float aspectRatio = width / height; // 长宽比例

        if (aspectRatio < light_min_wh_ratio)
            continue; // 检查宽高比是否小于最低值
        if (aspectRatio > light_max_wh_ratio)
            continue; // 检查宽高比是否大于最大值
        if (box.size.area() < 50)
            continue; // 筛除小灯条
        if (box.size.area() > 5000)
            continue; // 筛除大灯条
        // if (box.angle > 50 && box.angle < 130)continue;

        LightBlob light = LightBlob(box, width, height);
        lightblobs.push_back(light);
    }
}

bool ArmorDetector::isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    // float width = light_blob_j.rrect.center.x - light_blob_i.rrect.center.x;
    // float height = (light_blob_j.rrect.size.height + light_blob_i.rrect.size.height) / 2.0;
    // float angle_i = light_blob_i.rrect.angle > 90 ? 180.0 - light_blob_i.rrect.angle : light_blob_i.rrect.angle;
    // float angle_j = light_blob_j.rrect.angle > 90 ? 180.0 - light_blob_j.rrect.angle : light_blob_j.rrect.angle;

    // // 匹配灯条：角度差、长度比、高度差、组成的宽高比，滤除/\ \/
    // if (fabs(angle_i - angle_j) > param.lights_angle_max_dif ||
    //     max(light_blob_i.rrect.size.height, light_blob_j.rrect.size.height) / min(light_blob_i.rrect.size.height, light_blob_j.rrect.size.height) > param.lights_length_max_ratio ||
    //     fabs(light_blob_i.rrect.center.y - light_blob_j.rrect.center.y) / (height * cos(angle_i / 180.0 * M_PI)) > param.lights_Y_max_ratio ||
    //     width / height > param.armor_width_height_max_ratio ||
    //     width / height < param.armor_width_height_min_ratio ||
    //     (fabs(light_blob_i.rrect.angle - light_blob_j.rrect.angle) >= 90 && fabs(light_blob_i.rrect.angle - light_blob_j.rrect.angle) < 160)) // 滤除 /\ 和 \/ 灯条
    //     return false;
    double dis = distance(light_blob_i.rrect.center, light_blob_j.rrect.center); // 两个灯条的中心位置距离
    double dif_Y = light_blob_i.rrect.center.y - light_blob_j.rrect.center.y;    // 两个灯条的中心Y距离
    double hight = (light_blob_j.height + light_blob_i.height) / 2;              // 平均灯条高度
    float aspectRatio = dis / hight;
    if (aspectRatio < light_min_ch_ratio)
        return false;
    if (aspectRatio > light_max_ch_ratio)
        return false;
    if (dif_Y > 60)
        return false;
    if (abs(light_blob_i.rrect.angle - light_blob_j.rrect.angle) > light_angle_dif) // 两个灯条的旋转偏移量
        return false;
    return true;
}

bool ArmorDetector::matchArmorBoxes(LightBlobs &light_blobs, ArmorBoxes &armor_boxes)
{
    // armor_boxes.clear();
    if (lightblobs.size() < 2)
        return false;
    auto cmp = [](LightBlob a, LightBlob b) -> bool
    {
        return a.rrect.center.x < b.rrect.center.x;
    };
    sort(light_blobs.begin(), light_blobs.end(), cmp);

    for (int i = 0; i < light_blobs.size() - 1; i++)
    {
        for (int j = i + 1; j < light_blobs.size(); j++)
        {
            if (!isCoupleLight(light_blobs[i], light_blobs[j]))
                continue;
            // if (isBadArmor(i, j, light_blobs))
            //     continue;

            armor_boxes.push_back(ArmorBox(light_blobs[i], light_blobs[j]));
        }
    }

    if (armor_boxes.empty())
    {
        return false;
    }

    // return getArmorNum(armor_boxes);
}

void ArmorDetector::getBestArmor(ArmorBoxes &boxes)
{
    auto cmp = [](ArmorBox a, ArmorBox b)
    {
        return a > b;
    };

    sort(boxes.begin(), boxes.end(), cmp);
}

bool ArmorDetector::ifOldArmor()
{
    // if (last_target.box.empty() || ArmorState::LOST)
    // return false;
    // if (lost_count)
}

void ArmorDetector::find_armor()
{
    // if (lightblobs.size() < 2)
    //     return;
    // for (int i = 0; i < lightblobs.size(); i++)
    //     for (int j = i + 1; j < lightblobs.size(); j++)
    //     {
    //         double dis = distance(lightblobs[i].rrect.center, lightblobs[j].rrect.center); // 两个灯条的中心位置距离
    //         double dif_Y = lightblobs[i].rrect.center.y - lightblobs[j].rrect.center.y;    // 两个灯条的中心Y距离
    //         double hight = (lightblobs[j].height + lightblobs[i].height) / 2;              // 平均灯条高度
    //         float aspectRatio = dis / hight;
    //         if (aspectRatio < light_min_ch_ratio)
    //             continue;
    //         if (aspectRatio > light_max_ch_ratio)
    //             continue;
    //         if (dif_Y > 60)
    //             continue;
    //         // if (abs(lightblobs[i].rrect.angle - lightblobs[j].rrect.angle) > light_angle_dif)continue;//两个灯条的旋转偏移量
    //         ArmorBox rect_armor = ArmorBox(lightblobs[i], lightblobs[j]);
    //         armorboxes.push_back(rect_armor);
    //     }
    // lightblobs.clear();
    matchArmorBoxes(lightblobs, armorboxes);
    getBestArmor(armorboxes);
    // if (ifOldArmor())
    //     state = ArmorState::SHOOT;
    // else
    //     state = ArmorState::FIRST;
    // last_target = armorboxes[0];
}

cv::Point3f ArmorDetector::pnp(ArmorBox armor)
{
    vector<cv::Point3f> Points3D;
    if (armor.type == BIG_ARMOR)
    {
        Points3D.push_back(cv::Point3f(-12, 6, 0));
        Points3D.push_back(cv::Point3f(12, 6, 0));
        Points3D.push_back(cv::Point3f(12, -6, 0));
        Points3D.push_back(cv::Point3f(-12, -6, 0));
    }
    else
    {
        Points3D.push_back(cv::Point3f(-0.12, 0.06, 0));
        Points3D.push_back(cv::Point3f(0.12, 0.06, 0));
        Points3D.push_back(cv::Point3f(0.12, -0.06, 0));
        Points3D.push_back(cv::Point3f(-0.12, -0.06, 0));
        // Points3D.push_back(cv::Point3f(-7, 3, 0));
        // Points3D.push_back(cv::Point3f(7, 3, 0));
        // Points3D.push_back(cv::Point3f(7, -3, 0));
        // Points3D.push_back(cv::Point3f(-7, -3, 0));
    }
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    solvePnP(Points3D, armor.points, Camera().cameraMatrix, Camera().distCoeffs, rvec, tvec, false, cv::SOLVEPNP_AP3P);
    // myData<float> data((float)tvec.ptr<double>(0)[0], (float)tvec.ptr<double>(0)[1], (float)tvec.ptr<double>(0)[2]);
    cv::Point3f points((float)tvec.ptr<double>(0)[0], (float)tvec.ptr<double>(0)[1], (float)tvec.ptr<double>(0)[2]);
    //cout << "x:" << points.x << "     y:" << points.y << "      z:" << points.z << endl;
    return points;
}

// void ArmorDetector::getArmorNum(ArmorBoxes& armor_boxes)
//{
//	Mat temp;
//	bool all_wrong_flag = true; // 是否全为WRONG装甲板
//	for (auto& armor : armor_boxes)
//	{
//		adjustBox(armor.box);
//		temp = src(Rect2d(armor.box));
//		resize(temp, temp, Size(28, 28), cv::INTER_LINEAR);
//		Gamma(temp, temp, 0.6);
//		cvtColor(temp, temp, COLOR_BGR2GRAY);
//		armor.id = classifier(temp);
//		armor.confidence = 1.0;
//
//		if (debug_param.debug_classifier)
//			imshow("ID", temp);
//
//		switch (armor.id)
//		{
//		case 0:
//		case 1:
//		case 6:
//		case 7:
//			all_wrong_flag = false;
//			armor.type = ArmorType::BIG;
//			break;
//
//		case 2:
//		case 3:
//		case 4:
//		case 5:
//			all_wrong_flag = false;
//			armor.type = ArmorType::SMALL;
//			break;
//
//			// case 3:
//			// case 4:
//			// case 5:
//			//     all_wrong_flag = false;
//
//		default:
//			break;
//		}
//	}
cv::Point3f camera_to_world(Eigen::Quaternionf q1, cv::Point3f point, cv::Point3f trans_offset = cv::Point3f(-0.3, 9, 27.9))
{
    // point += trans_offset;
    Eigen::Quaternionf p(0, point.z, -point.x, -point.y);
    Eigen::Quaternionf result = q1 * p * q1.inverse();
    return cv::Point3f(result.x(), result.y(), result.z());
}

void ArmorDetector::getPitchYaw(cv::Point3f world_point, Gyropose gyro_pose)
{
    /// cv::Point3f real_point = camera_to_world(Eigen::Quaternionf(gyro_pose.q_0, gyro_pose.q_1, gyro_pose.q_2, gyro_pose.q_3), world_point);
    yaw = atan2(world_point.x, world_point.z) * 180.0 / CV_PI;
    pitch = atan2(world_point.y, world_point.z) * 180.0 / CV_PI;
    // cout << "real:" << real_point.x << "     " << real_point.y << "      " << real_point.z << endl;

    // yaw = atan(real_point.x / real_point.z) * 180.0 / CV_PI;
    // pitch = atan(real_point.z / sqrt(real_point.x * real_point.x + real_point.y * real_point.y)) * 180.0 / CV_PI;

    // yaw = atan2(real_point.x, real_point.y) * 180.0 / CV_PI;
    // pitch = atan2(real_point.z, real_point.y) * 180.0 / CV_PI;

    cout << "yaw:" << yaw << "     pitch:" << pitch << endl;
    // float targetPitch = atan(world_point.z / sqrt(world_point.x * world_point.x + world_point.y * world_point.y)) * 180.0 / CV_PI;
    // pitch = cp_pitch - predictor.pitch_time.x + targetPitch + 1.0;
}