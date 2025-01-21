#include "armordetector.h"
#include "tools.h"

double ArmorDetector::distance(cv::Point2f first, cv::Point2f second)
{
    return sqrt(pow(first.x - second.x, 2) + pow(first.y - second.y, 2));
}

void ArmorDetector::get_roi(float ratio)
{
    roi_rect.x = roi_temp.x - roi_temp.width/2 * (ratio*3.5-1);
    roi_rect.y = roi_temp.y - roi_temp.height/2 * (ratio-1);
    roi_rect.width = roi_temp.width * ratio*3.5;
    roi_rect.height = roi_temp.height * ratio;
    roi_rect &= cv::Rect(cv::Point2f(0,0),cv::Size(src.img.cols,src.img.rows));
    roi = src.img(roi_rect);
    offset = roi_rect.tl();
}

bool ArmorDetector::setRoi(Mat_time _src,cv::Rect &tracking_rect)
{
    _src.copyTo(src);
    if(tracking_rect.empty())
    {
        if(roi_temp.empty())
        {
            roi = src.img;
            offset = cv::Point(0,0);
            lost_count = 0;
            state = ArmorState::LOST;
            return true;
        }
        if(lost_count < 3)
        {
            get_roi(roi_enlarge);
            lost_count++;
            state = ArmorState::FINDING;
            return true;
        }
        else{
            roi = src.img;
            roi_rect = cv::Rect();
            roi_temp  = cv::Rect();
            offset = cv::Point(0,0);
            lost_count = 0;
            state = ArmorState::LOST;
            return true;
        }
        
    }
    roi_temp = tracking_rect;
    get_roi(roi_enlarge);
    lost_count = 0;
    cv::imshow("roi",roi);
    return true;
}

cv::Mat ArmorDetector::img_preprocess(int color)
{
    vector<cv::Mat> channels;
    split(roi, channels);
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

bool ArmorDetector::find_light(cv::Mat binary)
{
    vector<vector<cv::Point>> contours;
    findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE,offset); // 寻找轮廓的最大四点
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

        LightBlob light = LightBlob(box);
        lightblobs.push_back(light);
    }
    if(lightblobs.size() < 2)
        return false;
    return true; 
}

bool ArmorDetector::isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    double dis = distance(light_blob_i.rrect.center, light_blob_j.rrect.center); // 两个灯条的中心位置距离
    double dif_Y = light_blob_i.rrect.center.y - light_blob_j.rrect.center.y;    // 两个灯条的中心Y距离
    double hight = (light_blob_j.rrect.size.height + light_blob_i.rrect.size.height) / 2;              // 平均灯条高度
    float aspectRatio = dis / hight;
    if (aspectRatio < light_min_ch_ratio)
        return false;
    if (aspectRatio > light_max_ch_ratio)
        return false;
    if (dif_Y > 60)
        return false;
    // if (abs(light_blob_i.rrect.angle - light_blob_j.rrect.angle) > light_angle_dif) // 两个灯条的旋转偏移量
    // {
    //     cout<<3<<endl;
    //     return false;
    // }
    return true;
}

bool ArmorDetector::matchArmorBoxes(LightBlobs &light_blobs, ArmorBoxes &armor_boxes)
{
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
            ArmorBox armor(light_blobs[i], light_blobs[j]);
            armor_boxes.push_back(armor);
            
        }
    }

    if (armor_boxes.empty())
    {
        return false;
    }

    //getArmorNum(armor_boxes);
    return true;
}

void ArmorDetector::getArmorNum(ArmorBoxes &armor_boxes)
{
    ArmorBoxes temp_armor_boxes;
for (auto& armor : armor_boxes)  // 使用引用来修改原始元素
{
    armor.id = classify.predit(armor.points, &roi);  // 直接修改原始 'armor' 的 id
    cout << armor.id << "    ";  // 输出的是原始 'armor' 的 id
    if (armor.id == 3)  // 如果 'armor' 的 id 为 3，加入 temp_armor_boxes
        temp_armor_boxes.push_back(armor);
}
cout << endl;
if (!temp_armor_boxes.empty())
    armor_boxes = std::move(temp_armor_boxes);  // 只保留 id 为 3 的元素
else
    armor_boxes.clear();  // 如果 temp_armor_boxes 为空，清空 armor_boxes

    
}

bool ArmorDetector::getBestArmor(ArmorBoxes &boxes)
{
    auto cmp = [](ArmorBox a, ArmorBox b)
    {
        return a > b;
    };
    sort(boxes.begin(), boxes.end(), cmp);
    
    return true;
}

bool ArmorDetector::ifOldArmor()
{
    if (last_target.box.empty() || ArmorState::LOST)
        return false;
    cv::Point2f delta = target.light_rect.center - last_target.light_rect.center;
    float distance = sqrt(delta.x * delta.x + delta.y * delta.y);
    if(last_target.id == target.id||distance/target.box.height<15)
        return true;
    return false;
}


cv::Point3f ArmorDetector::pnp(ArmorBox armor)
{
    vector<cv::Point3f> Points3D;
    if (armor.type == BIG_ARMOR)
    {
        Points3D.push_back(cv::Point3f(-24, 12, 0));
        Points3D.push_back(cv::Point3f(24, 12, 0));
        Points3D.push_back(cv::Point3f(24, -12, 0));
        Points3D.push_back(cv::Point3f(-24, -12, 0));
        cout<<"big"<<endl;
    }
    else
    {
        // Points3D.push_back(cv::Point3f(-12, 6, 0));
        // Points3D.push_back(cv::Point3f(12, 6, 0));
        // Points3D.push_back(cv::Point3f(12, -6, 0));
        // Points3D.push_back(cv::Point3f(-12, -6, 0));
        Points3D.push_back(cv::Point3f(-135.0/2.0, 55.0/2.0, 0));
        Points3D.push_back(cv::Point3f(135.0/2.0, 55.0/2.0, 0));
        Points3D.push_back(cv::Point3f(135.0/2.0, -55.0/2.0, 0));
        Points3D.push_back(cv::Point3f(-135.0/2.0, -55.0/2.0, 0));
    }
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    solvePnP(Points3D, armor.get_lightpoints(), Camera().cameraMatrix, Camera().distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    cv::Point3f points((float)tvec.ptr<double>(0)[0], (float)tvec.ptr<double>(0)[1], (float)tvec.ptr<double>(0)[2]);
    // cout << "x:" << points.x << "     y:" << points.y << "      z:" << points.z << endl;
    return points;
}

cv::Point2f ArmorDetector::getPitchYaw(ArmorBox temp_target,Gyropose gyro_pose)
{
    cv::Point3f world_point = pnp(temp_target);
    //cout<<world_point<<endl;
    Eigen::Quaternionf  q(gyro_pose.q_0,gyro_pose.q_1,gyro_pose.q_2,gyro_pose.q_3);
    cv::Point3f world_coord = camera2world(q,world_point,predictor.cam2gyro);
    //cout<<world_coord<<endl;
    if(state == ArmorState::FIRST)
        predictor.initState(world_point,gyro_pose);
    cv::Point3f world_predict = predictor.predict(world_point,gyro_pose);
    //cout<<"world_predict"<<world_predict<<endl;
    last_world_point = predictor.world_coord;
    return calPredict(world_predict,gyro_pose);
}

cv::Point2f ArmorDetector::getPitchYaw(cv::Point3f world_point,Gyropose gyro_pose)
{
    cv::Point3f world_predict = predictor.predict(world_point,gyro_pose.receive_time);
    return calPredict(world_predict,gyro_pose);
}

cv::Point2f ArmorDetector::calPredict(cv::Point3f world_predict,Gyropose gyro_pose)
{
    Eigen::Quaternionf  q(gyro_pose.q_0,gyro_pose.q_1,gyro_pose.q_2,gyro_pose.q_3);
    cv::Point3f cp_predict = world2camera(q,world_predict,predictor.cam2gyro);
    //cout<<"cp_predict"<<cp_predict<<endl;
    float yaw = atan(cp_predict.x/cp_predict.z)*180/CV_PI-0.3;
    float cp_pitch = atan(cp_predict.y/cp_predict.z)*180/CV_PI;
    float targetPitch = atan(world_predict.z/sqrt(world_predict.x*world_predict.x+world_predict.y*world_predict.y))*180/CV_PI;
    float Pitch = cp_pitch - predictor.pitch_time.x+targetPitch+1;
    return cv::Point2f(Pitch,yaw);
}

bool ArmorDetector::findArmorBox(ArmorBox &box)
{
    box = ArmorBox();
    target = ArmorBox();
    armorboxes.clear();
    lightblobs.clear();
    cv::Mat binary = img_preprocess(RED);
    //cv::imshow("binary",binary);
    if(!find_light(binary))
        return false;
    if(!matchArmorBoxes(lightblobs,armorboxes))
        return false;
    getBestArmor(armorboxes);
    target = armorboxes[0];
    
    box = armorboxes[0];
    if(ifOldArmor()) state = ArmorState::SHOOT;
    else state = ArmorState::FIRST;
    last_target = target;
    return true;
}


bool ArmorDetector::run(Mat_time _src,cv::Point2f &pitch_yaw)
{
    ArmorBox temp_target;
    setRoi(_src,target.box);
    if(!findArmorBox(temp_target))
    {
        if(state!=ArmorState::LOST)
        {
            pitch_yaw = getPitchYaw(last_world_point,_src.gyro_pose);
        }
        return true;
    }
    //cout<<_src.gyro_pose.q_0<<" "<<_src.gyro_pose.q_1<<" "<<_src.gyro_pose.q_2<<" "<<_src.gyro_pose.q_3<<endl;
    pitch_yaw = getPitchYaw(temp_target,_src.gyro_pose);
    cout<<pitch_yaw<<endl;
    return true;
}