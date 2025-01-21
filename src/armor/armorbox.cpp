#include "armorbox.h"

void LightBlob::regularRotated(cv::RotatedRect &rect)
{
    if (rect.size.width > rect.size.height)
    {
        float temp = rect.size.width;
        rect.size.width = rect.size.height;
        rect.size.height = temp;
        rect.angle = rect.angle>=0.0f?rect.angle-90.0f:rect.angle+90.0f;
    }
    if(rect.angle<0) rect.angle+=180.0f;
}

LightBlob::LightBlob(cv::RotatedRect r)
{
    this->rrect = cv::RotatedRect(r);
    regularRotated(rrect);
    float x= rrect.center.x;
    float y = rrect.center.y;
    float angle = rrect.angle;
    float height = rrect.size.height/2.0;
    if(angle<90)
    {
        up = cv::Point(x+height*sin(angle/180.0*M_PI),y-height*cos(angle/180.0*M_PI)); 
        down = cv::Point(x-height*sin(angle/180.0*M_PI),y+height*cos(angle/180.0*M_PI));

    }
    else{
        angle = 180-angle;
        up = cv::Point(x-height*sin(angle/180.0*M_PI),y-height*cos(angle/180.0*M_PI)); 
        down = cv::Point(x+height*sin(angle/180.0*M_PI),y+height*cos(angle/180.0*M_PI));
    }
}

std::vector<cv::Point2f> sortRotatedRectPoints(std::vector<cv::Point2f> points)
{
    // 1. 根据 x 坐标排序，确保左边的两个点在前
    std::sort(points.begin(), points.end(), [](const cv::Point2f &a, const cv::Point2f &b)
              { return a.x < b.x; });

    // 2. 前两个点是最左边的点，根据 y 坐标排序，将上面的点放在 points[0]
    if (points[0].y > points[1].y)
    {
        std::swap(points[0], points[1]);
    }

    // 3. 后两个点是最右边的点，根据 y 坐标排序，将上面的点放在 points[2]
    if (points[2].y > points[3].y)
    {
        std::swap(points[2], points[3]);
    }

    // 返回按顺序排列的点：左上、左下、右上、右下
    return points;
}

ArmorBox::ArmorBox(LightBlob left, LightBlob right)
{
    light_Blobs.push_back(left);
    light_Blobs.push_back(right);
    center = (left.rrect.center + right.rrect.center) / 2.0;
    double light_height, width, angle;
    width = sqrt(pow(right.rrect.center.x - left.rrect.center.x, 2) + pow(right.rrect.center.y - left.rrect.center.y, 2));
    light_height = std::max(left.rrect.size.height, right.rrect.size.height); // 灯条长度
    angle = atan2(right.rrect.center.y - left.rrect.center.y, right.rrect.center.x - left.rrect.center.x) * 180 / CV_PI;
    light_rect = cv::RotatedRect(center, cv::Size(width, light_height), angle);
    double armor_height = light_height*2;
    light_rect = cv::RotatedRect(center, cv::Size(width, armor_height), angle);
    std::vector<cv::Point2f> points(4);
    armor_rect.points(points.data());
    this->points = sortRotatedRectPoints(points);
    // 初步判断装甲板大小
    if (width / light_height > 2.5)
        type = BIG_ARMOR;
    else
        type = SMALL_ARMOR;
    box = cv::Rect(center - cv::Point(width / 2.0, light_height / 2.0), cv::Size(width, light_height));
}

bool ArmorBox::operator>(const ArmorBox &armor_2) const
{
    // 分类结果比较
    if ((id >= 8 && armor_2.id < 8) ||
        (id != 1 && armor_2.id == 1) ||
        (id == 2 && armor_2.id != 2))
        return false;
    else if ((id < 8 && armor_2.id >= 8) ||
             (id == 1 && armor_2.id != 1) ||
             (id != 2 && armor_2.id == 2))
        return true;

    // Lambda函数定义
    auto calDistance = [](cv::Point2f pt1, cv::Point2f pt2)
    {
        cv::Point2f dis = pt1 - pt2;
        return sqrt(pow(dis.x, 2) + pow(dis.y, 2));
    };

    // 计算到图像中心的距离
    cv::Point2f center(cv::CAP_PROP_FRAME_WIDTH / 2.0, cv::CAP_PROP_FRAME_HEIGHT / 2.0);
    float distance_score = calDistance(center, light_rect.center);
    float distance_score2 = calDistance(center, armor_2.light_rect.center);

    return distance_score < distance_score2;
}

std::vector<cv::Point2f> ArmorBox::get_lightpoints()
{
    std::vector<cv::Point2f> points;
    points.push_back(light_Blobs[0].up);
    points.push_back(light_Blobs[1].up);
    points.push_back(light_Blobs[1].down);
    points.push_back(light_Blobs[0].down);
    return points;
}
