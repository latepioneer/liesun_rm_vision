#include "armorbox.h"

LightBlob::LightBlob(cv::RotatedRect rrect, double width, double height)
{
    this->rrect = rrect; // 保存传入的 RotatedRect 对象
    this->height = height;
    this->width = width;
    cv::Point2f src_points[4];
    rrect.points(src_points); // 获取旋转矩形的四个顶点坐标

    // 清空 points 向量，确保没有残留数据
    points.clear();

    // 根据旋转矩形的角度选择如何添加顶点
    if (rrect.angle > 45 && rrect.angle <= 90)
    {
        // 如果角度在 45 到 90 度之间，按顺序添加顶点
        for (int i = 0; i < 4; i++)
        {
            points.push_back(src_points[i]);
        }
    }
    else if (rrect.angle >= 0 && rrect.angle <= 45)
    {
        // 如果角度在 0 到 45 度之间，添加顶点时将顺序偏移一个位置
        for (int i = 0; i < 4; i++)
        {
            points.push_back(src_points[(i + 1) % 4]);
        }
    }
}

std::vector<cv::Point2f> sortRotatedRectPoints(std::vector<cv::Point2f> points, double angle)
{
    for (int i = 0; i < 4; i++)
        for (int k = i + 1; k < 4; ++k)
            if (points[i].x > points[k].x)
                swap(points[i], points[k]);
    if (points[0].y > points[1].y)
    {
        cv::Point2f t = points[0];
        points[0] = points[1];
        points[1] = points[3];
        points[3] = t;
    }
    else
        swap(points[1], points[3]);
    if (points[1].y > points[2].y)
        swap(points[1], points[2]);
    return points;
}

ArmorBox::ArmorBox(LightBlob left, LightBlob right)
{
    this->light_Blobs->push_back(left);
    this->light_Blobs->push_back(right);
    cv::Point center = (left.rrect.center + right.rrect.center) / 2.0;
    double length, width, angle;
    width = sqrt(pow(right.rrect.center.x - left.rrect.center.x, 2) + pow(right.rrect.center.y - left.rrect.center.y, 2));
    length = std::max(left.height, right.height); // 灯条长度
    angle = atan2(right.rrect.center.y - left.rrect.center.y, right.rrect.center.x - left.rrect.center.x) * 180 / CV_PI;
    double height = 2 * length; // 装甲板宽度
    rect = cv::RotatedRect(center, cv::Size(width, height), angle);
    std::vector<cv::Point2f> points(4);
    rect.points(points.data());
    this->points = sortRotatedRectPoints(points, angle);
    // 初步判断装甲板大小
    if (width / height > 2.5)
        type = BIG_ARMOR;
    else
        type = SMALL_ARMOR;
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
    float distance_score = calDistance(center, rect.center);
    float distance_score2 = calDistance(center, armor_2.rect.center);

    return distance_score < distance_score2;
}
