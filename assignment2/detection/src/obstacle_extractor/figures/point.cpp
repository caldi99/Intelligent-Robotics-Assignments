#include <detection/obstacle_extractor/figures/point.h>

Point::Point(cv::Point2d point)
{
    ROS_INFO_STREAM("(Server) detection Point Point");
    this->point = point;
}

double Point::length() const
{
    ROS_INFO_STREAM("(Server) detection Point length");
    return std::sqrt(lengthSquared());
} // length

double Point::lengthSquared() const
{
    ROS_INFO_STREAM("(Server) detection Point lengthSquared");
    return std::pow(point.x, 2) + std::pow(point.y, 2);
} // lengthSquared

double Point::angle() const
{
    ROS_INFO_STREAM("(Server) detection Point angle");
    return atan2(point.y, point.x);
} // angle

double Point::angleDeg() const
{
    ROS_INFO_STREAM("(Server) detection Point angleDeg");
    return 180 * angle() / M_PI;
} // angleDeg

double Point::dot(const Point &point) const
{
    ROS_INFO_STREAM("(Server) detection Point dot");
    return this->point.dot(point.point);
} // dot

double Point::cross(const Point &point) const
{
    ROS_INFO_STREAM("(Server) detection Point cross");
    return this->point.cross(point.point);
} // cross

Point Point::normalized()
{
    ROS_INFO_STREAM("(Server) detection Point normalized");
    return (length() > 0.0) ? *this / length() : *this;
} // normalized

Point Point::reflected(const Point &normal) const
{
    ROS_INFO_STREAM("(Server) detection Point reflected");
    return *this - 2.0 * normal * (normal.dot(*this));
} // reflected

Point Point::perpendicular()
{
    ROS_INFO_STREAM("(Server) detection Point perpendicular");
    return Point(cv::Point2d(-point.y, point.x));
} // perpendicular

Point Point::operator-()
{
    ROS_INFO_STREAM("(Server) detection Point operator-");
    return Point(cv::Point2d(-point.x, -point.y));
} // operator-

Point Point::operator+()
{
    ROS_INFO_STREAM("(Server) detection Point operator+");
    return Point(cv::Point2d(point.x, point.y));
} // operator+

Point &Point::operator=(const Point &point)
{
    ROS_INFO_STREAM("(Server) detection Point operator=");
    if (this != &point)
    {
        this->point.x = point.point.x;
        this->point.y = point.point.y;
    }
    return *this;
} // operator=

Point &Point::operator+=(const Point &point)
{
    ROS_INFO_STREAM("(Server) detection Point operator+=");
    this->point.x += point.point.x;
    this->point.y += point.point.y;
    return *this;
} // operator+=

Point &Point::operator-=(const Point &point)
{
    ROS_INFO_STREAM("(Server) detection Point operator-=");
    this->point.x -= point.point.x;
    this->point.y -= point.point.y;
    return *this;
} // operator-=

cv::Point2d Point::getPoint() const
{
    ROS_INFO_STREAM("(Server) detection Point getPoint");
    return point;
} // getPoint

void Point::setPoint(cv::Point2d point)
{
    ROS_INFO_STREAM("(Server) detection Point setPoint");
    this->point = point;
} // setPoint

Point::Point(const Point &point) : point(point.getPoint())
{
    ROS_INFO_STREAM("(Server) detection Point Point");
}