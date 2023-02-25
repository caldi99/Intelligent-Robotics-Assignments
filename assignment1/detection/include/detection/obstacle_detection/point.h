#ifndef POINT_H
#define POINT_H

//ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

//STL
#include <ostream>

//OPENCV
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

/**
 * This class represents a point in the space with all its possible applicable operations
 * @author : Francesco Caldivezzi
*/
class Point
{
public:
    //CONSTRUCTORS

    /**
     * Create a point given the coordinates of the point
     * @param point : The coordinates of the point
    */
    Point(cv::Point2d point = cv::Point2d(0,0));
    
    /**
     * Copy constructor
     * @param point : The point to copy
    */
    Point(const Point& point) ;

    //GETTERS

    /**
     * This function returns the current point
     * @return The current point
    */
    cv::Point2d getPoint() const;

    //SETTERS

    /**
     * This function set the current point with the point provided as parameter
     * @param point : The point to use as new point for the current one
    */
    void setPoint(cv::Point2d point);
    
    //MEMBER FUNCTIONS

    /**
     * This function computes the length of the segment that connects the orgin and the current point
     * @return : The length of the segment that connects the orgin and the current point
    */
    double length() const;

    /**
     * This function computes the length of the segment that connects the orgin and the current point squared
     * @return : The length of the segment that connects the orgin and the current point squared
    */
    double lengthSquared() const;

    /**
     * This function computes the angle formed by the segment that connects the orgin and the current point in radiants
     * @return : The angle formed by the segment that connects the orgin and the current point in radiants
    */
    double angle() const;

    /**
     * This function computes the angle formed by the segment that connects the orgin and the current point in grades
     * @return : The angle formed by the segment that connects the orgin and the current point in grades
    */
    double angleDeg() const;

    /**
     * This function computes the dot product between two points
     * @param point : The point with which computing the dot product
     * @return : The dot product
    */
    double dot(const Point& point) const;

    /**
     * This function computes the cross product between two points
     * @param point : The point with which computing the cross product
     * @return : The cross product
    */
    double cross(const Point& point) const;

    /**
     * This function returns the normalized version of the current point
     * @return The normalized version of the current point
    */
    Point normalized();

    /**
     * TODO: MAYBE REMOVE???
    */
    Point reflected(const Point& normal) const;

    /**
     * This function return point(-y,x)
     * @return : The point point(-y,x)
    */
    Point perpendicular();

    //OPERATOR OVERLOADING WITH MEMEBER FUNCTION
    Point operator- ();
    Point operator+ ();
    Point& operator=  (const Point& point);
    Point& operator+= (const Point& point);
    Point& operator-= (const Point& point);

    //OPERATOR OVERLOADING WITH FRIEND FUNCTION

    friend Point operator+ (const Point& point1, const Point& point2)
    {
        return Point(cv::Point2d(point1.point.x + point2.point.x, point1.point.y + point2.point.y));
    }

    friend Point operator- (const Point& point1, const Point& point2)
    {
        return Point(cv::Point2d(point1.point.x - point2.point.x, point1.point.y - point2.point.y));
    }

    friend Point operator* (const double factor, const Point& point)
    {
        return Point(cv::Point2d(factor * point.point.x,  factor * point.point.y));
    }

    friend Point operator* (const Point& point, const double factor)
    {
        return Point(cv::Point2d(factor * point.point.x,  factor * point.point.y));
    }

    friend Point operator/ (const Point& point, const double factor)
    {
        return (factor != 0.0) ? Point(cv::Point2d(point.point.x / factor, point.point.y / factor)) : Point();
    }
    
    friend bool operator== (const Point& point1, const Point& point2)
    {
        return (point1.point.x == point2.point.x) && (point1.point.y == point2.point.y);
    }
    
    friend bool operator!= (const Point& point1, const Point& point2)
    {
        return !(point1 == point2);
    }
    
    friend bool operator< (const Point& point1, const Point& point2)
    {
        return point1.lengthSquared() < point2.lengthSquared();
    }
    
    friend bool operator<= (const Point& point1, const Point& point2)
    {
        return point1.lengthSquared() <= point2.lengthSquared();
    }
    
    friend bool operator>  (const Point& point1, const Point& point2)
    {
        return point1.lengthSquared() > point2.lengthSquared();
    }
    
    friend bool operator>= (const Point& point1, const Point& point2)
    {
        return point1.lengthSquared() >= point2.lengthSquared();
    }
    
    friend bool operator! (const Point& point1)
    {
        return (point1.point.x == 0.0 && point1.point.y == 0.0);
    }
    
    friend std::ostream& operator<<(std::ostream& out, const Point& point)
    {
        out << point.point;
        return out;
    }

private:
    cv::Point2d point;
};

#endif //POINT_H