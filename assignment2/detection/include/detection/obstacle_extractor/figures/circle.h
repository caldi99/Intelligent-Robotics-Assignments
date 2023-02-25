#ifndef CIRCLE_H
#define CIRCLE_H

// ROS
#include <ros/ros.h>

// DETECTION PACKAGE
#include <detection/obstacle_extractor/figures/point.h>
#include <detection/obstacle_extractor/figures/segment.h>

/**
 * This class, represent a circle
 * @author : Francesco Caldivezzi
 */
class Circle
{
public:
    // CONSTRUCTORS

    /**
     * Creates a circle given the center and the radius
     * @param point : The center of the circle
     * @param radius : The radius of the circle
     */
    Circle(const Point &point = Point(), const double radius = 0.0);

    /**
     * Create a circle by taking the segment as a base of equilateral triangle. The circle is circumscribed on this triangle.
     * @param segment : The segment to use to build the circle
     */
    Circle(const Segment &segment);

    // GETTERS

    /**
     * This function get the point sets corresponding to the current circle
     * @return : The point sets corresponding to the current Circle
     */
    std::vector<PointSet> getPointSets() const;

    /**
     * This function returns the radius of the current circle
     * @param : The radius radius of the current circle
     */
    double getRadius() const;

    /**
     * This function returns the center of the current circle
     * @param : The center of the current circle
     */
    Point getCenter() const;

    // MEMEBER FUNCTIONS

    /**
     * This function compute the distance between the current circumference and a point
     * @param point : The point with which computing the distance
     * @return : The distance between the current circumference and a point
     */
    double distanceTo(const Point &point);

    /**
     * This function increment the radius of the current circle by the specified parameter
     * @param radius : The quantity to increment the current circle radius
     */
    void incrementRadius(double radius);

    // STATIC FUNCTIONS
    /**
     * This function computes the total best fit approximation of a circle based on given point set.
     * The equation used for fitting is given by : a1 * x + a2 * y + a3 = -(x^2 + y^2) where parameters
     * a1, a2, a3 are obtained from circle equation (x-x0)^2 + (y-y0)^2 = r^2.
     * @param pointSet : The pointset to use for builing a circle
     * @return The circle that fits the pointset
     */
    static Circle fitCircle(const std::list<Point> &pointSet);

    // OVERLOADING OPERATORS
    friend std::ostream &operator<<(std::ostream &out, const Circle &c)
    {
        out << "Center : " << c.center << ", Radius : " << c.radius;
        return out;
    }

private:
    Point center;
    double radius;
    std::vector<PointSet> pointSets;
};

#endif // CIRCLE_H