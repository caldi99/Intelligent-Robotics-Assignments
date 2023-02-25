#ifndef SEGMENT_H
#define SEGMENT_H

//ROS
#include <ros/ros.h>

//DETECTION PACKAGE
#include <detection/obstacle_detection/pointset.h>
#include <detection/obstacle_detection/point.h>

//ARMADILLO LIBRARY
#include <armadillo>

/**
 * This function represents a Segment, and all operations that can be done with it
 * @author : Francesco Caldivezzi
*/
class Segment
{
public:
    //CONSTRUCTORS

    /**
     * Create a segment given two points
     * @param point1 : The first point of the segment 
     * @param point2 : The last point of the segment
    */
    Segment(const Point& point1 = Point(), const Point& point2 = Point());

    //SETTERS

    /**
     * This function sets the first point of the segment
     * @param point : The first point of the segment
    */
    void setFirstPoint(Point point);

    /**
     * This function sets the last point of the segment
     * @param point : The last point of the segment
    */
    void setLastPoint(Point point);

    //GETTERS

    /**
     * This function returns the first point of the segment
     * @return : The first point of the segment
    */
    Point getFirstPoint() const;

    /**
     * This function returns the last point of the segment
     * @return : The last point of the segment
    */
    Point getLastPoint() const;

    /**
     * This function returns the pointsets of the current segment
     * @return : The pointSets of the current segment
    */
    std::vector<PointSet> getPointSets() const;

    //MEMBER FUNCTIONS

    /**
     * This function computes the length of the current segment
     * @return : The length of the current segment
    */
    double length() const;

    /**
     * This function computes the length of the current segment squared
     * @return : The length of the current segment squared
    */
    double lengthSquared() const;
    
    /**
     * This function returns the point coordinates of the normal unit vector of the current segment
     * @return The point coordinates of the normal unit vector of the current segment
    */
    Point normal() const;
    
    /**
     * This function computes the projection of the point provided as a parameter
     * @param point : The point to project to the segment
     * @return The projected point to the segment
    */
    Point projection(const Point& point) const;
    
    /**
     * This function computes the projection of the point provided as a parameter and checks if the projected point lay in the current segment, 
     * if so, then it return the projection computed, otherwise, one of the two extremes.
     * @param point : The point to project to the segment
     * @return The projected point to the segment or, eventually one of the two extremes of the segment if the projected point does not lay in the current segment
    */
    Point trueProjection(const Point& point) const;
    
    /**
     * This function computes the distance between the current segment and the point provideded as parameter
     * @param point : The point with which compunting the distance
     * @return The distance between the current segment and the point provideded as parameter
    */
    double distanceTo(const Point& point) const;
    
    /**
     * This function computes the distance between :
     * - the current segment and the point provideded as parameter if the projected point in the segment lays in the segment
     * - the distance between one of the two extremes of the segment and the provided point if, instead the projected point do not lays in the projected segment
     * @param point : The point with which compunting the distance
     * @return The distance between the current segment and the point provideded as parameter
    */
    double trueDistanceTo(const Point& point) const;

    //STATIC FUNCTIONS

    /**
     * This function returns a total best fit approximation of segment based on given point set
     * The equation used for fitting is given by Ax + By = -C and the A, B, C parameters are normalized
     * @param pointSet : The pointSet to fit
     * @return The segment that fits the pointset
    */
    static Segment fitSegment(const PointSet& pointSet);
    
    /**
     * This function returns a total best fit approximation of segment based on given vector of point sets
     * The equation used for fitting is given by Ax + By = -C and the A, B, C parameters are normalized
     * @param pointSets : A vector of pointsets 
     * @return The segment that fits the vector of pointsets 
    */
    static Segment fitSegment(const std::vector<PointSet>& pointSets);

    //OPERATOR OVERLOADING

    friend std::ostream& operator<<(std::ostream& out, const Segment& segment)
    {
        out << "point 1 : " << segment.firstPoint << ", point 2 : " << segment.lastPoint;
        return out;
    } 

private:
    Point firstPoint; //First point of the segment
    Point lastPoint; //Last point of the segment
    std::vector<PointSet> pointSets; //TODO:CHECK IF NEED TO ADD GETTER AND SETTER(NO SETTER BUT METHOD TO ADD,ETC..)
};
  
#endif //SEGMENT_H