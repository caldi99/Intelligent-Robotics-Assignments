#ifndef POINTSET_H
#define POINTSET_H

//ROS
#include <ros/ros.h>

//DETECTION PACKAGE
#include <detection/obstacle_extractor/figures/point.h>

//STL
#include <list>

typedef std::list<Point>::iterator PointIterator;

/**
 * This class implements a wrapper of a container of points stored somewhere else with which you can easily access to it.
 * @author : Francesco Caldivezzi
*/
class PointSet
{
public:
    //CONSTRUCTORS

    /**
     * Create a PointSet
    */
    PointSet();

    //MEMBER FUNCTIONS

    /**
     * Increment the number of points of the pointset
    */
    void incrementNumberOfPoints();

    //GETTERS

    /**
     * This function returns the begin iterator of the pointset
     * @return the begin iterator of the pointset
    */
    PointIterator getBegin() const;

    /**
     * This function returns the end iterator of the pointset
     * @return the end iterator of the pointset
    */
    PointIterator getEnd() const;

    /**
     * This function returns the number of points of the pointset
     * @return The number of points of the pointset
    */
    int getNumberOfPoints() const;

    /**
     * This function returns if the current pointset is visible or not
     * @return : True if the pointset is visible
    */
    bool getIsVisible() const;

    //SETTERS

    /**
     * This function sets the begin iterator of the pointset
     * @param iterator : The begin iterator value
    */
    void setBegin(PointIterator iterator);

    /**
     * This function sets the end iterator of the pointset
     * @param iterator : The end iterator value
    */
    void setEnd(PointIterator iterator);
    
    /**
     * This function sets the number of points of the pointset
     * @param numberOfPoints : The number of points of the pointset
    */
    void setNumberOfPoints(int numberOfPoints);

    /**
     * This function sets the the visibility of the pointset
     * @param isVisible : The visibility of the pointset
    */
    void setIsVisible(bool isVisible);
private:
    // DATA MEMBERS

    PointIterator begin; //Begin iterator of the poinset
    PointIterator end; //End iterator of the poinset
    int numberOfPoints; //Number of points of the pointset
    bool isVisible;  //Visibility of the pointset
};

#endif //POINTSET_H