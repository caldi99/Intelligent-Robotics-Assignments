#include <detection/obstacle_extractor/figures/pointset.h>

PointSet::PointSet() : numberOfPoints(0), isVisible(false)
{
    ROS_INFO_STREAM("(Server) detection PointSet PointSet");
}

PointIterator PointSet::getBegin() const
{
    ROS_INFO_STREAM("(Server) detection PointSet getBegin");
    return begin;
} // getBegin

PointIterator PointSet::getEnd() const
{
    ROS_INFO_STREAM("(Server) detection PointSet getEnd");
    return end;
} // getEnd

int PointSet::getNumberOfPoints() const
{
    ROS_INFO_STREAM("(Server) detection PointSet getNumberOfPoints");
    return numberOfPoints;
} // getNumberOfPoints

bool PointSet::getIsVisible() const
{
    ROS_INFO_STREAM("(Server) detection PointSet getIsVisible");
    return isVisible;
} // getIsVisible

void PointSet::setBegin(PointIterator iterator)
{
    ROS_INFO_STREAM("(Server) detection PointSet setBegin");
    begin = iterator;
} // setBegin

void PointSet::setEnd(PointIterator iterator)
{
    ROS_INFO_STREAM("(Server) detection PointSet setEnd");
    end = iterator;
} // setEnd

void PointSet::setNumberOfPoints(int numberOfPoints)
{
    ROS_INFO_STREAM("(Server) detection PointSet setNumberOfPoints");
    this->numberOfPoints = numberOfPoints;
} // setNumberOfPoints

void PointSet::setIsVisible(bool isVisible)
{
    ROS_INFO_STREAM("(Server) detection PointSet setIsVisible");
    this->isVisible = isVisible;
} // setIsVisible

void PointSet::incrementNumberOfPoints()
{
    ROS_INFO_STREAM("(Server) detection PointSet incrementNumberOfPoints");
    numberOfPoints++;
} // incrementNumberOfPoints