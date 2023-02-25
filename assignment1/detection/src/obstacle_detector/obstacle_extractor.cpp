#include <detection/obstacle_detection/obstacle_extractor.h>

ObstacleExtractor::ObstacleExtractor(std::list<Point> inputPoints,double angleIncrement)
{
    this->inputPoints = inputPoints;
    this->distanceProportion = angleIncrement;
}

void ObstacleExtractor::processPoints(std::list<Segment>& segments, std::list<Circle>& circles)
{
    this->segments.clear();
    this->circles.clear();
    
    //Group Points    
    groupPoints();

    //Merge Segments
    mergeSegments();
    
    //Detect Circles
    detectCircles();

    //Merge Circles
    mergeCircles();
    
    //Remove circles that are very "small"
    for (auto circle = this->circles.begin(); circle != this->circles.end(); circle++)     
    {
        if((*circle).getRadius() < MIN_CIRCLE_RADIUS)
        {
            circle = this->circles.erase(circle);
            --circle;
        }
    }
    
    segments = std::list<Segment>(this->segments);
    circles = std::list<Circle>(this->circles);
}//processPoints

void ObstacleExtractor::groupPoints()
{
    double sinDp = std::sin(2.0 * distanceProportion);
    
    //Group points into same subsets in order to obtain similar separated objects
    PointSet pointSet;
    pointSet.setBegin(inputPoints.begin());
    pointSet.setEnd(inputPoints.begin());
    pointSet.setNumberOfPoints(1);
    pointSet.setIsVisible(true);

    for (PointIterator point = inputPoints.begin()++; point != inputPoints.end(); ++point) 
    {
        //Compute distance of the current point and the origin of the reference frame (Ri in the paper)
        double range = (*point).length();

        //Compute the distance between current and previous added point to the current set of points (d_i-1)^i
        double distance = (*point - *pointSet.getEnd()).length();

        //Check if the distance  between current and previous added point to the current set of points
        //is less than a certain threshold (MAX_GROUP_DISTANCE). If so, then add the current point to the pointset
        if (distance < MAX_GROUP_DISTANCE + range * distanceProportion) 
        {
            pointSet.setEnd(point);
            pointSet.incrementNumberOfPoints();
        }
        else 
        {
            double prevRange = (*pointSet.getEnd()).length();

            // Heron's equation
            double p = (range + prevRange + distance) / 2.0;
            double s = std::sqrt(p * (p - range) * (p - prevRange) * (p - distance));
            double sinD = 2.0 * s / (range * prevRange); // Sine of angle between beams

            // TODO: This condition can be fulfilled if the point are on the opposite sides
            // of the scanner (angle = 180 deg). Needs another check.
            if ((std::abs(sinD) < sinDp) && (range < prevRange))
            {
                pointSet.setIsVisible(false);
            }

            //Detect the segments for the current pointSet
            detectSegments(pointSet);

            // Begin new point set
            pointSet.setBegin(point);
            pointSet.setEnd(point);
            pointSet.setNumberOfPoints(1);
            pointSet.setIsVisible((std::abs(sinD) > sinDp) || (range < prevRange));
        }
    }

    // Check the last point set too!
    detectSegments(pointSet); 

       
}//groupPoints

void ObstacleExtractor::detectSegments(const PointSet& pointSet)
{
    //Check if the pointset of near points contains at least MIN_GROUP_POINTS number of points
    if (pointSet.getNumberOfPoints() < MIN_GROUP_POINTS)    
        return;
    
    //Instantiate the segment with first and last point of the pointset 
    //In this case we use Iterative End Point Fit version algorithm
    Segment segment(*pointSet.getBegin(), *pointSet.getEnd());  

    //Check if we are using the Iterative End Point Fit or Split and Merge algorithm
    if (USE_SPLIT_AND_MERGE)
        segment = Segment::fitSegment(pointSet);

    //Seeks the points that are farthest from the line defined by the previous segment
    PointIterator setDivider;
    double maxDistance = 0.0;
    double distance = 0.0;
    int splitIndex = 0; // Natural index of splitting point (counting from 1)
    int pointIndex = 0; // Natural index of current point in the set

    // Seek the point of division
    for (PointIterator point = pointSet.getBegin(); point != pointSet.getEnd(); ++point) 
    {
        ++pointIndex;

        //Check if the distance between the point selected and the segment built for the pointset is grater than the current max distance
        //dj^(bar) in the paper
        if ((distance = segment.distanceTo(*point)) >= maxDistance) 
        {
            //Compute the distance of the current point and the reference frame (R_j in the paper)
            double r = (*point).length();

            //Check if the distance between the point selected and the segment built for the pointset is greater
            //than a certain threshold (MAX_SPLIT_DISTANCE), if so update : maxDistance, the 
            if (distance > MAX_SPLIT_DISTANCE + r * distanceProportion) //((d_j)^bar > d_split + R_jd_p in the paper)
            {
                maxDistance = distance;
                setDivider = point;
                splitIndex = pointIndex;
            }
        }
    }

    // Split the set only if the sub-groups are not 'small'
    if ((maxDistance > 0.0) && 
        (splitIndex > MIN_GROUP_POINTS) &&
        (splitIndex < (pointSet.getNumberOfPoints() - MIN_GROUP_POINTS))) 
    {
        // Clone the dividing point for each group
        setDivider = inputPoints.insert(setDivider, *setDivider);  

        PointSet subset1, subset2;        

        subset1.setBegin(pointSet.getBegin());
        subset1.setEnd(setDivider);
        subset1.setNumberOfPoints(splitIndex);
        subset1.setIsVisible(pointSet.getIsVisible());

        subset2.setBegin(++setDivider);
        subset2.setEnd(pointSet.getEnd());
        subset2.setNumberOfPoints(pointSet.getNumberOfPoints() - splitIndex);
        subset2.setIsVisible(pointSet.getIsVisible());

        //Repeat the procedure util no more subset is returned
        detectSegments(subset1); 
        detectSegments(subset2);
    } else 
    {  
        // Add the segment
        if (!USE_SPLIT_AND_MERGE) //If not split and merge use the segment that fits the point as detected segment for the pointset
            segment = Segment::fitSegment(pointSet);

        segments.push_back(segment);
    }
}//detectSegments

void ObstacleExtractor::mergeSegments()
{
    for (auto i = segments.begin(); i != segments.end(); i++) 
    {
        for (auto j = i; j != segments.end(); j++) 
        {
            Segment mergedSegment;

            if (compareSegments(*i, *j, mergedSegment))//Check if the two segments must be merged 
            {
                //Insert the merged segment
                auto tempIterator = segments.insert(i, mergedSegment); 

                //Erase the other two segments
                segments.erase(i);
                segments.erase(j);

                //Check the new segment against others
                i = --tempIterator; 
                break;
            }
        }
    }
}//mergeSegments

bool ObstacleExtractor::compareSegments(const Segment& segment1, const Segment& segment2, Segment& mergedSegment)
{
    if (&segment1 == &segment2) //same address => same segment
        return false;

    // Segments must be provided counter-clockwise
    if (segment1.getFirstPoint().cross(segment2.getFirstPoint()) < 0.0)
        return compareSegments(segment2, segment1, mergedSegment);

    //Check proximity between segments (connectivity test in the paper)
    if (checkSegmentsProximity(segment1, segment2)) 
    {
        std::vector<PointSet> pointSets;
        pointSets.insert(pointSets.end(), segment1.getPointSets().begin(), segment1.getPointSets().end());
        pointSets.insert(pointSets.end(), segment2.getPointSets().begin(), segment2.getPointSets().end());

        //Compute the segment that fits the set of points given by the 4 points that the define the two segments
        Segment segment = Segment::fitSegment(pointSets);

        //Check if the fitted segment is collinear with the other two (spread test in the paper)
        if (checkSegmentsCollinearity(segment, segment1, segment2)) 
        {
            mergedSegment = segment;
            return true;
        }
    }
}//compareSegments

bool ObstacleExtractor::checkSegmentsProximity(const Segment& segment1, const Segment& segment2) 
{
    return (segment1.trueDistanceTo(segment2.getFirstPoint()) < MAX_MERGE_SEPARATION ||
            segment1.trueDistanceTo(segment2.getLastPoint())  < MAX_MERGE_SEPARATION ||
            segment2.trueDistanceTo(segment1.getFirstPoint()) < MAX_MERGE_SEPARATION ||
            segment2.trueDistanceTo(segment1.getLastPoint())  < MAX_MERGE_SEPARATION);
}//checkSegmentsProximity

bool ObstacleExtractor::checkSegmentsCollinearity(const Segment& segment, const Segment& segment1, const Segment& segment2) {
    return (segment.distanceTo(segment1.getFirstPoint()) < MAX_MERGE_SPREAD &&
            segment.distanceTo(segment1.getLastPoint())  < MAX_MERGE_SPREAD &&
            segment.distanceTo(segment2.getFirstPoint()) < MAX_MERGE_SPREAD &&
            segment.distanceTo(segment2.getLastPoint())  < MAX_MERGE_SPREAD);
}//checkSegmentsCollinearity

void ObstacleExtractor::detectCircles()
{
    for (auto segment = segments.begin(); segment != segments.end(); segment++) 
    {
        if (CIRCLE_FROM_VISIBLE) 
        {
            bool segmentIsVisible = true;
            for (const PointSet& pointSet : segment->getPointSets()) 
            {
                if (!pointSet.getIsVisible()) 
                {
                    segmentIsVisible = false;
                    break;
                }
            }
            if (!segmentIsVisible)
                continue;
        }

        //Build circle from current segment
        Circle circle(*segment);

        //Increase radius of the circle to the maximum value
        circle.incrementRadius(RADIUS_ENLARGEMENT);

        //Check if the enlarged circle is not bigger than maximum size
        if (circle.getRadius() < MAX_CIRCLE_RADIUS) 
        {
            //Add circle to set of circles
            circles.push_back(circle);

            //Eventually remove converted segment
            if (DISCARD_CONVERTED_SEGMENTS) 
            {
                segment = segments.erase(segment);
                --segment;
            }
        }
    }
}//detectCircles

void ObstacleExtractor::mergeCircles() 
{
    for (auto i = circles.begin(); i != circles.end(); i++) 
    {
        for (auto j = i; j != circles.end(); j++) 
        {
            Circle mergedCircle;

            //Check if two detected circles must be merged and, eventually do it
            if (compareCircles(*i, *j, mergedCircle)) 
            {
                auto tempIterator = circles.insert(i, mergedCircle);
                circles.erase(i);
                circles.erase(j);
                i = --tempIterator;
                break;
            }
        }
    }
}//mergeCircles

bool ObstacleExtractor::compareCircles(const Circle& circle1, const Circle& circle2, Circle& mergedCircle)
{
    if (&circle1 == &circle2) //same address => same circle
        return false;

    //If circle circle1 is fully inside circle2 then, the merged circle is circle2
    if (circle2.getRadius() - circle1.getRadius() >= (circle2.getCenter() - circle1.getCenter()).length()) 
    {
        mergedCircle = circle2;
        return true;
    }

    //If circle circle2 is fully inside circle1 then, the merged circle is circle1
    if (circle1.getRadius() - circle2.getRadius() >= (circle2.getCenter() - circle1.getCenter()).length()) 
    {
        mergedCircle = circle1;
        return true;
    }

    //If circles intersect and are "small" (their intersection radius circle is less than MAX_CIRCLE_RADIUS) them merge them
    if (circle1.getRadius() + circle2.getRadius() >= (circle2.getCenter() - circle1.getCenter()).length()) 
    {   
        //Compute center of the merged circle
        Point center = circle1.getCenter() + (circle2.getCenter() - circle1.getCenter()) * circle1.getRadius() / (circle1.getRadius() + circle2.getRadius());

        //Compute radius of the merged circle
        double radius = (circle1.getCenter() - center).length() + circle1.getRadius();

        //Create circle
        Circle circle(center, radius);

        //Increment radius
        circle.incrementRadius(std::max(circle1.getRadius(), circle2.getRadius()));

        //Check radius
        if (circle.getRadius() < MAX_CIRCLE_RADIUS) 
        {
            circle.getPointSets().insert(circle.getPointSets().end(), circle1.getPointSets().begin(), circle1.getPointSets().end());
            circle.getPointSets().insert(circle.getPointSets().end(), circle2.getPointSets().begin(), circle2.getPointSets().end());
            mergedCircle = circle;
            return true;
        }
    }

    return false;
}//compareCircles


//TODO: THIS CONSTRUCTOR IS USED FOR DEBUGGING PORPOSES
ObstacleExtractor::ObstacleExtractor(std::list<Point> inputPoints, double angleIncrement,int minGroupPoints,double maxGroupDistance,
double maxSplitDistance,double maxMergeSeparation,double maxMergeSpread,double radiusEnlargement, double maxCircleRadius) : ObstacleExtractor(inputPoints,angleIncrement)
{
    //TODO: TO USE IT, MAKE ALL THOSE CONSTANTS A VARIABLE
    /*MIN_GROUP_POINTS = minGroupPoints;
    MAX_GROUP_DISTANCE = maxGroupDistance;
    MAX_SPLIT_DISTANCE = maxSplitDistance;
    MAX_MERGE_SEPARATION = maxMergeSeparation;
    MAX_MERGE_SPREAD = maxMergeSpread;
    RADIUS_ENLARGEMENT = radiusEnlargement;
    MAX_CIRCLE_RADIUS = maxCircleRadius;*/
}

//TODO: THIS CONSTRUCTOR IS USED FOR DEBUGGING PORPOSES
ObstacleExtractor::ObstacleExtractor(std::list<Point> inputPoints, double angleIncrement, double radiusEnlargement, double maxCircleRadius): ObstacleExtractor(inputPoints,angleIncrement)
{
    //TODO: TO USE IT, MAKE ALL THOSE CONSTANTS A VARIABLE
    /*
    RADIUS_ENLARGEMENT = radiusEnlargement;
    MAX_CIRCLE_RADIUS = maxCircleRadius;*/
}