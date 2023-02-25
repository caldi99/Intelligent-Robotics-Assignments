#include <laser_scanner/clusterizer.h>

Clusterizer::Clusterizer(int kMin, int kMax, bool kKnown)
{
    this->kKnown = kKnown;
    this->kMin = kMin;
    this->kMax = kMax;
}

void Clusterizer::clusterize(std::vector<cv::Point2f> pointsToCluster, std::vector<int>& labels, cv::Mat& centers)
{

    if(kKnown)
    {        
        //Apply k-means algorithm
        cv::kmeans(pointsToCluster,
                    kMin,
                    labels, 
                    cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, MAX_ITERATIONS, DESIRED_ACCURACY), 
                    ATTEMPTS, 
                    cv::KMEANS_PP_CENTERS, 
                    centers
        );
    }else
    {
        std::vector<cv::Mat> listOfCenters;
        std::vector<std::vector<int>> listOfLabels;
        std::vector<float> silhouetteCoefficents;

        //Compute the best value of k
        for(int k = kMin; k <= kMax && k < pointsToCluster.size(); k++)
        {
            
            std::vector<int> currentLabels;
            cv::Mat currentCenters;

            //Apply k-means algorithm
            cv::kmeans(pointsToCluster,
                    k,
                    currentLabels, 
                    cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, MAX_ITERATIONS, DESIRED_ACCURACY), 
                    ATTEMPTS, 
                    cv::KMEANS_PP_CENTERS, 
                    currentCenters
            );

            //Compute silhoutte coefficent
            std::vector<cv::Point2f> currentCentersConverted = convertToVector(currentCenters);
            listOfCenters.push_back(currentCenters);
            listOfLabels.push_back(currentLabels);
            silhouetteCoefficents.push_back(computeSc(pointsToCluster,currentCentersConverted,currentLabels,k));        
        }

        //Select best cluster depending on the silhoutte coefficent
        int indexMax = 0;
        float currentMax = silhouetteCoefficents.at(0);
        for(int i = 1; i < silhouetteCoefficents.size(); i++)
        {
            if(currentMax < silhouetteCoefficents.at(i))
            {
                currentMax = silhouetteCoefficents.at(i);
                indexMax = i;
            }
        }
        labels = listOfLabels.at(indexMax);
        centers = listOfCenters.at(indexMax);
    }
}

std::vector<cv::Point2f> Clusterizer::convertToVector(cv::Mat points)
{
    std::vector<cv::Point2f> ret;

    //Conver to std::vector a cv::Mat object
    for(int r = 0; r < points.rows; r++)
    {
        cv::Point2f point(points.at<float>(r,0),points.at<float>(r,1));
        ret.push_back(point);
    }
    return ret;
}

float Clusterizer::computeSc(std::vector<cv::Point2f> points, std::vector<cv::Point2f> centers, std::vector<int> labels, int currentK)
{
    float sc = 0;
    //Compute average silhoutte coefficent
    for(int i = 0; i < points.size(); i++)
    {
        //Compute ax
        float dSumCurrentPointCluster = 0.0;
        int sizeCurrentPointCluster = 0;
        getDSumCurrentPointCluster(points.at(i),labels.at(i),
                                    points,labels,dSumCurrentPointCluster,sizeCurrentPointCluster);
        float ax = computeAx(dSumCurrentPointCluster,sizeCurrentPointCluster);
        
        //Compute bx
        std::vector<float> dSumCurrentPointClustersDifferentToClusterPoint;
        std::vector<int> sizeClustersDifferentClusterPoint;
        getDSumCurrentPointClustersDifferentToClusterPoint(points.at(i),labels.at(i),
                                            points,labels,currentK,
                                            dSumCurrentPointClustersDifferentToClusterPoint,
                                            sizeClustersDifferentClusterPoint
        );
        float bx = computeBx(dSumCurrentPointClustersDifferentToClusterPoint,sizeClustersDifferentClusterPoint);

        //Compute average silhouette coefficent
        sc += computeSx(ax,bx);
    }
    
    return sc / points.size();
}

std::vector<cv::Point2f> Clusterizer::getClusterBelongingToAPoint(std::vector<cv::Point2f> points, std::vector<int> labels, int labelPoint)
{    
    std::vector<cv::Point2f> ret;
    
    //Compute the cluster beloging to a selected point
    for(int i = 0; i < points.size(); i++)
    {
        if(labelPoint == labels.at(i))
        {
            ret.push_back(points.at(i));
        }
    }
    return ret;
}

std::vector<std::vector<cv::Point2f>> Clusterizer::getClustersDifferentToClusterOfPoint(std::vector<cv::Point2f> points,std::vector<int> labels, int labelPoint)
{
    std::vector<std::vector<cv::Point2f>>ret(points.size() - 1);

    //Compute the clusters that are not the clusters of the current selected point
    for(int i= 0; i < labels.size(); i++)
    {
        if(labelPoint != labels.at(i))
        {
            ret.at(labels.at(i)).push_back(points.at(i));
        }
    }
    return ret;
}

float Clusterizer::computeSx(float ax, float bx)
{
    //Compute the silhoutte coefficent for a single point
    return (bx - ax) / std::max(ax,bx);
}

float Clusterizer::computeAx(float dSumCurrentPointCluster, int sizeClusterCurrentPoint)
{
    //Compute the ax coefficent
    return dSumCurrentPointCluster / sizeClusterCurrentPoint;
}

float Clusterizer::computeBx(std::vector<float> dSumCurrentPointClustersDifferentToClusterPoint, std::vector<int> sizeClustersDifferentClusterPoint)
{
    float min = dSumCurrentPointClustersDifferentToClusterPoint.at(0) / sizeClustersDifferentClusterPoint.at(0);
    int minIndex = 0;

    //Compute the bx coefficent
    for(int i = 1; i < dSumCurrentPointClustersDifferentToClusterPoint.size(); i++)
    {
        if(min > (dSumCurrentPointClustersDifferentToClusterPoint.at(i) / sizeClustersDifferentClusterPoint.at(i)))
        {
            min = dSumCurrentPointClustersDifferentToClusterPoint.at(i) / sizeClustersDifferentClusterPoint.at(i);
            minIndex = i;
        }               
    }
    return min;
}

void Clusterizer::getDSumCurrentPointCluster(cv::Point2f currentPoint, int currentLabelPoint, 
                std::vector<cv::Point2f> points, std::vector<int> labels, 
                float& dSumCurrentPointCluster, int& sizeCurrentPointCluster)
{
    std::vector<cv::Point2f> clusterCurrentPoint;
    for(int i = 0; i < labels.size(); i++)
    {
        if(currentLabelPoint == labels.at(i))
        {
            clusterCurrentPoint.push_back(points.at(i));
        }
    }

    sizeCurrentPointCluster = clusterCurrentPoint.size();
    for(int i = 0; i < clusterCurrentPoint.size(); i++)
    {
        dSumCurrentPointCluster += computeDistance(currentPoint,clusterCurrentPoint.at(i));
    }
}

float Clusterizer::computeDistance(cv::Point2f p1, cv::Point2f p2)
{
    //Compute the distance between two points
    return std::sqrt(((p1.x - p2.x) * (p1.x - p2.x)) + ((p1.y - p2.y) * (p1.y - p2.y)));
}

void Clusterizer::getDSumCurrentPointClustersDifferentToClusterPoint(cv::Point2f currentPoint, int currentLabelPoint, 
                std::vector<cv::Point2f> points, std::vector<int> labels,int currentK, 
                std::vector<float>& dSumCurrentPointClustersDifferentToClusterPoint, 
                std::vector<int>& sizeClustersDifferentClusterPoint)
{
    std::vector<std::vector<cv::Point2f>> temp(currentK); //label != currentLabel -> list

    for(int i = 0; i < labels.size(); i++)
    {
        if(currentLabelPoint != labels.at(i))
        {
            temp.at(labels.at(i)).push_back(points.at(i));
        }
    }

    std::vector<std::vector<cv::Point2f>> clustersMinusClusterCurrentPoint; 

    for(int i = 0; i < temp.size(); i++)
    {
        if(!temp.at(i).empty())
        {
            clustersMinusClusterCurrentPoint.push_back(temp.at(i));
        }
    }

    for(int i = 0; i < clustersMinusClusterCurrentPoint.size(); i++)
    {
        float sum = 0;
        for(int j = 0; j < clustersMinusClusterCurrentPoint.at(i).size(); j++)
        {
            sum += computeDistance(currentPoint,clustersMinusClusterCurrentPoint.at(i).at(j));
        }

        dSumCurrentPointClustersDifferentToClusterPoint.push_back(sum);
        sizeClustersDifferentClusterPoint.push_back(clustersMinusClusterCurrentPoint.at(i).size());   
    }
}
