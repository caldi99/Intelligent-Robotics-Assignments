#ifndef CLUSTERIZER_H
#define CLUSTERIZER_H

#include <vector>

//OPENCV
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


/// @brief Class to implement clusterization
class Clusterizer
{
private:
    int kMin; //min value of k
    int kMax; //max value of k
    bool kKnown; //k is known or not
    
    const int MAX_ITERATIONS = 100; //max iterations of single run of the kmeans
    const float DESIRED_ACCURACY = 0.001; //accuracy to achieve
    const int ATTEMPTS = 5; //number of times it tries the kmeans several times with different centers

    /// @brief Convert the set of points to cluster from cv::Mat to std::vector
    /// @param points The set of points to cluster
    /// @return The points to cluster converted into std::vector
    std::vector<cv::Point2f> convertToVector(cv::Mat points);

    /// @brief Compute the average silhouette coefficent
    /// @param points The set of points to cluster
    /// @param centers The centers associated to the computed clusters
    /// @param labels The labels associated to the points to cluster
    /// @param currentK Current number of clusters selected
    /// @return The average silhouette coefficent
    float computeSc(std::vector<cv::Point2f> points, std::vector<cv::Point2f> centers, std::vector<int> labels, int currentK);

    /// @brief Get the set of points of the cluster belonging to a given point
    /// @param points The set of points to cluster
    /// @param labels The labels associated to the points to cluster
    /// @param labelPoint The label of the current point selected
    /// @return The set of points of the cluster of a given point
    std::vector<cv::Point2f> getClusterBelongingToAPoint(std::vector<cv::Point2f> points, std::vector<int> labels, int labelPoint);

    /// @brief Get the list of set of points, i.e. a list of clusters of points such that those clusters are not the cluster of the selected point
    /// @param points The set of points to cluster
    /// @param labels The labels associated to the points to cluster
    /// @param labelPoint The label of the current point selected
    /// @return The list of clusters of points such that those clusters are not the cluster of the selected point
    std::vector<std::vector<cv::Point2f>> getClustersDifferentToClusterOfPoint(std::vector<cv::Point2f> points,std::vector<int> labels,int labelPoint);

    /// @brief Compute the silhoutte coefficent for a single point i.e. sx = (bx - ax) / max(ax,bx)
    /// @param ax The ax coefficent
    /// @param bx The bx coefficent
    /// @return The silhoutte coefficent for a single point
    float computeSx(float ax, float bx);

    /// @brief Compute the ax coefficent ax = dsum(x,Ci) / |Ci| where Ci is the cluster for which x belongs to
    /// @param dSumCurrentPointCluster The dsum(x,Ci) quantity
    /// @param sizeClusterCurrentPoint The cardinality of Ci i.e. |Ci|
    /// @return The ax coefficent
    float computeAx(float dSumCurrentPointCluster, int sizeClusterCurrentPoint);

    /// @brief Compute the bx coefficent bx = min i!=j dsum(x,Cj) / |Cj| where Ci is the cluster for which the selected point belongs to
    /// @param dSumCurrentPointClustersDifferentToClusterPoint The list of values of dsum(x,Cj) for each j != i where Ci is the cluster for which the selected point belongs to
    /// @param sizeClustersDifferentClusterPoint The list of cardinalities of Cj for each j != i where Ci is the cluster for which the selected point belongs to
    /// @return The bx coefficent
    float computeBx(std::vector<float> dSumCurrentPointClustersDifferentToClusterPoint, std::vector<int> sizeClustersDifferentClusterPoint);

    /// @brief Compute dsum(x,C) = sum y in C (x,y)
    /// @param currentPoint The current selected point
    /// @param currentLabelPoint The label assigned to the current point
    /// @param points The set of points to cluster
    /// @param labels The labels associated to the points to cluster
    /// @param dSumCurrentPointCluster The dsum value
    /// @param sizeCurrentPointCluster The size of the cluster C
    void getDSumCurrentPointCluster(cv::Point2f currentPoint, int currentLabelPoint, 
                std::vector<cv::Point2f> points, std::vector<int> labels, 
                float& dSumCurrentPointCluster, int& sizeCurrentPointCluster);

    /// @brief Compute the distance between two points (euclidian distance)
    /// @param p1 First point
    /// @param p2 Second point
    /// @return The distance between two points
    float computeDistance(cv::Point2f p1, cv::Point2f p2);

    /// @brief Compute dsum(x,Cj) and |Cj| for each j != i where Ci is the cluster for which the selected point belongs to
    /// @param currentPoint The current selected point
    /// @param currentLabelPoint The label assigned to the current point
    /// @param points The set of points to cluster
    /// @param labels The labels associated to the points to cluster
    /// @param currentK The current number of cluster chosen
    /// @param dSumCurrentPointClustersDifferentToClusterPoint list of dsum(x,Cj) where Ci is the cluster for which the selected point belongs to
    /// @param sizeClustersDifferentClusterPoint list of |Cj| where Ci is the cluster for which the selected point belongs to
    void getDSumCurrentPointClustersDifferentToClusterPoint(cv::Point2f currentPoint, int currentLabelPoint, 
                std::vector<cv::Point2f> points, std::vector<int> labels,int currentK, 
                std::vector<float>& dSumCurrentPointClustersDifferentToClusterPoint, 
                std::vector<int>& sizeClustersDifferentClusterPoint);

public:

    /// @brief Constructor for the Clusterizer class
    /// @param kMin min Value of k for the k-means algorithm if k is not known, otherwise if k is known it is equal to kMax
    /// @param kMax max Value of k for the k-means algorithm if k is not known, otherwise if k is known it is equal to kMin
    /// @param kKnown true if k is known false, otherwise
    Clusterizer(int kMin, int kMax, bool kKnown);

    /// @brief Function to clusterize the points
    /// @param pointsToCluster The set of points to cluster
    /// @param labels The labels associated to the points to cluster
    /// @param centers The centers associated to the computed clusters
    void clusterize(std::vector<cv::Point2f> pointsToCluster,std::vector<int>& labels, cv::Mat& centers);
};

#endif // CLUSTERIZER_H
