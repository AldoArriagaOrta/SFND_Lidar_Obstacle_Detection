// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
//#include <pcl/kdtree/kdtree.h> // commented to prevent multiple definitions
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>
#include "quiz/cluster/kdtree.h"  // location of the implementation developed during the lessons

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
 
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    BoxQ OrientedBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

private:

    std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
    {
        std::unordered_set<int> inliersResult;
        srand(time(NULL));
        
        // For max iterations 
        for (int t = 0;  t < maxIterations; t++)
        {
            std::unordered_set<int> inliersCurrent;

            //Randomly select three points from the cloud, this could be done in a loop instead if we wanted to use more points
            int index_p1, index_p2, index_p3;
            index_p1 = rand() % cloud->points.size();
            index_p2 = rand() % cloud->points.size();
            index_p3 = rand() % cloud->points.size();
            inliersCurrent.insert(index_p1);
            inliersCurrent.insert(index_p2);
            inliersCurrent.insert(index_p3);

            float x1,y1,z1,x2,y2,z2,x3,y3,z3;

            x1 = cloud->points[index_p1].x;
            y1 = cloud->points[index_p1].y;
            z1 = cloud->points[index_p1].z;
            x2 = cloud->points[index_p2].x;
            y2 = cloud->points[index_p2].y;
            z2 = cloud->points[index_p2].z;
            x3 = cloud->points[index_p3].x;
            y3 = cloud->points[index_p3].y;
            z3 = cloud->points[index_p3].z;

            //plane fitting (cross product)
            float a =(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
            float b =(z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
            float c =(x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
            float d =-(a*x1+b*y1+c*z1);

            // Measure distance between every point and plane
            for (int i = 0; i < cloud->points.size(); i++)
            {	
                if (i == index_p1 || i == index_p2 || i == index_p3 ) 
                 	continue;

                PointT point_s = cloud->points[i]; 

                float normalDistance = fabs(a * point_s.x + b * point_s.y + c * point_s.z + d)/sqrt(a * a + b * b + c * c); //distance between the current point and the fitted line

                if(normalDistance < distanceTol)
                    inliersCurrent.insert(i); //If distance is smaller than threshold count it as inlier and add the index to the current set	
            }

            if(inliersCurrent.size() > inliersResult.size())
            {
                //store the point indices in the results set
                inliersResult = inliersCurrent;
            }
        }
        // Return indices of inliers from fitted line with most inliers
        return inliersResult;
    }
    
    void proximityRecursor(float distanceTol, const std::vector<std::vector<float>>& points, 
                            std::vector<bool>& processed, std::vector<int>& cluster, KdTree* tree, int idx)
    {
        processed[idx] = true; //mark point as processed
        cluster.push_back(idx); //add point to cluster
        std::vector<int> closePointsIds = tree->search(points[idx], distanceTol);

        for (int pointId: closePointsIds) // iterate through each nearby point
        {
            if(!processed[pointId])
            {
                proximityRecursor(distanceTol, points, processed, cluster, tree, pointId);
            }
        }
    }

    std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
    {
        std::vector<std::vector<int>> clusters; //"clusters" vector of vector of integers to store points ids belonging to clusters
        std::vector<bool> processed(points.size(), false); //"processed" vector with size equal to the number of points and initialize as false

        for(int idx = 0; idx < points.size(); idx++) //iterate through each point
        {
            if (!processed[idx]) // check if the current point has been processed
            {
                std::vector<int> cluster; //create cluster
                proximityRecursor(distanceTol, points, processed, cluster, tree, idx);
                clusters.push_back(cluster);
            }
        }
        return clusters;
    }

};
#endif /* PROCESSPOINTCLOUDS_H_ */