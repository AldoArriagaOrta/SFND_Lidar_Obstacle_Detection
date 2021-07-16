// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    //create filtering object
    pcl::VoxelGrid<PointT> voxelGrid;
    typename pcl::PointCloud<PointT>::Ptr downSampledCloud (new pcl::PointCloud<PointT>);
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*downSampledCloud);

    typename pcl::PointCloud<PointT>::Ptr roiCloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roi(true);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(downSampledCloud);
    roi.filter(*roiCloud);

    std::vector<int> roofIndices;
    pcl::CropBox<PointT> roof(true);
    
    roof.setMin(Eigen::Vector4f(-1.8f, -1.45f, -2.0f, 1));
    roof.setMax(Eigen::Vector4f(2.8f, 1.45f, 0.5f, 1)); 
    roof.setInputCloud(roiCloud);
    roof.filter(roofIndices);

    pcl::PointIndices::Ptr roofPoints (new pcl::PointIndices);

    for (int pointIndex : roofIndices)
    {
        roofPoints->indices.push_back(pointIndex);
    }

    pcl::ExtractIndices<PointT> removeRoof;
    removeRoof.setInputCloud(roiCloud);
    removeRoof.setIndices(roofPoints);
    removeRoof.setNegative(true);
    removeRoof.filter(*roiCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roiCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>()); //do not forget to use typename to be able to use the template 
    typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>());

    // from the solution as an example of how to iterate over indices to populate a point cloud
    // for (int index : inliers->indices)
    //     plane-> points.push_back(cloud->points[index]);

    // from the PCL tutorial
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane);
    extract.setNegative (true);
    extract.filter (*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

/*  //This commented block corresponds to the tutorial on the point cloud library implementation
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); 
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg; //the most significant change wrt the tutorial, since we use a template for PointT rather than the specific point type
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    } 

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
*/

    // My implementation of 3D (plane) RANSAC	
    std::unordered_set<int> groundPoints = RansacPlane(cloud, maxIterations,distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr ground(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(groundPoints.count(index))
			ground->points.push_back(point);
		else
			obstacles->points.push_back(point);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, ground);
 
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersClouds;

/*  //This commented block corresponds to the tutorial on the point cloud library implementation
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clusters_indices;

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusters_indices);

    for (pcl::PointIndices pointsInCluster : clusters_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);

        for (int idx : pointsInCluster.indices)
            clusterCloud->points.push_back(cloud->points[idx]); 
        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;

        clusters.push_back(clusterCloud);
    }
*/

    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
  
    for (int i=0; i < cloud->points.size(); i++)
    {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        points.push_back(point);
        //populate the tree
    	tree->insert(points[i],i);
    }  
    
    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, clusterTolerance);

    for(std::vector<int> cluster : clusters)
  	{
        if (cluster.size() > minSize && cluster.size() < maxSize)
        {
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
            for(int indice: cluster)
            {
                PointT point;
                point.x = points[indice][0];
                point.y = points[indice][1];
                point.z = points[indice][2];
                clusterCloud->points.push_back(point);
            }  			
            clustersClouds.push_back(clusterCloud);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clustersClouds.size() << " clusters" << std::endl;

    return clustersClouds;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::OrientedBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    BoxQ orientedBox;
    PointT minInitPoint, maxInitPoint;
    pcl::getMinMax3D(*cluster, minInitPoint, maxInitPoint);
    Eigen::Vector4f pcaCentroid3D; // 3D centroid needed for the final transformation
    pcl::compute3DCentroid(*cluster, pcaCentroid3D);

    typename pcl::PointCloud<PointT>::Ptr cluster2D(new pcl::PointCloud<PointT>());
    for(int i = 0; i < cluster->points.size(); i++)
    {
        PointT point;
        point.x = cluster->points[i].x;
        point.y = cluster->points[i].y;
        point.z = 0;// remove all the Z values to effectively have a 2D representation
        cluster2D->points.push_back(point);
    }  

    //////// This section was taken from  http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
    /////// Only a few changes were introduced to be compatible with the 2D representation

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster2D, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster2D, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
    /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    /// the signs are different and the box doesn't get correctly oriented in some cases.

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr clusterProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster2D, *clusterProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*clusterProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid3D.head<3>();

    ///////////
    ///////////

    orientedBox.bboxTransform = bboxTransform;
    orientedBox.bboxQuaternion = bboxQuaternion;
    orientedBox.cube_length = maxInitPoint.z - minInitPoint.z;// The original bounding box height, due to the rotations we need to specify height along the X axis
    orientedBox.cube_width = -minPoint.y + maxPoint.y;
    orientedBox.cube_height = -minPoint.z + maxPoint.z;

    return orientedBox;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in ascending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}