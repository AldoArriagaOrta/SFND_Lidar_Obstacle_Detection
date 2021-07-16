/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	
	// For max iterations 
	for (int t = 0;  t < maxIterations; t++)
	{
		std::unordered_set<int> inliersCurrent;

		//Randomly select two points from the cloud, this could be done in a loop instead if we wanted to use more points
		int index_p1, index_p2;
		index_p1 = rand() % cloud->points.size();
		index_p2 = rand() % cloud->points.size();
		// inliersCurrent.insert(index_p1); 
		// inliersCurrent.insert(index_p2);

		//Line fitting
		float a = cloud->points[index_p1].y - cloud->points[index_p2].y;
		float b = cloud->points[index_p2].x - cloud->points[index_p1].x;
		float c = cloud->points[index_p1].x  * cloud->points[index_p2].y - cloud->points[index_p2].x * cloud->points[index_p1].y;

		// Measure distance between every point and fitted line
		for (int i = 0; i < cloud->points.size(); i++)
		{	// we can avoid evaluating the points used to build the line if we insert them first and check the indices during the iteration. How much runtime does that save?
			// if (i == index_p1 || i == index_p2) //check if the current point is one of the points used to construct the line... I completely overlooked this at first	
			// 	continue;

			pcl::PointXYZ point_s = cloud->points[i]; 

			float normalDistance = fabs(a * point_s.x + b * point_s.y + c)/sqrt(a * a + b * b); //istance between the current point and the fitted line

			if(normalDistance < distanceTol)
				inliersCurrent.insert(i); //If distance is smaller than threshold count it as inlier and add the index to the current set	
		}

		if(inliersCurrent.size() > inliersResult.size())
		{
			//store the point indices in the results set
			inliersResult = inliersCurrent;
		}

	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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

		//Line fitting
		float a =(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float b =(z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float c =(x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float d =-(a*x1+b*y1+c*z1);

		// Measure distance between every point and fitted line
		for (int i = 0; i < cloud->points.size(); i++)
		{	// we can avoid evaluating the points used to build the line if we insert them first and check the indices during the iteration. How much runtime does that save?
			// if (i == index_p1 || i == index_p2) //check if the current point is one of the points used to construct the line... I completely overlooked this at first	
			// 	continue;

			pcl::PointXYZ point_s = cloud->points[i]; 

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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);
	std::unordered_set<int> inliers = RansacPlane(cloud, 20, 0.25);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
