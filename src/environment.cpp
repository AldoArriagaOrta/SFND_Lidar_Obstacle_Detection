/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
//#include "quiz/ransac/ransac2d.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    Box egoBox;
    egoBox.x_min = -1.8f;
    egoBox.y_min = -1.2f;
    egoBox.z_min = -2.0f;
    egoBox.x_max = 2.8f;
    egoBox.y_max = 1.3f;
    egoBox.z_max = 0.5f;

    renderBox(viewer,egoBox, 0,Color(0,0,1));

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZI>);
    filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.15 , Eigen::Vector4f (-20.0f, -6.0f, -5.0f, 1), Eigen::Vector4f (30.0f, 7.0f, 5.0f, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);

    renderPointCloud(viewer,segmentCloud.second,"ground",Color(0,1,0));
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstaclesClusters = pointProcessorI->Clustering(segmentCloud.first, 0.4, 15, 1500);

    int clusterId = 1;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,1,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : obstaclesClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud" + std::to_string(clusterId), colors[clusterId%3]);

        BoxQ clusterBox = pointProcessorI->OrientedBoundingBox(cluster);
        renderBox(viewer, clusterBox,clusterId);
        ++clusterId;
    } 
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    //Lidar::Lidar lidar = Lidar(std::vector<Car> cars, 0.0) //wrong!! you should really review your C++ concepts
    Lidar* lidar = new Lidar(cars, 0.0);  // to instantiate on the heap, we need a pointer type and the word new
    //pcl::PointCloud<pcl::PointXYZ>::Ptr rc_pcl = lidar.scan();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan(); //lidar is a pointer, so we call the functon with the arrow operator
    //renderRays(viewer, lidar->position , cloud);
    // renderPointCloud(viewer, cloud, "ray_cast_cloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pcd_processor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pcd_processor->SegmentPlane(cloud, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstacles",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"plane",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pcd_processor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pcd_processor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId), colors[clusterId%3]);

        Box box = pcd_processor->BoundingBox(cluster);
        //renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}




int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
     
    // For streaming
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
    //

    //simpleHighway(viewer);
    // cityBlock(viewer);

    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // } 
}
