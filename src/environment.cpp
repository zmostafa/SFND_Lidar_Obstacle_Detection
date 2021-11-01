/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false; // false : remove cars from the scene
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = lidar->scan();
    // renderRays(viewer,lidar->position,pcl_ptr);
    // renderPointCloud(viewer, pcl_ptr, "Point Cloud",Color{1,1,1});
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> *ppc_ptr = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = ppc_ptr->SegmentPlane(pcl_ptr, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = ppc_ptr->Clustering(segmentCloud.first, 1.5, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        ppc_ptr->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = ppc_ptr->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

// Load PCD from files
// void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer){
//     ProcessPointClouds<pcl::PointXYZI> *pcd_ptr = new ProcessPointClouds<pcl::PointXYZI>();
//     pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pcd_ptr->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//     // renderPointCloud(viewer,inputCloud,"InputCloud");
//     pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pcd_ptr->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -6, -3, 1), Eigen::Vector4f(30, 6, 4, 1));
//     std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pcd_ptr->SegmentPlane(filteredCloud, 100, 0.2);
//     // renderPointCloud(viewer, filteredCloud, "Filtered Cloud");
//     renderPointCloud(viewer, segmentCloud.first, "Obstacle Cloud",Color(1, 0, 0));
//     renderPointCloud(viewer, segmentCloud.second, "Plane Cloud", Color(0, 1, 0));

//     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pcd_ptr->Clustering(segmentCloud.first, 0.5, 8, 300);

//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

//     for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
//     {
//         std::cout << "cluster size ";
//         pcd_ptr->numPoints(cluster);
//         renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);

//         Box box = pcd_ptr->BoundingBox(cluster);
//         renderBox(viewer,box,clusterId);
//         ++clusterId;
//     }
// }

// Stream PCD files
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -6, -3, 1), Eigen::Vector4f(30, 6, 4, 1));
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);

    // Use own RANSAC for project submition
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->Ransac_segmentPlane(filteredCloud, 100, 0.2);
    // renderPointCloud(viewer, filteredCloud, "Filtered Cloud");
    renderPointCloud(viewer, segmentCloud.first, "Obstacle Cloud",Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "Plane Cloud", Color(0, 1, 0));

    // Import data points to K-d tree;
    KdTree* kdTree = new KdTree();

    for(int index = 0 ; index < segmentCloud.first->points.size() ; ++index){
        kdTree->insert(segmentCloud.first->points[index], index);
    }

    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 8, 300);
    // Use own EuclideanClustring for Project submition
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanClustering(segmentCloud.first, kdTree, 0.4, 8, 300);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
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
    // simpleHighway(viewer);
    // CityBlock(viewer);
    ProcessPointClouds<pcl::PointXYZI>* stream_ptr = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = stream_ptr->streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_itr = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    while (!viewer->wasStopped ())
    {
        // clear view
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load pcd & run processing
        inputCloud = stream_ptr->loadPcd((*stream_itr).string());
        cityBlock(viewer, stream_ptr, inputCloud);

        ++stream_itr;
        if(stream_itr == stream.end()){
            stream_itr = stream.begin(); // loop over files;
        }

        viewer->spinOnce ();
    } 
}