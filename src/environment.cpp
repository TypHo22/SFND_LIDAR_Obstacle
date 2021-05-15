/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//SensorFusion
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"
#include "../src/quiz/cluster/kdtree3D.h"
#include "ransac3d.h"

//stl
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include <filesystem>
namespace fs = std::filesystem;
inline bool exists_test1 (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}
/*
void listOfFiles(std::string path)
{
    for (const auto & entry : fs::directory_iterator(path))
        std::cout << entry.path() << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    listOfFiles("/home/zaragesh/");
    bool existFile = exists_test1("/home/zaragesh/develop/sensorFusion/SFND_Lidar_Obstacle_Detection-master/src/sensors/data/pcd/simpleHighway.pcd");

    return pointProcessor.loadPcd("/home/zaragesh/develop/sensorFusion/SFND_Lidar_Obstacle_Detection-master/src/sensors/data/pcd/simpleHighway.pcd");
}

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
    // RENDER OPTIONS
    bool renderScene = false;
    bool render_obst = false;
    bool render_plane = false;
    bool render_cluster = true;
    bool render_box = true;

  //processor which applies all the techniques on the inputClouds
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //loaded inputCloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr const& inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

  //filter inputCloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.4, Eigen::Vector4f (-10, -6.5, -2, 1), Eigen::Vector4f (30, 6.5, 1, 1));
  //found clusters
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(filterCloud, 0.5, 10, 140);

  //boxes
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {

      std::cout << "cluster size";
      pointProcessorI->numPoints(cluster);
      renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId),
                       colors[clusterId % colors.size()]);
      // Fourth: Find bounding boxes for each obstacle cluster
      Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
      ++clusterId;

  }

  renderPointCloud(viewer,inputCloud,"inputCloud");
  //renderPointCloud(viewer,filterCloud,"filterCloud");
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------


    // RENDER OPTIONS
    bool renderScene = false;
    bool render_obst = false;
    bool render_plane = false;
    bool render_cluster = true;
    bool render_box = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
   // Lidar* lidar = new Lidar(cars,0.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = CreateData3D();// lidar->scan();
    //renderRays(viewer,lidar->position, inputCloud);
    //renderPointCloud(viewer,inputCloud,"inputCloud");

    // TODO:: Create point processor
     ProcessPointClouds<pcl::PointXYZ> pointProcessor;
     std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(
             inputCloud, 100, 0.2);
     if (render_obst) {
         renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
     }
     if (render_plane) {
         renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
     }


     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0,
                                                                                                3, 30);

     int clusterId = 0;
     std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

     for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
         if (render_cluster) {
             std::cout << "cluster size:  ";
             pointProcessor.numPoints(cluster);
             renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId),
                              colors[clusterId % colors.size()]);
             ++clusterId;
         }
         if (render_box) {
             Box box = pointProcessor.BoundingBox(cluster);
             renderBox(viewer, box, clusterId);
         }
         ++clusterId;
     }
     renderPointCloud(viewer, segmentCloud.second, "planeCloud");
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
      // RENDER OPTIONS
      bool renderScene = false;
      bool render_obst = false;
      bool render_plane = false;
      bool render_cluster = true;
      bool render_box = true;


    //filter inputCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.4, Eigen::Vector4f (-10, -6.5, -2, 1), Eigen::Vector4f (30, 6.5, 1, 1));

    //found clusters
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(filterCloud, 0.5, 10, 140);

    //boxes
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {

        std::cout << "cluster size";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId),
                         colors[clusterId % colors.size()]);
        // Fourth: Find bounding boxes for each obstacle cluster
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }

    //renderPointCloud(viewer,inputCloud,"inputCloud");
    renderPointCloud(viewer,filterCloud,"filterCloud");
}
*/



void cityBlockOwn(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
      // RENDER OPTIONS
      bool renderScene = false;
      bool render_filtered = true;
      bool render_obst = true;
      bool render_plane = false;
      bool render_box = true;

      // Setting hyper parameters


      float filterRes = 0.4;
      Eigen::Vector4f minpoint(-10, -6.5, -2, 1);
      Eigen::Vector4f maxpoint(30, 6.5, 1, 1);
      // SegmentPlane
      int maxIterations = 40;
      float distanceThreshold = 0.3;
      // Clustering
      float clusterTolerance = 0.5;
      int minsize = 10;
      int maxsize = 140;

      // First:Filter cloud to reduce amount of points step I
      pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minpoint,
                                                                                        maxpoint);

    //ransac filtering step II
      ransacOwn::Ransac3D mRansac;
      mRansac.executeRansac3D(filteredCloud,maxIterations,distanceThreshold);
      pcl::PointCloud<pcl::PointXYZI>::Ptr inlierCloud = mRansac.inLierCloud();//road
      pcl::PointCloud<pcl::PointXYZI>::Ptr outlierCloud = mRansac.outLierCloud();//obstacles

    //kdtree step III
    std::unique_ptr<KdTree3D> mKdtree(new KdTree3D);
    int index = 0;

    for(auto &a : outlierCloud->points)
    {
        mKdtree.get()->insert(a,index);
        index++;
    }

    //kdtree clustering IV
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> owncloudClusters = mKdtree->euclideanCluster3D(clusterTolerance,minsize,maxsize);
    //custom function
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(outlierCloud,clusterTolerance,minsize,maxsize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

     //custom animation
    if(render_box)
    {
        for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : owncloudClusters) {

            std::cout << "cluster size";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId),
                             colors[clusterId % colors.size()]);
            // Fourth: Find bounding boxes for each obstacle cluster
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
            ++clusterId;
        }
    }

    if(render_filtered)
        renderPointCloud(viewer,filteredCloud,"filterCloud");

    if(render_obst)
        renderPointCloud(viewer,outlierCloud,"obstacles");

    if(render_plane)
        renderPointCloud(viewer,inlierCloud,"road");

    if(renderScene)
        renderPointCloud(viewer,inputCloud,"filterCloud");

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
    CameraAngle setAngle = XY;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("Viewer 3D"));


    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);

    //Streamer
    //  Stream cityBlock function
        ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
        std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
        auto streamIterator = stream.begin();
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

        while (!viewer->wasStopped()) {
            // Clear viewer
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            // Load pcd and run obstacle detection process
            inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
            cityBlockOwn(viewer, pointProcessorI, inputCloudI);
            streamIterator++;
            if (streamIterator == stream.end()) {
                streamIterator = stream.begin();
            }
            viewer->spinOnce();
        }
}
