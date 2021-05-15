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

        auto comparePoint = [](pcl::PointXYZ& pA, pcl::PointXYZ& pB)
        {
             if(pA.x == pB.x &&
                pA.y == pB.y &&
                pA.z == pB.z)
                 return true;

             return false;
        };

        auto randomIntegers = [&cloud,comparePoint](pcl::PointXYZ& pA, pcl::PointXYZ& pB,pcl::PointXYZ& pC)
        {
            const int min = 0;
            const int max = cloud.get()->points.size();
            while(comparePoint(pA,pB) ||
                  comparePoint(pA,pC) ||
                  comparePoint(pB,pC))
            {
                pA = cloud.get()->points.at(min + rand() % (max - min));
                pB = cloud.get()->points.at(min + rand() % (max - min));
                pC = cloud.get()->points.at(min + rand() % (max - min));
            }
        };

        // TODO: Fill in this function
        // For max iterations
        for(int iteration = 0; iteration < maxIterations;iteration++)
        {
        // Randomly sample subset and fit line
            pcl::PointXYZ pA; pA.x = 0; pA.y = 0; pA.z = 0;//stuetzpunkt
            pcl::PointXYZ pB; pB.x = 0; pB.y = 0; pB.z = 0;
            pcl::PointXYZ pC; pC.x = 0; pC.y = 0; pC.z = 0;
            randomIntegers(pA,pB,pC);
            pcl::PointXYZ v1, v2;//vectors from stuetzpunkt to  pB and pC
            v1.x = pB.x - pA.x; v1.y = pB.y - pA.y; v1.z = pB.z - pA.z;
            v2.x = pC.x - pA.x; v2.y = pC.y - pA.y; v2.z = pC.z - pA.z;
            pcl::PointXYZ vNormal;//normalen vector senkrecht auf ebene
            vNormal.x = v1.y * v2.z  -  v1.z * v2.y;
            vNormal.y = v1.z * v2.x  -  v1.x * v2.z;
            vNormal.z = v1.x * v2.y  -  v1.y * v2.x;

            //line parameters in coordinate form
            const double A = vNormal.x;
            const double B = vNormal.y;
            const double C = vNormal.z;
            const double D = (-pA.x * vNormal.x) + (-pA.y * vNormal.y) + (-pA.z * vNormal.z);

            // Measure distance between every point and fitted line
            double distance = 0;
            std::unordered_set<int> possibleInliersResult;

            for(size_t a = 0; a < cloud.get()->points.size();a++)
            {
                pcl::PointXYZ p = cloud.get()->points.at(a);
                distance = std::abs(A * p.x + B * p.y + C * p.z + D) /
                        std::sqrt(A * A + B * B + C * C);
            // If distance is smaller than threshold count it as inlier
                if(distance < distanceTol)
                    possibleInliersResult.insert(a);
            }

            // check if this iteration is better
            if(possibleInliersResult.size() > inliersResult.size())
                inliersResult = possibleInliersResult;
        }
        // Return indicies of inliers from fitted line with most inliers
        return inliersResult;
    }



int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 169, 0.4);

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
