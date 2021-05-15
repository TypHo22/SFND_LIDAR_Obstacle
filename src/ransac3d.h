/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting


#include <unordered_set>
#include <memory>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker

namespace ransacOwn
{
    class Ransac3D
    {
    public:

    Ransac3D() :
        m_inLierCloud(new pcl::PointCloud<pcl::PointXYZI>()),
        m_outLierCloud(new pcl::PointCloud<pcl::PointXYZI>())
    {

    }
    ~Ransac3D()
    {

    }

    //actual algorithm
    inline void executeRansac3D(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int maxIterations, float distanceTol)
    {
        std::unordered_set<int> inliersResult;
        srand(time(NULL));

        auto comparePoint = [](pcl::PointXYZI& pA, pcl::PointXYZI& pB)
        {
             if(pA.x == pB.x &&
                pA.y == pB.y &&
                pA.z == pB.z)
                 return true;

             return false;
        };

        auto randomIntegers = [&cloud,comparePoint](pcl::PointXYZI& pA, pcl::PointXYZI& pB,pcl::PointXYZI& pC)
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
        for(int iteration = 0; iteration <= maxIterations;iteration++)
        {
        // Randomly sample subset and fit line
            pcl::PointXYZI pA; pA.x = 0; pA.y = 0; pA.z = 0;//stuetzpunkt
            pcl::PointXYZI pB; pB.x = 0; pB.y = 0; pB.z = 0;
            pcl::PointXYZI pC; pC.x = 0; pC.y = 0; pC.z = 0;
            randomIntegers(pA,pB,pC);

            pcl::PointXYZI v1, v2;//vectors from stuetzpunkt to  pB and pC
            v1.x = pB.x - pA.x; v1.y = pB.y - pA.y; v1.z = pB.z - pA.z;
            v2.x = pC.x - pA.x; v2.y = pC.y - pA.y; v2.z = pC.z - pA.z;
            pcl::PointXYZI vNormal;//normalen vector senkrecht auf ebene
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
                pcl::PointXYZI p = cloud.get()->points.at(a);
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
        size_t index = 0;
        for(auto& a : cloud->points)
        {
            if(inliersResult.count(index))
                m_inLierCloud->points.push_back(a);
            else
                m_outLierCloud->points.push_back(a);

            index++;
        }
    }
    //getter
    pcl::PointCloud<pcl::PointXYZI>::Ptr inLierCloud() const
    {
        return m_inLierCloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr outLierCloud() const
    {
        return m_outLierCloud;
    }

    private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr  m_inLierCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr  m_outLierCloud;
    };

    //Pure testing function
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



}


