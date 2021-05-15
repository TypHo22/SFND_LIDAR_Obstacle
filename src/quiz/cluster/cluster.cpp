/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"
#include "kdtree3D.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
        pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);


  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D(std::vector<std::vector<float>> points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for(int i = 0; i < points.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = points[i][0];
        point.y = points[i][1];
        point.z = points[i][2];

        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
        Box upperWindow = window;
        Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
            viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else
		{
            viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);
	}
}

void clusterHelper(int indice,const std::vector<std::vector<float>>& points,std::vector<int>& cluster,std::vector<bool>& processed, KdTree* tree, double distanceTol)
{

    processed[indice] = true;
    cluster.push_back(indice);

    std::vector<int> nearest = tree->search(points[indice],distanceTol);

    for(int id : nearest)
    {
        if(!processed[id])
            clusterHelper(id, points,cluster,processed,tree,distanceTol);
    }
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

    std::vector<bool> processed(points.size(),false);

    int i = 0;
    while(i < points.size())
    {

        if(processed[i])
        {
            i++;
            continue;
        }

        std::vector<int> cluster;
        clusterHelper(i,points, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
        i++;
    }
 
	return clusters;

}

int main ()
{

	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min =   0;
  	window.z_max =   0;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
    //std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };

    std::vector<std::vector<float>> points;
    srand (time(NULL));
    for(int i = 0; i < 5000; i++)
    {
        std::vector<float> f = {rand() % 10 - rand() % 10,rand() % 10 - rand() % 10 };
        points.push_back(f);
    }
    std::vector<std::vector<float>> points3D = {{3,1,4},
                                                {2,3,7},
                                                {4,3,4},
                                                {2,1,3},
                                                {2,4,5},
                                                {6,1,4},
                                                {1,4,4},
                                                {0,5,7},
                                                {5,2,5},
                                                {4,0,6},
                                                {7,1,6},
                                                {7,1,6.01},
                                               };

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D = CreateData3D(points3D);
    KdTree* tree = new KdTree;
    KdTree3D* tree3D = new KdTree3D;

    for(int i=0; i<cloud->points.size();i++)
        tree3D->insert(cloud->points[i],i);
  
    for (int i=0; i<points.size(); i++)
    	tree->insert(points[i],i); 





  	int it = 0;
  	render2DTree(tree->root,viewer,window, it);


  
  	std::cout << "Test Search" << std::endl;
    pcl::PointXYZ toSearch = {-6,7,0};
    std::vector<int> foundId = tree3D->search(toSearch,3.0);
    std::vector<int> nearby = tree->search({-6,7},3.0);
    foundId;
  	for(int index : nearby)
      std::cout << index << ",";
  	std::cout << std::endl;

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//
    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
  	//
  	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    auto startTimeE = std::chrono::steady_clock::now();
    //
     std::unique_ptr<std::vector<std::vector<pcl::PointXYZ*>>> mClusters = tree3D->euclideanCluster3D(3.0);
    //
    auto endTimeE = std::chrono::steady_clock::now();
    auto elapsedTimeE = std::chrono::duration_cast<std::chrono::milliseconds>(endTimeE - startTimeE);

    std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::cout << "clusteringOwn found " << clusters.size() << " and took " << elapsedTimeE.count() << " milliseconds" << std::endl;











  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
            clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
