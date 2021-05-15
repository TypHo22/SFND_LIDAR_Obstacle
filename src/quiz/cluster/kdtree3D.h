#include <memory.h>

class Node3d
{
public:

    typedef std::unique_ptr<Node3d> Node3dPtr;
    //Constructor
    Node3d() :
        m_id(-1),m_depth(-1),
        m_left(nullptr),m_right(nullptr)
    {

    }

    Node3d(pcl::PointXYZI data,int id, int depth) :
        m_data(data),m_id(id),m_depth(depth),
        m_left(nullptr),m_right(nullptr)
    {

    }

    ~Node3d()
    {
        m_left.release();
        m_right.release();
    }

    pcl::PointXYZI data() const
    {return m_data;}

    pcl::PointXYZI* dataPtr()
    {
        pcl::PointXYZI* ptr = &m_data;
        return ptr;
    }

    int id() const
    {return m_id;}


public: //public member
    Node3dPtr m_left;
    Node3dPtr m_right;

private:  //private member
    pcl::PointXYZI m_data;

    int m_id;
    int m_depth;

};


class KdTree3D
{
public:
    KdTree3D() :
        root(nullptr)
    {

    }

    ~KdTree3D()
    {
       root.release();
    }
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////Insert/////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
private:
    inline void insertHelper (Node3d::Node3dPtr& node,
                              uint depth,pcl::PointXYZI point, int id)
    {
        if(!node)
        {
             node = Node3d::Node3dPtr(new Node3d(point,id,depth));
             pointOverview.push_back(node.get());
        }
        else
        {
            uint cd = depth % 3;

            switch(cd)
            {
                case(0)://compare x
                {
                    if(point.x < node.get()->data().x)
                        insertHelper(node->m_left,depth + 1, point, id);
                    else
                        insertHelper(node->m_right,depth + 1, point, id);

                    break;
                }
                case(1)://compare y
                {
                    if(point.y < node.get()->data().y)
                        insertHelper(node->m_left,depth + 1, point, id);
                    else
                        insertHelper(node->m_right,depth + 1, point, id);

                    break;
                }
                default://compare z
                {
                    if(point.z < node.get()->data().z)
                        insertHelper(node->m_left,depth + 1, point, id);
                    else
                        insertHelper(node->m_right,depth + 1, point, id);

                    break;
                }
            }
        }
    }
public:
    inline void insert(pcl::PointXYZI point, int id)
    {
        if(point.x == 6 &&
                point.y == 3 &&
                point.z == 1)
        {
            int stop = 3;
        }
        insertHelper(root,0,point,id);
    }

/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////Search/////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
private:
    inline bool checkBox3D(const pcl::PointXYZI& target,const pcl::PointXYZI& pointToCheck,
                               float distanceTol) const
    {

        if(target.x + distanceTol >= pointToCheck.x &&
           target.x - distanceTol <= pointToCheck.x &&
           target.y + distanceTol >= pointToCheck.y &&
           target.y - distanceTol <= pointToCheck.y &&
           target.z + distanceTol >= pointToCheck.z &&
           target.z - distanceTol <= pointToCheck.z)
            return true;

        return false;
    }
private:

    inline bool checkEuclideanDistance3D(const pcl::PointXYZI& target,const pcl::PointXYZI& pointToCheck,
                               float distanceTol) const
    {
        const double vec[3] =  {{target.x - pointToCheck.x},
                                {target.y - pointToCheck.y},
                                {target.z - pointToCheck.z}};
        if(std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]) <= distanceTol)
            return true;

        return false;
    }

    inline void searchHelper(pcl::PointXYZI target, Node3d::Node3dPtr& node,
                             uint depth, float distanceTol, std::vector<int>&ids)
    {

        if(!node)
            return;

        if(checkBox3D(target,node->data(),distanceTol))
        {
            if(checkEuclideanDistance3D(target,node->data(),distanceTol))
                ids.push_back(node->id());
        }

        //check boundaries
        const int boundarie = depth % 3;

        switch(boundarie)
        {
            case(0): //compare x
            {

                if((target.x - distanceTol) < node->data().x)
                    searchHelper(target,node->m_left,depth + 1,distanceTol,ids);
                if((target.x + distanceTol) > node->data().x)
                    searchHelper(target,node->m_right,depth + 1,distanceTol,ids);

                break;
            }
            case(1): //compare y
            {
                if((target.y - distanceTol) < node->data().y)
                    searchHelper(target,node->m_left,depth + 1,distanceTol,ids);
                if((target.y + distanceTol) > node->data().y)
                    searchHelper(target,node->m_right,depth + 1,distanceTol,ids);

                break;
            }
            default: //compare z
            {
                if((target.z - distanceTol) < node->data().z)
                    searchHelper(target,node->m_left,depth + 1,distanceTol,ids);
                if((target.z + distanceTol) > node->data().z)
                    searchHelper(target,node->m_right,depth + 1,distanceTol,ids);

               break;
            }
        }

    }
public:
    inline std::vector<int> search(pcl::PointXYZI target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);

        return ids;
    }

/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////Clustering/////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
private:
void clusterHelper(int indice, std::vector<bool>&processed,std::vector<int>& clusterind, double distanceTol)
{
    processed[indice] = true;
    clusterind.push_back(indice);

    std::vector<int> nearest = this->search(pointOverview[indice]->data(),distanceTol);

    for(int id : nearest)
    {
       if(!processed[id])
           clusterHelper(id,processed,clusterind,distanceTol);
    }
}
public:
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanCluster3D(float distanceTol, int minSize, int maxSize)
{
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> foundClusters;

    if(pointOverview.empty())
        return foundClusters;

    std::vector<bool> processed(pointOverview.size(),false);
    int i = 0;

    while(i < pointOverview.size())
    {
        if(processed[i])
        {
            i++;
            continue;
        }
        std::vector<int> cluster_ind;

        clusterHelper(i,processed,cluster_ind,distanceTol);
        if(cluster_ind.size() >= minSize &&
           cluster_ind.size() <= maxSize)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for(auto& a : cluster_ind)
            {
                cluster->points.push_back(pointOverview[a]->data());
            }
            cluster->width = cluster->points.size();
            cluster->height = 1;
            foundClusters.push_back(cluster);
        }

        i++;
    }


    return foundClusters;
}

private:
        Node3d::Node3dPtr root;
        std::vector<Node3d*> pointOverview;//a non tree overwiew
};


