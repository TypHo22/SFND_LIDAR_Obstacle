/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;
    int m_depth;
    Node(std::vector<float> arr, int setId, int depth)
    :	point(arr), id(setId), left(NULL), right(NULL), m_depth(depth)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

    /**
     * @brief insertHelper
     * Diese Funktion wandert mit dem datensatz point
     * solange den kdtree hinab bis es einen leeren
     * childplatz findet (*node==NULL), dann wird an dieser stelle eine neue Node
     * erstellt. Ist es nicht der fall das ich eine leere node vorfinde, dann
     * bin ich noch nicht an einem einfuegbaren kdtree childplatz, es wird von der
     * aktuellen node aber das entsprechende child rekursiv aufgerufen.
     * @param node, aktuelle node im kdtree (kann auch leer sein, also NULL)
     * @param depth, tiefe im kdtree
     * @param point, datensatz, in diesem fall xy
     * @param id, der node welche ich einfuege. fuer den eigentlichen kdtree fuer die suche wichtig
     */
    inline void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
    {
        //tree is empty
        if(*node == NULL)
            *node = new Node(point,id,depth);//enter new child node, or root node if first loop
        else
        {
            uint cd = depth % 2;//gets used to compare x or y value by depth
                                //even number -> compare x
                                //odd number -> compare y

            if(point[cd] < ((*node)->point[cd]))
                insertHelper(&((*node)->left), depth+1, point,id);
            else
                insertHelper(&((*node)->right),depth+1,point,id);
        }
    }

    inline void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insertHelper(&root,0,point,id);
	}

    /**
     * @brief checkBox
     * create a 2d bounding box areound target and check
     * if pointToCheck is in it
     * @param target, point you create the bounding box around
     * @param pointToCheck point which needs to be checked
     * @param distanceTol size of bounding box
     * @return true = pointToCheck is in boundingBox
     */
    inline bool checkBox(std::vector<float>&target,std::vector<float>&pointToCheck,float distanceTol) const
    {
        if(target[0] + distanceTol >= pointToCheck[0]&&
           target[0] - distanceTol <= pointToCheck[0]&&
           target[1] + distanceTol >= pointToCheck[1]&&
           target[1] - distanceTol <= pointToCheck[1])
            return true;

        return false;
    }

    /**
     * @brief euclideanDistance
     * @param target
     * @param pointToCheck
     * @param distanceTol
     * @return is in range or not
     */
    inline bool euclideanDistance(std::vector<float>&target,std::vector<float>&pointToCheck,float distanceTol) const
    {
        const double vec[2] = {pointToCheck[0] - target[0], pointToCheck[1] - target[1]};

        if(std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]) <= distanceTol)
            return true;

        return false;
    }

    void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if(node != NULL)
        {
            if(checkBox(target,node->point,distanceTol))
            {
                if(euclideanDistance(target,node->point,distanceTol))
                    ids.push_back(node->id);
            }

            //check boundaries
            const int ad = depth % 2;

            if((target[ad] - distanceTol) < node->point[ad])
                   searchHelper(target,node->left,depth+1,distanceTol,ids);

            if((target[ad] + distanceTol) > node->point[ad])
                searchHelper(target,node->right,depth+1,distanceTol,ids);
        }

    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	

};




