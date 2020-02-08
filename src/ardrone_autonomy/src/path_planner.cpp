
#include <stack>
#include <queue>
#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

typedef std::pair<int, int> Pair;

typedef std::pair<double, std::pair<int, int>> pPair;

struct Node
{
    Pair coords;
    int prev_x = -1;
    int prev_y = -1;

    float gCost = 1000000;
    float fCost = 1000000;
    float hCost = 1000000;

    Node* previousNode = NULL;
};

// comparison operator to make std::prioirty_queue a min priority queue
struct LessThanByCost
{
  bool operator()(const Node& lhs, const Node& rhs) const
  {
    return lhs.fCost > rhs.fCost;
  }
};

class PathPlanner{

  public:

  PathPlanner(octomap::OcTree* tree){

    double x, y, z;

    // example on how to loop through octree to get each voxel/node
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it){
      if(it->getOccupancy() > .5){
        // Fetching the coordinates in octomap-space
        std::cout << "  x = " << it.getX() << std::endl;
        std::cout << "  y = " << it.getY() << std::endl;
        std::cout << "  z = " << it.getZ() << std::endl;
        std::cout << "  size = " << it.getSize() << std::endl;
        std::cout << "  depth = " << it.getDepth() << std::endl;
      }
    }

    map = tree;
  }


  
  // todo:
  // finish function to reconstruct path

  /*std::vector<Pair> reconstruct_path(std::pair<int, int> **parents, int rows, int cols){
    std::vector<Pair> path;
    Pair p = current->coords;
    
    int x, y;
    x = current->coords.first;
    y = current->coords.second;

    int count = 0;

    path.push_back(p);
    while(current != NULL){
      std::cout << current->previousNode->coords.first << " " << y << std::endl;
      current = current->previousNode;
      x = current->coords.first;
      y = current->coords.second;
      p = current->coords;
      path.push_back(p);
      //previousNode = *(current.previousNode);
      count++;
    }
  }*/






  // A star pathing algorithm
  // Param: the starting point of the drone
  // Param: the destination of the drone
  // Return: A vector of coordinates that denote the path
  std::vector<Pair> A_star(octomap::point3d start, octomap::point3d end){

    std::priority_queue<Node, std::vector<Node>, LessThanByCost> openSet;

    bool visited[gridMap.size()][gridMap[0].size()] = {false};

    Pair parents[gridMap.size()][gridMap[0].size()];

    Node startNode;
    Pair src;
    src.first = (floor(start.x() * 10) + gridMap.size() / 2);
    src.second = (floor(start.y() * 10) + gridMap[0].size() / 2);

    visited[src.first][src.second] = true;

    Pair dest;
    dest.first = (floor(end.x() * 10) + gridMap.size() / 2);
    dest.second = (floor(end.y() * 10) + gridMap[0].size() / 2);

    visited[dest.first][dest.second] = true;

    startNode.coords = std::make_pair(src.first, src.second);
    startNode.gCost = 0;
    startNode.fCost = calculate_H(startNode.coords.first, startNode.coords.second, dest.first, dest.second);
    
    openSet.push(startNode);

    for(int i = 0; i < gridMap.size(); i++){
      for(int j = 0; j < gridMap[i].size(); j++){
        std::cout << visited[i][j] << " ";
      }
      std::cout << std::endl;
    }

    while(!openSet.empty()){ std::cout << openSet.size() << std::endl;

      Node current = openSet.top();
      
      std::cout << current.coords.first << " " << current.coords.second << std::endl;

      if(current.coords.first == dest.first && current.coords.second == dest.second){
        std::cout << "Dest found" << std::endl;

        //print grid of visited nodes
        for(int i = 0; i < gridMap.size(); i++){
          for(int j = 0; j < gridMap[i].size(); j++){
            std::cout << visited[i][j] << " ";
          }
          std::cout << std::endl;
        }

        std::vector<Pair> a;// temp variable so code will complile
        return a;// temp return

        // todo: 
        // return reconstruct_path(parents, gridMap.size(), gridMap[0].size());
      }

      openSet.pop();

      std::vector<Node> neighbors = getNeighbors(current.coords.first, current.coords.second);

      for(int i = 0; i < neighbors.size(); i++){

        //std::cout << (neighbors[i].point.x()) << std::endl;

        if(!isValid(neighbors[i]))
          continue;

        float tentative_gScore = current.gCost + 1;
     
        if(tentative_gScore < neighbors[i].gCost){
          neighbors[i].prev_x = current.coords.first;
          neighbors[i].prev_y = current.coords.second;
          neighbors[i].gCost = tentative_gScore;
          neighbors[i].fCost = neighbors[i].gCost + calculate_H(neighbors[i].coords.first, 
                                                                neighbors[i].coords.second, 
                                                                dest.first, dest.second);

          int x = neighbors[i].coords.first;
          int y = neighbors[i].coords.second;

          parents[x][y] = std::make_pair(current.coords.first, current.coords.second);

          if(!visited[x][y])
            visited[x][y] = true;
            neighbors[i].previousNode = &current;
            openSet.push(neighbors[i]);
        }
        std::cout << neighbors[i].previousNode->coords.first << std::endl;
      }
    }
    
  }

  // check if a node is an obstacle
  bool isValid(Node node){

    if(gridMap[node.coords.first][node.coords.second] > 0)
      return false;

    return true;
  }

  // calculate heuristic value
  float calculate_H(float start_x, float start_y, float end_x, float end_y){
    return (std::abs(end_x - start_x) +
           std::abs(end_y - start_y));
  }

  // find all adjacent neighbors to a node
  std::vector<Node> getNeighbors(int x, int y){

    std::vector<Node> neighbors;
    Node node;

    for(int colNum = x - 1; colNum <= x + 1; colNum++){
      for(int rowNum = y - 1; rowNum <= y + 1; rowNum++){

        if(!( (colNum == x) && (rowNum == y) )){

          if(! ((colNum < 0) && (rowNum < 0) &&
                (colNum > gridMap.size()) && (rowNum > gridMap[0].size())) ){

            node.coords.first = colNum;
            node.coords.second = rowNum;
            neighbors.push_back(node);
          }
        }
      }
    }

    return neighbors;
  }

  // generate a 2D occupancy gridmap from the 3D octomap
  // by projecting onto the x, y plane
  void generateGridMap(){
    double x, y, z;
    double res = map->getResolution();

    // dimensions of the map
    int length, width, height;
    
    map->getMetricSize(x, y, z);
    length = x / res;
    width = y / res;
    height = z / res;

    std::cout << length << " " << width << " " << height << " " << std::endl;

    // resize gridmap
    gridMap.resize(length);
    for(int i = 0; i < gridMap.size(); i++){
      gridMap[i].resize(width);
    }

    std::cout << gridMap.size() << " " << gridMap[0].size() << std::endl;

    for(octomap::OcTree::leaf_iterator it = map->begin_leafs(), end = map->end_leafs(); it != end; ++it){

      // only get highest resolution voxels with certain occupancy value
      if(it->getOccupancy() > .4 && it.getDepth() == 16){

        // only get voxels below certain height
        if(it.getZ() < .15){
          
          //convert map coordinate to gridmap index
          int x_index = (floor(it.getX() * 10) + length/ 2); std::cout << x_index << ", ";

          int y_index = (floor(it.getY() * 10) + width / 2); std::cout << y_index << std::endl;

          int offset = 1; //2 * it.getSize() / res;

          // plot obstacle in gridmap
          for(int i = 0; i < offset; i++){
            for(int j = 0; j < offset; j++){
              //std::cout << gridMap[i][j] << " ";
              if(x_index >= 0 && y_index >= 0 && x_index < length && y_index < width)
                gridMap[x_index][y_index] = 1;
            }
            std::cout << std::endl;
          }

        }
      }
    }

    // print gridmap
    for(int i = 0; i < gridMap.size(); i++){
      for(int j = 0; j < gridMap[i].size(); j++){
        std::cout << gridMap[i][j] << " ";
      }
      std::cout << std::endl;
    }

  }

  // convert to ROS geometry_msgs::Point32
  octomap::point3d ros_to_oc_point(geometry_msgs::Point32 point){
    return octomap::point3d(point.x, point.y, point.z);
  }

  // return the octomap
  octomap::OcTree* getMap(){
    return map;
 }

  private:

  octomap::OcTree* map; //pointer to the octomap

  std::vector<std::vector<float>> gridMap; //2D occupancy gridmap

};

int main(int argc, char** argv)
{
  octomap::OcTree ocMap = octomap::OcTree("simple_tree.bt"); //get octomap from file

  PathPlanner planner = PathPlanner(&ocMap);

  //std::vector<octomap::point3d> ray;

  octomap::point3d start = octomap::point3d(-2, -2, -2); // start point in map coordinates

  octomap::point3d end = octomap::point3d(3, 3, 3); // end point in map cordinates

  planner.generateGridMap();

  planner.A_star(start, end);

  return 0;
}


