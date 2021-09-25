#ifndef _GRAPH_SEARCH_H_
#define _GRAPH_SEARCH_H_

#include "graph_search/planner_interface.h"
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>



class Node
{
public:
  Node(){};
  Node(Node*, int, int, int, int);
  void H_Set(int, int);
  void G_Set();
  void F_Set();
  void Update(Node*);
  int x_,y_;
  int g_,h_,f_;
  Node *father_node_;
};


class GraphSearch : public PlannerInterface
{
public:
  GraphSearch();
  virtual ~GraphSearch();
  // entrance to A*
  bool Search(int startX, int startY, int goalX, int goalY, unsigned char** occMap, int width, int height,
              unsigned char obsthresh, std::vector<PointInt>& path, std::vector<PointInt>& expands);
  // find the node with min F
  bool Figure(std::vector<PointInt>& path, std::vector<PointInt>& expands);
  // backpropogate to find the path
  void FindRes(std::vector<PointInt>& path, std::vector<PointInt>& expands, Node* handle_node);
  // extend to 8 adjecent grids
  void Extend(Node*);
private:
  std::vector<Node*> openlist_, closelist_;
  Node* start_node_, *goal_node_;
  int width_, height_;
  int startX_, startY_, goalX_, goalY_;
  unsigned char** occMap_;
  ros::Publisher expands_pub_;
};




#endif  // _GRAPH_SEARCH_H_