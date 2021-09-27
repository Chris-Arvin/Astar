#include "graph_search/graph_search.h"
#include <math.h>

Node::Node(Node* father_node, int x, int y, int goalX, int goalY){
  this->x_ = x;
  this->y_ = y;
  this->father_node_ = father_node;
  this->H_Set(goalX, goalY);
  this->G_Set();
  this->F_Set();
}

void Node::H_Set(int goalX, int goalY){
  this->h_ = 10*std::max(std::abs(this->x_-goalX), std::abs(this->y_-goalY));
  // this->h_ = 10*sqrt(pow((this->x_-goalX),2) + pow((this->y_-goalY),2));
}

void Node::G_Set(){
  if(this->father_node_ == NULL){
    this->g_ = 0;
  }
  else{
    if (std::abs(this->father_node_->x_ - this->x_) + std::abs(this->father_node_->y_ - this->y_) == 1){
      this->g_ = this->father_node_->g_ + 10;
    }
    else{
      this->g_ = this->father_node_->g_ + 14;
    }
  }
}

void Node::F_Set(){
  this->f_ = this->g_ + this->h_;
}

void Node::Update(Node* new_node){
  // may exist a better father node
  if (this->g_ > new_node->g_){
    this->g_ = new_node->g_;
    this->father_node_ = new_node->father_node_;
    this->F_Set();
  }
}



GraphSearch::GraphSearch()
{
}

GraphSearch::~GraphSearch()
{
}

bool GraphSearch::Search(int startX, int startY, int goalX, int goalY, unsigned char** occMap, int width, int height,
                         unsigned char obsthresh, std::vector<PointInt>& path, std::vector<PointInt>& expands)
{
  start_node_ = new Node(NULL, startX, startY, goalX, goalY);
  goal_node_ = new Node(NULL, goalX, goalY, goalX, goalY);
  occMap_ = occMap;
  width_ = width;
  height_ = height;
  startX_ = startX;
  startY_ = startY;
  goalX_ = goalX;
  goalY_ = goalY;
  bool is_success = this->Figure(path, expands);
  return is_success;
}


bool GraphSearch::Figure(std::vector<PointInt>& path, std::vector<PointInt>& expands){
  ros::NodeHandle nhh;
  expands_pub_ = nhh.advertise<visualization_msgs::Marker>("expands", 1);

  Node* handle_node = this->start_node_;
  this->openlist_.push_back(handle_node);
  while (true){
    if (handle_node->x_ == goal_node_->x_ && handle_node->y_ == goal_node_->y_){
      std::cout<<"distance(g): "<<handle_node->g_<<std::endl;
      this->FindRes(path, expands, handle_node);
      return true;
    }
    if (this->openlist_.size()==0){
      return false;
    }
    this->Extend(handle_node);
    handle_node = this->openlist_[0];
    for (std::vector<Node*>::iterator it=openlist_.begin(); it!=openlist_.end(); it++){
      if ((*it)->f_ <= handle_node->f_){
        handle_node = *it;
      }
    }

    // for visualization
    float resolution_=0.05;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = resolution_ * 0.5;
    marker.scale.y = resolution_ * 0.5;
    marker.scale.z = resolution_ * 0.5;
    marker.color.a = 1.0;
    marker.color.b = 1.0;
    marker.color.g = 0.0;
    marker.color.r = 0.0;
    marker.points.resize(closelist_.size());
    for (int i = 0; i < closelist_.size(); i++)
    {
      double x = DISCXY2CONT(closelist_[i]->x_, resolution_) + -19;
      double y = DISCXY2CONT(closelist_[i]->y_, resolution_) + -19;
      marker.points[i].x = x;
      marker.points[i].y = y;
    }
    expands_pub_.publish(marker);
  }
}


void GraphSearch::Extend(Node* handle_node){
  // remove handle_node from openlist to closelist
  for (std::vector<Node*>::iterator it=openlist_.begin(); it!=openlist_.end(); it++){
    if(*it == handle_node){
      this->openlist_.erase(it);
      break;
    }
  }
  this->closelist_.push_back(handle_node);
  // extend adjecent grids
  for(int x=handle_node->x_-1; x<=handle_node->x_+1; x++){
    for(int y=handle_node->y_-1; y<=handle_node->y_+1; y++){
      if (x==handle_node->x_ && y==handle_node->y_){
        continue;
      }
      if (x<0 || y<0 || x>=width_ || y>= height_){
        continue;
      }
      if (occMap_[x][y]>0){
        continue;
      }
      Node* new_node = new Node(handle_node, x, y, goalX_, goalY_);
      int flag1=0;
      int flag2=0;
      int flag3=0;
      // update check
      for (std::vector<Node*>::iterator it=openlist_.begin(); it!=openlist_.end(); it++){
        if ((*it)->x_ == new_node->x_ && (*it)->y_ == new_node->y_){
          (*it)->Update(new_node);
          flag1=1;
          break;
        } 
      }
      // in-closelist check
      for (std::vector<Node*>::iterator it=closelist_.begin(); it!=closelist_.end(); it++){
        if ((*it)->x_ == new_node->x_ && (*it)->y_ == new_node->y_){
          flag2=1;
          break;
        }
      }
      // X cross check
      if (std::abs(new_node->x_-handle_node->x_)==1 && std::abs(new_node->y_-handle_node->y_)==1){
        if (occMap_[new_node->x_][handle_node->y_]>0 && occMap_[handle_node->x_][new_node->y_]>0){
          flag3=1;
        }
      }
      if (flag1==0 && flag2==0 and flag3==0){
        this->openlist_.push_back(new_node);
      }
    }
  }
}
void GraphSearch::FindRes(std::vector<PointInt>& path, std::vector<PointInt>& expands, Node* handle_node){
  while (true){
    path.push_back(PointInt(handle_node->x_,handle_node->y_));
    if (handle_node->father_node_ == NULL){
      break;
    }
    handle_node = handle_node->father_node_;
  }
  
  for (std::vector<Node*>::iterator it=closelist_.begin(); it!=closelist_.end(); it++){
    expands.push_back(PointInt((*it)->x_, (*it)->y_));
  }
}