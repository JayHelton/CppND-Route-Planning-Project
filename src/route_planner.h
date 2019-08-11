#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    float GetDistance() const {

      return distance;
    }
    void AStarSearch();
    

  private:
    // Add private variables or methods declarations here.
    void AddNeighbors(RouteModel::Node *current_node);
    RouteModel &m_Model;
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;
    float distance;
    std::vector<RouteModel::Node*> open_list;

    //ConstructFinalPath should accept the pointer RouteModel::Node *current_node as the argument,
    // and it should return a vector of RouteModel::Node objects
    float CalculateHValue (RouteModel::Node *node);

    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);

    RouteModel::Node* NextNode();    
    


};
