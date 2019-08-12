
#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
//Convert inputs to percentege
std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
start_x *= 0.01;
start_y *= 0.01;
end_x *= 0.01;
end_y *= 0.01;


start_node = &m_Model.FindClosestNode(start_x, start_y);
end_node = &m_Model.FindClosestNode(end_x, end_y);
std::cout<<"function STOP ::"<<__FUNCTION__<<std::endl;
};

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node){
//create path found vector
std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
distance = 0.0f;
std::vector<RouteModel::Node> path_found;
RouteModel::Node parent;

while (current_node -> parent !=nullptr){

    path_found.push_back(*current_node);
    parent = *(current_node->parent);
    distance += current_node -> distance(parent);
    current_node = current_node->parent;
}
path_found.push_back(*current_node);
distance *=m_Model.MetricScale();
std::cout<<"function STOP ::"<<__FUNCTION__<<std::endl;
return path_found;

};



/////////////


void RoutePlanner::AStarSearch(){
std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
//initialize open_list with starting node.
start_node ->visited =true;
open_list.push_back(start_node);
RouteModel::Node *current_node = nullptr;

// Expand nodes until the goal is reached. Use heuristic to proritize what mode to open first.

while(open_list.size()>0) {

    // Selct the best node to expolore next

    current_node = NextNode();

    //Check if the node selcted is the goal.
    if (current_node ->distance(*end_node) == 0){

        //set the model path variable with the path found
        
        m_Model.path =ConstructFinalPath(current_node);
        return;
    }
    AddNeighbors(current_node); 
}
std::cout<<"function STOP ::"<<__FUNCTION__<<std::endl;
};



float RoutePlanner::CalculateHValue(RouteModel::Node *node){
std::cout<<"function START / STOP::"<<__FUNCTION__<<std::endl;
    return node->distance(*end_node);
};

RouteModel::Node* RoutePlanner::NextNode(){
std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
    std::sort(open_list.begin(), open_list.end(), [](const auto &_1st, const auto &_2nd){
        return _1st->h_value + _1st->g_value < _2nd->h_value +_2nd->g_value;
    });

    RouteModel::Node *lowest_node = open_list.front();
    open_list.erase(open_list.begin());
    std::cout<<"function STOP ::"<<__FUNCTION__<<std::endl;
    return lowest_node;
};

void RoutePlanner::AddNeighbors(RouteModel::Node * current_node){
std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
// Expand the current mmode (add all unvisted neighbors to the open list)
current_node->FindNeighbors();

for(auto neighbor : current_node->neighbors){
    neighbor->parent = current_node;
    neighbor ->g_value = current_node->g_value + current_node->distance(*neighbor);
    neighbor-> h_value = CalculateHValue(neighbor);


    //Add the neighbor to the open list.
    open_list.push_back(neighbor);
    neighbor->visited = true;

}

};

