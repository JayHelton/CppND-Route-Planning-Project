#include "route_model.h"
#include <iostream>
//ok
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    int counter = 0;
    for(Model::Node node : this->Nodes()) {
        m_Nodes.push_back(Node(counter, this, node));
        counter++;
    }
    CreateNodeToRoadHashMap();
};

//ok
float RouteModel::Node::distance(Node dest) const {
    return std::sqrt(std::pow((x - dest.x), 2) + std::pow((y - dest.y), 2));
}

void RouteModel::CreateNodeToRoadHashMap()
{


	//std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
	for(const Model::Road& road : Roads()){
		if(road.type != Model::Road::Type::Footway){

		for(auto node_idx : Ways()[road.way].nodes)
		{
			if(node_to_road.find(node_idx) == node_to_road.end())
				node_to_road[node_idx] = std::vector<const Model::Road *> {};
			
			node_to_road[node_idx].push_back(&road);
		}
	}
    }
	return;
    //std::cout<<"function STOP ::"<<__FUNCTION__<<std::endl;
};


RouteModel::Node * RouteModel::Node::FindNeighbor(std::vector<int> node_indices)
{
	std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
	RouteModel::Node *closest_node = nullptr;
	Node node;

	for(int node_index : node_indices)
	{
		node = parent_model->SNodes()[node_index];

		if(this->distance(node) != 0 && !node.visited)
			if(closest_node == nullptr || this->distance(node) < this->distance(*closest_node))
				closest_node = &parent_model->SNodes()[node_index];
	}
std::cout<<"function STOP ::"<<__FUNCTION__<<std::endl;
	return closest_node;
};

void RouteModel::Node::FindNeighbors(){
	std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
	for(auto &road : parent_model->node_to_road[this->index])
	{
		RouteModel::Node * new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);

		if(new_neighbor) this->neighbors.emplace_back(new_neighbor);
	}
	std::cout<<"function STOP ::"<<__FUNCTION__<<std::endl;
	return;
}



RouteModel::Node& RouteModel::FindClosestNode(float x, float y) {
    RouteModel::Node input;
    input.x = x;
    input.y = y;

    float min_dis = std::numeric_limits<float>::max();
    float dis;
    int closest_idx;

    std::cout << "Moving into for loop to find closest node\n";

    for(const Model::Road& road : Roads()) {
        if(road.type != Model::Road::Type::Footway) {
            for(int node_idx : Ways()[road.way].nodes) {
                std::cout << "Calculating distance\n";
                dis = input.distance(SNodes()[node_idx]);
                if(dis < min_dis) {
                    std::cout << "updating closest_idx\n";
                    closest_idx = node_idx;
                    min_dis = dis;
                }
            }
        }
    }
    return SNodes()[closest_idx];
}

