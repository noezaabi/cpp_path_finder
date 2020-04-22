#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
  
  	RouteModel::Node closest_start_node = m_Model.FindClosestNode(start_x, start_y);
  	RouteModel::Node closest_end_node = m_Model.FindClosestNode(end_x, end_y);

  	start_node = &closest_start_node;
  	end_node = &closest_end_node;
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  std::cout << current_node << "\n";
  // The code is breaking here
  // Finding the node's neighbors
  	RouteModel::Node node = *current_node;
	node.FindNeighbors();
  
  	// Iterating through them and adding them to the open_list
  	for(RouteModel::Node* neighbor : current_node -> neighbors){
      neighbor -> parent = current_node;
      neighbor -> g_value = current_node -> g_value + 1;
      neighbor -> h_value = CalculateHValue(neighbor);
      open_list.push_back(neighbor);
      current_node -> visited = true;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool CompareNode (RouteModel::Node* node_1, RouteModel::Node* node_2){
  float sum_1 = node_1 -> g_value + node_1 -> h_value;
  float sum_2 = node_2 -> g_value + node_2 -> h_value;
  
  return sum_1 > sum_2;
}

RouteModel::Node *RoutePlanner::NextNode() {
std::sort(open_list.begin(), open_list.end(), CompareNode);
  
  RouteModel::Node* optimal_node = open_list.back();
  open_list.erase(open_list.end(), open_list.end());
  return optimal_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found = {};
  	
  	// Create an empty parent_node pointer;
  	RouteModel::Node* parent_node = current_node -> parent;
  	
  	// Add the final node as the first element of the path vector
  	path_found.push_back(*current_node);
  
  	// Calculate the distance between the final node and it's parent
  	distance += current_node -> distance(*parent_node);
  
  	// Loop until we can't find any parent (to the initial node)
  	while(parent_node != nullptr) {
      // Update the distance
      distance += parent_node -> distance(*(parent_node -> parent));
      // Update the parent_node
      parent_node = parent_node -> parent;
      // Add the new node to the path vector if it isn't null
      if(parent_node != nullptr){
      path_found.push_back(*parent_node);
      }
    }
  	// Reverse the vector so that it is in the correct order
  	std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
  // Initializing the current node as the first one
    RouteModel::Node* current_node = start_node;
  
  	std::cout << start_node << "\n";
  
  	start_node -> visited = true;
  
  // Push it to the open_list
  	open_list.push_back(start_node);
  
  // Loop until there's no remaining solution
  	while(open_list.size() > 0 && current_node != nullptr){
       
      // Check if we are on end node
      if( current_node -> distance(*end_node) == 0){
        //In this case construct the for display path and break the loop
        m_Model.path = ConstructFinalPath(current_node);
        break;
      } else {
        // Else get the neighbors around our node
        AddNeighbors(current_node);
        // And choose the node with the lowest cost value to be the next one
        current_node = NextNode();
      }
    }
  
}