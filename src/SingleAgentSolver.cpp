#include "SingleAgentSolver.h"


list<pair<int,double>> SingleAgentSolver::getNextLocations(int curr,double theta) const // including itself and its neighbors
{

	list<pair<int,double>> rst;
	list<list<pair<int, double> > > neighbors = instance.getPrimitives(curr,theta);
	for(auto n: neighbors){
		for(auto ni: n){
			rst.emplace_back(ni);
		}
	}
	return rst;	
}

void SingleAgentSolver::compute_heuristics()
{
	struct Node
	{
		int location;
		double theta;
		int value;

		Node() = default;
		Node(int location, double theta, int value) : location(location), theta(theta), value(value) {}
		// the following is used to compare nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
	};

	my_heuristic.resize(instance.map_size, MAX_TIMESTEP);

	// generate a heap that can save nodes (and an open_handle)
	boost::heap::pairing_heap< Node, boost::heap::compare<Node::compare_node> > heap;

	Node root(goal_location, 0, 0);
	my_heuristic[goal_location] = 0;
	heap.push(root);  // add root to heap
	while (!heap.empty())
	{
		Node curr = heap.top();
		heap.pop();
		for (auto next_location : instance.getPrimitives(curr.location,curr.theta))
		{
			pair <int,double> next_l = next_location.front();
			if (my_heuristic[next_l.first] > curr.value + 1)
			{
				my_heuristic[next_l.first] = curr.value + 1;
				Node next(next_l.first, next_l.second, curr.value + 1);
				heap.push(next);
			}
		}
	}
}