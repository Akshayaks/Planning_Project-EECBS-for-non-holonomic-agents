#include<boost/tokenizer.hpp>
#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include"Instance.h"
#include <stdio.h>
#include <iostream>
#include <math.h>

int RANDOM_WALK_STEPS = 100000;

Instance::Instance(const string& map_fname, const string& agent_fname, 
	int num_of_agents, int num_of_rows, int num_of_cols, int num_of_obstacles, int warehouse_width):
	map_fname(map_fname), agent_fname(agent_fname), num_of_agents(num_of_agents)
{
	bool succ = loadMap();
	if (!succ)
	{
		if (num_of_rows > 0 && num_of_cols > 0 && num_of_obstacles >= 0 && 
			num_of_obstacles < num_of_rows * num_of_cols) // generate random grid
		{
			generateConnectedRandomGrid(num_of_rows, num_of_cols, num_of_obstacles);
			saveMap();
		}
		else
		{
			cerr << "Map file " << map_fname << " not found." << endl;
			exit(-1);
		}
	}

	succ = loadAgents();
	if (!succ)
	{
		if (num_of_agents > 0)
		{
			generateRandomAgents(warehouse_width);
			saveAgents();
		}
		else
		{
			cerr << "Agent file " << agent_fname << " not found." << endl;
			exit(-1);
		}
	}

}


// int Instance::randomWalk(int curr, int steps) const
// {
// 	for (int walk = 0; walk < steps; walk++)
// 	{
// 		list<int> l = getNeighbors(curr);
// 		vector<int> next_locations(l.cbegin(), l.cend());
// 		auto rng = std::default_random_engine{};
// 		std::shuffle(std::begin(next_locations), std::end(next_locations), rng);
// 		for (int next : next_locations)
// 		{
// 			if (validMove(curr, next))
// 			{
// 				curr = next;
// 				break;
// 			}
// 		}
// 	}
// 	return curr;
// }

void Instance::generateRandomAgents(int warehouse_width)
{
	cout << "Generate " << num_of_agents << " random start and goal locations " << endl;
	vector<bool> starts(map_size, false);
	vector<bool> goals(map_size, false);
	start_locations.resize(num_of_agents);
	goal_locations.resize(num_of_agents);

	if (warehouse_width == 0)//Generate agents randomly
	{
		// Choose random start locations
		int k = 0;
		while ( k < num_of_agents)
		{
			int x = rand() % num_of_rows, y = rand() % num_of_cols;
			int start = linearizeCoordinate(x, y);
			if (my_map[start] || starts[start])
				continue;
				
			// update start
			start_locations[k] = start;
			starts[start] = true;

			// find goal
			bool flag = false;
			int goal = rand() % map_size; // randomWalk(start, RANDOM_WALK_STEPS);
			while (my_map[goal] || goals[goal])
				goal = rand() % map_size; // randomWalk(goal, 1);

			//update goal
			goal_locations[k] = goal;
			goals[goal] = true;

			k++;
		}
	}
	else //Generate agents for warehouse scenario
	{
		// Choose random start locations
		int k = 0;
		while (k < num_of_agents)
		{
			int x = rand() % num_of_rows, y = rand() % warehouse_width;
			if (k % 2 == 0)
				y = num_of_cols - y - 1;
			int start = linearizeCoordinate(x, y);
			if (starts[start])
				continue;
			// update start
			start_locations[k] = start;
			starts[start] = true;

			k++;
		}
		// Choose random goal locations
		k = 0;
		while (k < num_of_agents)
		{
			int x = rand() % num_of_rows, y = rand() % warehouse_width;
			if (k % 2 == 1)
				y = num_of_cols - y - 1;
			int goal = linearizeCoordinate(x, y);
			if (goals[goal])
				continue;
			// update goal
			goal_locations[k] = goal;
			goals[goal] = true;
			k++;
		}
	}
}

bool Instance::validMove(int curr, int next) const
{
	if (next < 0 || next >= map_size)
	{
		// cout <<"\nOut of the map!";
		return false;
	}
	if(my_map[next]){
		return false;
	}
	if(getManhattanDistance(curr,next) == 2){
		vector<int> n;
		int sum = curr + next;
		n.push_back(curr-1);
		n.push_back(curr+1);
		n.push_back(curr-num_of_cols);
		n.push_back(curr+num_of_cols);
		cout << "curr: " << getRowCoordinate(curr) << " " << getColCoordinate(curr) << endl;
		cout << "next: " << getRowCoordinate(next) << " " << getColCoordinate(next) << endl;

		for(int i=0;i<n.size();i++){
			int end = sum - n[i];
			if(end < 0 || end > map_size || end == n[i] || n[i] < 0 || n[i] > map_size){
				continue;
			}
			if(getManhattanDistance(n[i],end) > 2){
				continue;
			}
			else{
				cout << "\nn[i]: " << getRowCoordinate(n[i]) << " " << getColCoordinate(n[i]) << endl;
				cout << "\nend: " << getRowCoordinate(end) << " " << getColCoordinate(end) << endl;
				if(my_map[n[i]] && my_map[end]){
					cout << "\nCannot go through";
					
					return false;
				}
			}
		}
	}
	//Change cond to max theta change and max path length?
	return getManhattanDistance(curr, next) <= 3;
}

bool Instance::addObstacle(int obstacle)
{
	if (my_map[obstacle])
		return false;
	my_map[obstacle] = true;
	int obstacle_x = getRowCoordinate(obstacle);
	int obstacle_y = getColCoordinate(obstacle);
	int x[4] = { obstacle_x, obstacle_x + 1, obstacle_x, obstacle_x - 1 };
	int y[4] = { obstacle_y - 1, obstacle_y, obstacle_y + 1, obstacle_y };
	int start = 0;
	int goal = 1;
	while (start < 3 && goal < 4)
	{
		if (x[start] < 0 || x[start] >= num_of_rows || y[start] < 0 || y[start] >= num_of_cols 
			|| my_map[linearizeCoordinate(x[start], y[start])])
			start++;
		else if (goal <= start)
			goal = start + 1;
		else if (x[goal] < 0 || x[goal] >= num_of_rows || y[goal] < 0 || y[goal] >= num_of_cols 
			|| my_map[linearizeCoordinate(x[goal], y[goal])])
			goal++;
		else if (isConnected(linearizeCoordinate(x[start], y[start]), linearizeCoordinate(x[goal], y[goal]))) // cannot find a path from start to goal 
		{
			start = goal;
			goal++;
		}
		else
		{
			my_map[obstacle] = false;
			return false;
		}
	}
	return true;
}

bool Instance::isConnected(int start, int goal)
{
	std::queue<pair<int,double>> open;
	vector<bool> closed(map_size, false);
	open.push(make_pair(start,0));
	closed[start] = true;
	while (!open.empty())
	{
		auto curr = open.front(); open.pop();
		if (curr.first == goal)
			return true;
		for (auto next : getPrimitives(curr.first,curr.second))
		{
			pair<int, double> next_location = next.front();
			if (closed[next_location.first])
				continue;
			open.push(next_location);
			closed[next_location.first] = true;
		}
	}
	return false;
}

void Instance::generateConnectedRandomGrid(int rows, int cols, int obstacles)
{
	cout << "Generate a " << rows << " x " << cols << " grid with " << obstacles << " obstacles. " << endl;
	int i, j;
	num_of_rows = rows + 2;
	num_of_cols = cols + 2;
	map_size = num_of_rows * num_of_cols;
	my_map.resize(map_size, false);
	// Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
	/*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
	moves_offset[Instance::valid_moves_t::EAST] = 1;
	moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
	moves_offset[Instance::valid_moves_t::WEST] = -1;*/

	// add padding
	i = 0;
	for (j = 0; j<num_of_cols; j++)
		my_map[linearizeCoordinate(i, j)] = true;
	i = num_of_rows - 1;
	for (j = 0; j<num_of_cols; j++)
		my_map[linearizeCoordinate(i, j)] = true;
	j = 0;
	for (i = 0; i<num_of_rows; i++)
		my_map[linearizeCoordinate(i, j)] = true;
	j = num_of_cols - 1;
	for (i = 0; i<num_of_rows; i++)
		my_map[linearizeCoordinate(i, j)] = true;

	// add obstacles uniformly at random
	i = 0;
	while (i < obstacles)
	{
		int loc = rand() % map_size;
		if (addObstacle(loc))
		{
			printMap();
			i++;
		}
	}
}

bool Instance::loadMap()
{
	using namespace boost;
	using namespace std;
	ifstream myfile(map_fname.c_str());
	if (!myfile.is_open())
		return false;
	string line;
	tokenizer< char_separator<char> >::iterator beg;
	getline(myfile, line);
	if (line[0] == 't') // Nathan's benchmark
	{
		char_separator<char> sep(" ");
		getline(myfile, line);
		tokenizer< char_separator<char> > tok(line, sep);
		beg = tok.begin();
		beg++;
		num_of_rows = atoi((*beg).c_str()); // read number of rows
		getline(myfile, line);
		tokenizer< char_separator<char> > tok2(line, sep);
		beg = tok2.begin();
		beg++;
		num_of_cols = atoi((*beg).c_str()); // read number of cols
		getline(myfile, line); // skip "map"
	}
	else // my benchmark
	{
		char_separator<char> sep(",");
		tokenizer< char_separator<char> > tok(line, sep);
		beg = tok.begin();
		num_of_rows = atoi((*beg).c_str()); // read number of rows
		beg++;
		num_of_cols = atoi((*beg).c_str()); // read number of cols
	}
	map_size = num_of_cols * num_of_rows;
	my_map.resize(map_size, false);
	// read map (and start/goal locations)
	for (int i = 0; i < num_of_rows; i++) {
		getline(myfile, line);
		for (int j = 0; j < num_of_cols; j++) {
			my_map[linearizeCoordinate(i, j)] = (line[j] != '.');
		}
	}
	myfile.close();

	// initialize moves_offset array
	/*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
	moves_offset[Instance::valid_moves_t::EAST] = 1;
	moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
	moves_offset[Instance::valid_moves_t::WEST] = -1;*/
	return true;
}


void Instance::printMap() const
{
	for (int i = 0; i< num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (this->my_map[linearizeCoordinate(i, j)])
				cout << '@';
			else
				cout << '.';
		}
		cout << endl;
	}
}


void Instance::saveMap() const
{
	ofstream myfile;
	myfile.open(map_fname);
	if (!myfile.is_open())
	{
		cout << "Fail to save the map to " << map_fname << endl;
		return;
	}
	myfile << num_of_rows << "," << num_of_cols << endl;
	for (int i = 0; i < num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (my_map[linearizeCoordinate(i, j)])
				myfile << "@";
			else
				myfile << ".";
		}
		myfile << endl;
	}
	myfile.close();
}


bool Instance::loadAgents()
{
	using namespace std;
	using namespace boost;

	string line;
	ifstream myfile (agent_fname.c_str());
	if (!myfile.is_open()) 
	return false;

	getline(myfile, line);
	if (line[0] == 'v') // Nathan's benchmark
	{
		if (num_of_agents == 0)
		{
			cerr << "The number of agents should be larger than 0" << endl;
			exit(-1);
		}
		start_locations.resize(num_of_agents);
		goal_locations.resize(num_of_agents);
		char_separator<char> sep("\t");
		for (int i = 0; i < num_of_agents; i++)
		{
			getline(myfile, line);
			tokenizer< char_separator<char> > tok(line, sep);
			tokenizer< char_separator<char> >::iterator beg = tok.begin();
			beg++; // skip the first number
			beg++; // skip the map name
			beg++; // skip the columns
			beg++; // skip the rows
				   // read start [row,col] for agent i
			int col = atoi((*beg).c_str());
			beg++;
			int row = atoi((*beg).c_str());
			start_locations[i] = linearizeCoordinate(row, col);
			// read goal [row,col] for agent i
			beg++;
			col = atoi((*beg).c_str());
			beg++;
			row = atoi((*beg).c_str());
			goal_locations[i] = linearizeCoordinate(row, col);
		}
	}
	else // My benchmark
	{
		char_separator<char> sep(",");
		tokenizer< char_separator<char> > tok(line, sep);
		tokenizer< char_separator<char> >::iterator beg = tok.begin();
		num_of_agents = atoi((*beg).c_str());
		start_locations.resize(num_of_agents);
		goal_locations.resize(num_of_agents);
		for (int i = 0; i<num_of_agents; i++)
		{
			getline(myfile, line);
			tokenizer< char_separator<char> > col_tok(line, sep);
			tokenizer< char_separator<char> >::iterator c_beg = col_tok.begin();
			pair<int, int> curr_pair;
			// read start [row,col] for agent i
			int row = atoi((*c_beg).c_str());
			c_beg++;
			int col = atoi((*c_beg).c_str());
			start_locations[i] = linearizeCoordinate(row, col);
			// read goal [row,col] for agent i
			c_beg++;
			row = atoi((*c_beg).c_str());
			c_beg++;
			col = atoi((*c_beg).c_str());
			goal_locations[i] = linearizeCoordinate(row, col);
		}
	}
	myfile.close();
	return true;

}


void Instance::printAgents() const
{
  for (int i = 0; i < num_of_agents; i++) 
  {
    cout << "Agent" << i << " : S=(" << getRowCoordinate(start_locations[i]) << "," << getColCoordinate(start_locations[i]) 
				<< ") ; G=(" << getRowCoordinate(goal_locations[i]) << "," << getColCoordinate(goal_locations[i]) << ")" << endl;
  }
}


void Instance::saveAgents() const
{
  ofstream myfile;
  myfile.open(agent_fname);
  if (!myfile.is_open())
  {
	  cout << "Fail to save the agents to " << agent_fname << endl;
	  return;
  }
  myfile << num_of_agents << endl;
  for (int i = 0; i < num_of_agents; i++)
    myfile << getRowCoordinate(start_locations[i]) << "," << getColCoordinate(start_locations[i]) << ","
           << getRowCoordinate(goal_locations[i]) << "," << getColCoordinate(goal_locations[i]) << "," << endl;
  myfile.close();
}


list<int> Instance::getNeighbors(int curr) const
{
	list<int> neighbors;
	int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols};
	for (int next : candidates)
	{
		if (validMove(curr, next))
			neighbors.emplace_back(next);
	}
	return neighbors;
}

list<int> Instance::get_eight_Neighbors(int curr) const
{
	list<int> neighbors;
	int candidates[8] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols, curr + num_of_cols-1, curr - num_of_cols-1,curr + num_of_cols+1, curr - num_of_cols+1};
	for (int next : candidates)
	{
		if (validMove(curr, next))
			neighbors.emplace_back(next);
	}
	return neighbors;
}

// list<pair<int,double>> Instance::getPrimitives(int loc, int timestep, int theta) const
// {
// 	list<pair<int,double>> neighbors;
// 	int x = getRowCoordinate(loc);
// 	int y = getColCoordinate(loc);
// 	// cout << this->num_of_cols;

// 	if(theta < 0.1){
// 		theta = 0;
// 	}
// 	// cout << "\ncurrx: " << x;
// 	// cout << "\ncurry: " << y;
// 	// cout << "\ntheta: " << theta;

// 	neighbors.emplace_back(make_pair(loc,theta));

// 	if(theta == 0){
// 		int n1 = (x)*this->num_of_cols+y+1;
// 		// cout << "\n" << x < " " << y+1;
// 		if (validMove(loc, n1)){
// 			// // cout  "\nvalid 1";
// 			neighbors.emplace_back(make_pair(n1,0));
// 		}

// 		int n2 = (x-1)*this->num_of_cols+y+1;
// 		// cout << "\n" << x-1 << " " << y+1;
// 		if (validMove(loc, n2)){
// 			// // cout  "\nvalid 2";
// 			neighbors.emplace_back(make_pair(n2,45));
// 		}

// 		int n3 = (x+1)*this->num_of_cols+y+1;
// 		// cout << "\n" << x+1 << " " << y+1;
// 		if (validMove(loc, n3)){
// 			// // cout  "\nvalid 3";
// 			neighbors.emplace_back(make_pair(n3,315));
// 		}
// 	}

// 	else if(theta == 45){
// 		int n1 = (x-1)*this->num_of_cols+y+1;
// 		// cout << "\n" << x-1 << " " << y+1;
// 		if (validMove(loc, n1)){
// 			// // cout  "\nvalid 1";
// 			neighbors.emplace_back(make_pair(n1,45));
// 		}

// 		int n2 = (x-1)*this->num_of_cols+y;
// 		// cout << "\n" << x-1 << " " << y;
// 		if (validMove(loc, n2)){
// 			// // cout  "\nvalid 2";
// 			neighbors.emplace_back(make_pair(n2,90));
// 		}

// 		int n3 = (x)*this->num_of_cols+y+1;
// 		// cout << "\n" << x << " " << y+1;
// 		if (validMove(loc, n3)){
// 			// cout << "\nvlaid 3";
// 			neighbors.emplace_back(make_pair(n3,0));
// 		}
// 	}

// 	else if(theta == 90){
// 		int n1 = (x-1)*this->num_of_cols+y;
// 		// cout << "\n" << x-1 << " " << y;
// 		if (validMove(loc, n1)){
// 			// // cout  "\nvalid 1";
// 			neighbors.emplace_back(make_pair(n1,90));
// 		}

// 		int n2 = (x-1)*this->num_of_cols+y-1;
// 		// cout << "\n" << x-1 << " " << y-1;
// 		if (validMove(loc, n2)){
// 			// // cout  "\nvalid 2";
// 			neighbors.emplace_back(make_pair(n2,135));
// 		}

// 		int n3 = (x-1)*this->num_of_cols+y+1;
// 		// cout << "\n" << x-1 << " " << y+1;
// 		if (validMove(loc, n3)){
// 			// // cout  "\nvalid 3";
// 			neighbors.emplace_back(make_pair(n3,45));
// 		}
// 	}

// 	else if(theta == 135){
// 		int n1 = (x-1)*this->num_of_cols+y-1;
// 		// cout << "\n" << x-1 << " " << y-1;
// 		if (validMove(loc, n1)){
// 			// cout << "\nvlaid 1";
// 			neighbors.emplace_back(make_pair(n1,135));
// 		}

// 		int n2 = (x)*this->num_of_cols+y-1;
// 		// cout << "\n" << x << " " << y-1;
// 		if (validMove(loc, n2)){
// 			// // cout  "\nvalid 2";
// 			neighbors.emplace_back(make_pair(n2,180));
// 		}

// 		int n3 = (x-1)*this->num_of_cols+y;
// 		// cout << "\n" << x-1 << " " << y;
// 		if (validMove(loc, n3)){
// 			// // cout  "\nvalid 3";
// 			neighbors.emplace_back(make_pair(n3,90));
// 		}
// 	}

// 	else if(theta == 180){
// 		int n1 = (x)*this->num_of_cols+y-1;
// 		// cout << "\n" << x << " " << y-1;
// 		if (validMove(loc, n1)){
// 			// // cout  "\nvalid 1";
// 			neighbors.emplace_back(make_pair(n1,180));
// 		}

// 		int n2 = (x-1)*this->num_of_cols+y-1;
// 		// cout << "\n" << x-1 << " " << y-1;
// 		if (validMove(loc, n2)){
// 			// // cout  "\nvalid 2";
// 			neighbors.emplace_back(make_pair(n2,135));
// 		}

// 		int n3 = (x+1)*this->num_of_cols+y-1;
// 		// cout << "\n" << x+1 << " " << y-1;
// 		if (validMove(loc, n3)){
// 			// // cout  "\nvalid 3";
// 			neighbors.emplace_back(make_pair(n3,225));
// 		}
// 	}

// 	else if(theta == 225){
// 		int n1 = (x+1)*this->num_of_cols+y-1;
// 		// cout << "\n" << x+1 << " " << y-1;
// 		if (validMove(loc, n1)){
// 			// // cout  "\nvalid 1";
// 			neighbors.emplace_back(make_pair(n1,225));
// 		}

// 		int n2 = (x)*this->num_of_cols+y-1;
// 		// cout << "\n" << x << " " << y-1;
// 		if (validMove(loc, n2)){
// 			// // cout  "\nvalid 2";
// 			neighbors.emplace_back(make_pair(n2,180));
// 		}

// 		int n3 = (x+1)*this->num_of_cols+y;
// 		// cout << "\n" << x+1 << " " << y;
// 		if (validMove(loc, n3)){
// 			// // cout  "\nvalid 3";
// 			neighbors.emplace_back(make_pair(n3,270));
// 		}
// 	}

// 	else if(theta == 270){
// 		int n1 = (x+1)*this->num_of_cols+y;
// 		// cout << "\n" << x+1 << " " << y;
// 		if (validMove(loc, n1)){
// 			// // cout  "\nvalid 1";
// 			neighbors.emplace_back(make_pair(n1,270));
// 		}

// 		int n2 = (x+1)*this->num_of_cols+y-1;
// 		// cout << "\n" << x+1 << " " << y-1;
// 		if (validMove(loc, n2)){
// 			// cout << "\nvlaid 2";
// 			neighbors.emplace_back(make_pair(n2,225));
// 		}

// 		int n3 = (x+1)*this->num_of_cols+y+1;
// 		// cout << "\n" << x+1 << " " << y+1;
// 		if (validMove(loc, n3)){
// 			// // cout  "\nvalid 3";
// 			neighbors.emplace_back(make_pair(n3,315));
// 		}
// 	}

// 	else if(theta == 315){
// 		int n1 = (x+1)*this->num_of_cols+y+1;
// 		// cout << "\n" << x+1 << " " << y+1;
// 		if (validMove(loc, n1)){
// 			// // cout  "\nvalid 1";
// 			neighbors.emplace_back(make_pair(n1,315));
// 		}

// 		int n2 = (x)*this->num_of_cols+y+1;
// 		// cout << "\n" << x << " " << y+1;
// 		if (validMove(loc, n2)){
// 			// cout  "\nvalid 2";
// 			neighbors.emplace_back(make_pair(n2,0));
// 		}

// 		int n3 = (x+1)*this->num_of_cols+y;
// 		// cout << "\n" << x+1 << " " << y;
// 		if (validMove(loc, n3)){
// 			// cout  "\nvalid 3";
// 			neighbors.emplace_back(make_pair(n3,270));
// 		}
// 	}

// 	int recalc_loc = this->num_of_cols*x + y;
// 	// cout << "\nRecalc loc: " << recalc_loc;

// 	// cout << "\nNeighbors are: " << neighbors.size();
// 	// if(neighbors.size() > 0){
	
// 	// 	for(auto n:neighbors){
// 	// 		cout << "\nx: " << getRowCoordinate(n.first);
// 	// 		cout << "\ny: " << getColCoordinate(n.first);
// 	// 		cout << "\ntheta: " << n.second;
// 	// 	}
// 	// 	cout << "\n";
// 	// }
	

// 	//Say we have a maximum velocity limit of 10 and omega limit of 2
// 	return neighbors;
// }

list<pair<int, double> > Instance::getBezierPathCells(int loc1, double ang1, int loc2, double ang2) const
{
	float x0 = (float) getRowCoordinate(loc1);
	float y0 = (float) getColCoordinate(loc1);
	float x3 = (float) getRowCoordinate(loc2);
	float y3 = (float) getColCoordinate(loc2);

	// first control point
	float cx1 = x0 - sin(DEG2RAD(ang1));
	float cy1 = y0 + cos(DEG2RAD(ang1));
	// second control point - 180 is added since 2nd control point is behind the end point
	float cx2 = x3 - sin(DEG2RAD(WRAPTO360(ang2 + 180.0f)));
	float cy2 = y3 + cos(DEG2RAD(WRAPTO360(ang2 + 180.0f)));

	auto cubicBezier = [] (float x0, float x1, float x2, float x3, float ratio) {
		return pow((1 - ratio), 3)*x0 + 3*pow((1 - ratio), 2)*ratio*x1 + 3*(1 - ratio)*pow(ratio, 2)*x2 + pow(ratio, 3)*x3;
	};

	vector<pair<float, float> > bezier_path;

	float x, y;
	for(float ratio = 0; ratio <= 1.001; ratio += 0.05)
	{
		x = cubicBezier(x0, cx1, cx2, x3, ratio);
		y = cubicBezier(y0, cy1, cy2, y3, ratio);
		bezier_path.push_back(make_pair(x, y));
	}

	// iterate over bezier path to see which cells does it cross
	int last_x = (int) x0, last_y = (int) y0, next_loc;
	list<pair<int, double> > cellPath; // {last_x * num_of_cols + last_y} - don't need to add start loc
	for(auto pt : bezier_path)
	{
		tie(x, y) = pt;
		if(((int) round(x) != last_x) or ((int) round(y) != last_y))
		{
			// change in cell
			last_x = (int) round(x);
			last_y = (int) round(y);
			next_loc = last_x * num_of_cols + last_y;
			cellPath.push_back(make_pair(next_loc, ang2));
		}
	}

	return cellPath;
}

list<list<pair<int, double> > > Instance::getPrimitives(int loc, double theta) const
{
	list<list<pair<int, double> > > neighbors;
	int x = getRowCoordinate(loc);
	int y = getColCoordinate(loc);

	if(abs(theta) < 0.1) {
		theta = 0;
	}

	// staying at the current location is also a neighbor
	neighbors.push_back(list<pair<int, double> >{make_pair(loc, theta)});

	vector<pair<double, double> > angle_step_pair{
								make_pair(WRAPTO360(theta - D_THETA), -1),
								make_pair(WRAPTO360(theta - D_THETA), 1),
								make_pair(WRAPTO360(theta - D_THETA/2), -sqrt(5)), // -22.5, step = sqrt(5) --> hypot(2, 1)
								make_pair(WRAPTO360(theta - D_THETA/2), sqrt(5)), // -22.5, step = sqrt(5) --> hypot(2, 1)
								make_pair(theta, -1),
								make_pair(theta, 1),
								make_pair(WRAPTO360(theta + D_THETA/2), -sqrt(5)), // +22.5, step = sqrt(5) --> hypot(2, 1)
								make_pair(WRAPTO360(theta + D_THETA/2), sqrt(5)), // +22.5, step = sqrt(5) --> hypot(2, 1)
								make_pair(WRAPTO360(theta + D_THETA), -1),
								make_pair(WRAPTO360(theta + D_THETA), 1)};
	int new_x, new_y, new_loc, angle;
	double step;
	for(auto angle_step : angle_step_pair)
	{
		tie(angle, step) = angle_step;
		// sin, cos reversed since x is down (row), y is right (col)
		new_x = (int) round((double) x - step*sin(DEG2RAD(angle)));
		new_y = (int) round((double) y + step*cos(DEG2RAD(angle)));
		new_loc = new_x * num_of_cols + new_y;

		if(abs(step) <= 1.01)
		{
			if(validMove(loc, new_loc))
			{
				neighbors.push_back(list<pair<int, double> >{make_pair(new_loc, angle)});
			}
		}
		// else
		// {
		// 	// longer primitive
		// 	// calculate bezier curve - get cells - check each cell transition for validity
		// 	if(validMove(loc, new_loc))
		// 	{
		// 		cout << "\nLonger primitive " << loc << " " << new_loc << endl;
		// 		list<pair<int, double> > cellPath = getBezierPathCells(loc, theta, new_loc, angle);
		// 		bool valid = true;
		// 		cellPath.push_front(make_pair(loc, theta)); // add start loc to the path to check transition validity
		// 		auto it = cellPath.begin();
		// 		while(next(it) != cellPath.end())
		// 		{
		// 			if(!validMove(it->first, next(it)->first))
		// 			{
		// 				valid = false;
		// 				break;
		// 			}
		// 			++it;
		// 		}

		// 		if(valid) // path is valid
		// 		{
		// 			// add neighbor
		// 			cellPath.pop_front(); // remove start loc before adding to neighbors
		// 			neighbors.push_back(cellPath);
		// 		}
		// 	}
		// }
	}

	return neighbors;
}