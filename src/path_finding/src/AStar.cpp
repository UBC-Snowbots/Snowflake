/*
 * Created By: Marcus Swift
 * Created On: December 18th, 2017
 * Description: An A Star optimal path finding node
 * takes in a occupancy grid and goal point and returns
 * the optimal path using an A star algorithm
 */

#include <AStar.h>

struct PathNode {
	double coor_x;
	double coor_y;
	double prev_coor_x; //node it came from
	double prev_coor_y;
	double curr_travel_cost;
	double priority_measure; //current travel cost + the heuristic
							 //which is displacement to the goal
	
};

struct GreaterThanPathNode {
	bool operator()(const PathNode& lhs, const PathNode& rhs) {
		return lhs.priority_measure > rhs.priority_measure;
	}
};

AStar::AStar(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Setup Subscribers
    occupancy_grid_sub = nh.subscribe("/grid", 10, &AStar::occupancyGridCallBack, this);
	goal_sub = nh.subscribe("/goal", 10, &AStar::occupancyGridCallBack, this);

    // Setup Publisher
    path_pub = nh.advertise<nav_msgs::Path>("/path", 10);
	
	MatrixXd h(10,10);
	h << 14,13,12,11,10,9,8,7,6,5,
		 13,12,11,10,9,8,7,6,5,4,
		 12,11,10,9,8,7,6,5,4,3,
		 11,10,9,8,7,6,5,4,3,2,
		 10,9,8,7,6,5,4,3,2,1,
		 9,8,7,6,5,4,3,2,1,0,
		 10,9,8,7,6,5,4,3,2,1,
		 11,10,9,8,7,6,5,4,3,2,
		 12,11,10,9,8,7,6,5,4,3,
		 13,12,11,10,9,8,7,6,5,4;
						  
						  
	MatrixXd o(10,10);
	o << 0,1,0,0,0,0,0,0,0,0,
		 0,1,0,0,0,0,0,1,0,0,
		 0,1,0,0,1,1,1,1,0,0,
		 0,1,0,0,0,0,0,1,0,0,
		 0,1,0,0,0,0,0,1,0,0,
		 0,1,0,0,1,1,1,0,0,0,
		 0,1,0,1,0,0,1,0,0,0,
		 0,1,0,1,0,0,0,1,0,0,
		 0,0,0,0,0,0,0,1,0,0,
		 0,0,0,0,0,0,0,1,0,0;
	
	vector<vector<PathNode> > map;

	map.resize(10);
	double i;
	double j;
	for (i = 0; i < 10; ++i) {
		map[i].resize(10);
	}

	// Put some values in
	for (i = 0; i < 10; i++) {
		for (j = 0; j < 10; j++) {
			map[i][j] = {i,j,-1,-1,99,99};
		}
	}

	
	double goal_coor_x = 9;
	double goal_coor_y = 5;
	std::priority_queue<PathNode, std::vector<PathNode>, GreaterThanPathNode> pq;
	
	//start point
	double init_x = 0;
	double init_y = 0;
	pq.push({init_x, init_y, -1, -1, 0, h(init_y, init_x)});
	PathNode curr;
	curr = pq.top();
	//std::cout << "(" << curr.coor_x+1 << ", " << curr.coor_y << ")" << std::endl;
	while (((curr.coor_x != goal_coor_x) || (curr.coor_y != goal_coor_y)) && (!pq.empty())) {
		//std::cout << "loop iteration" << std::endl;
		if((curr.coor_x != 0) && (o(curr.coor_y, curr.coor_x-1) != 1) &&
		   (curr.coor_x-1 != curr.prev_coor_x) && 
		   ((map[curr.coor_x-1][curr.coor_y]).priority_measure > curr.curr_travel_cost+h(curr.coor_y, curr.coor_x-1)+1)) {
			pq.push({curr.coor_x-1, curr.coor_y, curr.coor_x, curr.coor_y, curr.curr_travel_cost+1, 
			 curr.curr_travel_cost+h(curr.coor_y, curr.coor_x-1)+1});
			map[curr.coor_x-1][curr.coor_y] = {curr.coor_x-1, curr.coor_y, curr.coor_x, curr.coor_y, curr.curr_travel_cost+1, 
			 curr.curr_travel_cost+h(curr.coor_y, curr.coor_x-1)+1};
			/*std::cout << "added to priority queue (-x direction)" <<std::endl;
			std::cout << curr.coor_x-1 << " " << curr.coor_y << " " << curr.coor_x << " " << curr.coor_y
			 << " " << curr.curr_travel_cost+1 << " " 
			 << curr.curr_travel_cost+h(curr.coor_y, curr.coor_x-1)+1 << std::endl;
			*/
		}
		
		if((curr.coor_x != 9) && (o(curr.coor_y, curr.coor_x+1) != 1) &&
		   (curr.coor_x+1 != curr.prev_coor_x) && 
		   ((map[curr.coor_x+1][curr.coor_y]).priority_measure > curr.curr_travel_cost+h(curr.coor_y, curr.coor_x+1)+1)) {
			pq.push({curr.coor_x+1, curr.coor_y, curr.coor_x, curr.coor_y, curr.curr_travel_cost+1, 
			 curr.curr_travel_cost+h(curr.coor_y, curr.coor_x+1)+1});
			map[curr.coor_x+1][curr.coor_y] = {curr.coor_x+1, curr.coor_y, curr.coor_x, curr.coor_y, curr.curr_travel_cost+1, 
			 curr.curr_travel_cost+h(curr.coor_y, curr.coor_x+1)+1};
			/*std::cout << "added to priority queue (+x direction)" <<std::endl;
			std::cout << "(" << curr.coor_x+1 << ", " << curr.coor_y << ", " << curr.coor_x << ", " << curr.coor_y
			 << ", " << curr.curr_travel_cost+1 << ", " 
			 << curr.curr_travel_cost+h(curr.coor_y, curr.coor_x+1)+1 << ")" << std::endl;
			*/
		}
		
		if((curr.coor_y != 0) && (o(curr.coor_y-1, curr.coor_x) != 1) &&
		   (curr.coor_y-1 != curr.prev_coor_y) && 
		   ((map[curr.coor_x][curr.coor_y-1]).priority_measure > curr.curr_travel_cost+h(curr.coor_y-1, curr.coor_x)+1)) {
			pq.push({curr.coor_x, curr.coor_y-1, curr.coor_x, curr.coor_y, curr.curr_travel_cost+1, 
			 curr.curr_travel_cost+h(curr.coor_y-1, curr.coor_x)+1});
			map[curr.coor_x][curr.coor_y-1] = {curr.coor_x, curr.coor_y-1, curr.coor_x, curr.coor_y, curr.curr_travel_cost+1, 
			 curr.curr_travel_cost+h(curr.coor_y-1, curr.coor_x)+1};
			/*std::cout << "added to priority  (-y direction)" <<std::endl;
			std::cout << curr.coor_x << " " << curr.coor_y-1 << " " << curr.coor_x << " " << curr.coor_y
			 << " " << curr.curr_travel_cost+1 << " " 
			 << curr.curr_travel_cost+h(curr.coor_y-1, curr.coor_x)+1 << std::endl;
			*/
		}
		
		if((curr.coor_y != 9) && (o(curr.coor_y+1, curr.coor_x) != 1) &&
		   (curr.coor_y+1 != curr.prev_coor_y) && 
		   ((map[curr.coor_x][curr.coor_y+1]).priority_measure > curr.curr_travel_cost+h(curr.coor_y+1, curr.coor_x)+1)) {
			pq.push({curr.coor_x, curr.coor_y+1, curr.coor_x, curr.coor_y, curr.curr_travel_cost+1, 
			 curr.curr_travel_cost+h(curr.coor_y+1, curr.coor_x)+1});
			map[curr.coor_x][curr.coor_y+1] = {curr.coor_x, curr.coor_y+1, curr.coor_x, curr.coor_y, curr.curr_travel_cost+1, 
			 curr.curr_travel_cost+h(curr.coor_y+1, curr.coor_x)+1};
			/*std::cout << "added to priority queue (+y direction)" <<std::endl;
			std::cout << curr.coor_x << " " << curr.coor_y+1 << " " << curr.coor_x << " " << curr.coor_y
			 << " " << curr.curr_travel_cost+1 << " " 
			 << curr.curr_travel_cost+h(curr.coor_y+1, curr.coor_x)+1 << std::endl;
			*/
		}
		/*std::cout << "popped from priority queue" <<std::endl;
		std::cout << curr.coor_x << " " << curr.coor_y << " " << curr.prev_coor_x << " " << curr.prev_coor_y
		 << " " << curr.curr_travel_cost << " " << curr.priority_measure << std::endl;*/
		pq.pop();
		curr = pq.top();
		std::cout << "analysing this from top of priority queue" <<std::endl;
		std::cout << curr.coor_x << " " << curr.coor_y << " " << curr.prev_coor_x << " " << curr.prev_coor_y
		 << " " << curr.curr_travel_cost << " " << curr.priority_measure << std::endl;
	}
	
	std::cout << "optimal path" << std::endl;
	
	while (((map[curr.coor_x][curr.coor_y]).coor_x != init_x) || ((map[curr.coor_x][curr.coor_y]).coor_y != init_y)) {
		std::cout << "(" << curr.coor_x << ", " << curr.coor_y << ") ->" << std::endl;
		curr = map[curr.prev_coor_x][curr.prev_coor_y];
	}
	std::cout << "(" << curr.coor_x << ", " << curr.coor_y << ")" << std::endl;
	
}

void AStar::occupancyGridCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

}

void AStar::goalCallBack(const geometry_msgs::Point::ConstPtr& msg) {

}

void AStar::publishOptimalPath(nav_msgs::Path path_to_publish) {
    path_pub.publish(path_to_publish);
}
