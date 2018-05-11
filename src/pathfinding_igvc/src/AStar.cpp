/*
 * Created By: Min Gyo Kim
 * Created On: May 9th 2018
 * Description: Class that implements A* algorithm to find the shortest path
 * between two points given an occupancy grid
 */

#include <AStar.h>
using namespace std;

AStar::AStar(nav_msgs::OccupancyGrid occupancy_grid, GridPoint start,
        GridPoint goal) {
    this->_num_cols = occupancy_grid.info.width;
    this->_num_rows = occupancy_grid.info.height;
    this->_goal = goal;
    this->_start = start;
    this->_grid = occupancy_grid.data;

    this->_cell_details = new Cell*[this->_num_rows];
    for (int i = 0; i < this->_num_rows; i++) {
        this->_cell_details[i] = new Cell[this->_num_cols];
    }

    /*
     * TODO: edit comment
     Create an open list having information as-
     <f, <row, col>>
     where f = g + h,
     and row, col are the row and column index of that cell
     Note that 0 <= row <= ROW-1 & 0 <= col <= COL-1
     This open list is implenented as a set of pair of pair.*/
    this->_open_list = set<GridPointWithScore>();

    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    this->_closed_list = new bool*[this->_num_rows];
    for (int i = 0; i < this->_num_rows; i++) {
        this->_closed_list[i] = new bool[this->_num_cols];
    }
}

std::stack<AStar::GridPoint> AStar::run(nav_msgs::OccupancyGrid occupancy_grid,
                                        GridPoint start,
                                        GridPoint goal) {
//    this->_num_cols = occupancy_grid.info.width;
//    this->_num_rows = occupancy_grid.info.height;
//    this->_goal = goal;
//    this->_start = start;
//    this->_grid = occupancy_grid.data;
//
//    this->_cell_details = new Cell*[this->_num_rows];
//    for (int i = 0; i < this->_num_rows; i++) {
//        this->_cell_details[i] = new Cell[this->_num_cols];
//    }
//
//    /*
//     * TODO: edit comment
//     Create an open list having information as-
//     <f, <row, col>>
//     where f = g + h,
//     and row, col are the row and column index of that cell
//     Note that 0 <= row <= ROW-1 & 0 <= col <= COL-1
//     This open list is implenented as a set of pair of pair.*/
//    this->_open_list = set<GridPointWithScore>();
//
//    // Create a closed list and initialise it to false which means
//    // that no cell has been included yet
//    // This closed list is implemented as a boolean 2D array
//    this->_closed_list = new bool*[this->_num_rows];
//    for (int i = 0; i < this->_num_rows; i++) {
//        this->_closed_list[i] = new bool[this->_num_cols];
//    }



//    return aStarSearch();

    return AStar(occupancy_grid, start, goal).aStarSearch();
}

std::stack<AStar::GridPoint> AStar::aStarSearch() {
    GridPoint start = this->_start;

    // Initialising the parameters of the starting node
    this->_cell_details[start.row][start.col].f = 0.0;
    this->_cell_details[start.row][start.col].g = 0.0;
    this->_cell_details[start.row][start.col].h = 0.0;
    this->_cell_details[start.row][start.col].parent.row = start.row;
    this->_cell_details[start.row][start.col].parent.col = start.col;

    this->_open_list.insert(std::make_pair(0.0, std::make_pair(start.row, start.col)));

    while (!this->_open_list.empty()) {
        GridPointWithScore p = *this->_open_list.begin();

        // Remove this vertex from the open list
        this->_open_list.erase(this->_open_list.begin());

        // Add this vertex to the open list
        int row = p.second.first;
        int col = p.second.second;
        this->_closed_list[row][col] = true;

        GridPoint parent(row, col);
        /*
        Generating all the 8 successor of this cell

            N.W   N   N.E
              \   |   /
               \  |  /
            W----Cell----E
                 / | \
               /   |  \
            S.W    S   S.E

        Cell-->Popped Cell (row, col)
        N -->  North       (row-1, col)
        S -->  South       (row+1, col)
        E -->  East        (row, col+1)
        W -->  West           (row, col-1)
        N.E--> North-East  (row-1, col+1)
        N.W--> North-West  (row-1, col-1)
        S.E--> South-East  (row+1, col+1)
        S.W--> South-West  (row+1, col-1)*/

        //----------- 1st Successor (North) ------------
        if (processSuccessor(GridPoint(row-1, col), parent)) {
            return tracePath();
        }

        //----------- 2nd Successor (South) ------------
        if (processSuccessor(GridPoint(row+1, col), parent)) {
            return tracePath();
        }

        //----------- 3rd Successor (East) ------------

        if (processSuccessor(GridPoint(row, col+1), parent)) {
            return tracePath();
        }

        //----------- 4th Successor (West) ------------

        if (processSuccessor(GridPoint(row, col-1), parent)) {
            return tracePath();
        }

        //----------- 5th Successor (North-East) ------------

        if (processSuccessor(GridPoint(row-1, col+1), parent)) {
            return tracePath();
        }

        //----------- 6th Successor (North-West) ------------

        if (processSuccessor(GridPoint(row-1, col-1), parent)) {
            return tracePath();
        }

        //----------- 7th Successor (South-East) ------------

        if (processSuccessor(GridPoint(row+1, col+1), parent)) {
            return tracePath();
        }

        //----------- 8th Successor (South-West) ------------

        if (processSuccessor(GridPoint(row+1, col-1), parent)) {
            return tracePath();
        }
    }

//    return nav_msgs::Path();
    return std::stack<GridPoint>();
}

bool AStar::processSuccessor(GridPoint successor, GridPoint parent) {
    if (!isValid(successor.row, successor.col)) return false;

    // If the destination cell is the same as the
    // current successor
    if (isDestination(successor.row, successor.col)) {
        // Set the Parent of the destination cell
        this->_cell_details[successor.row][successor.col].parent.row = parent.row;
        this->_cell_details[successor.row][successor.col].parent.col = parent.col;
        return true;
    }
        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
    if (!this->_closed_list[successor.row][successor.col] &&
             isUnBlocked(successor.row, successor.col)) {
        double g_new, h_new, f_new;
        g_new = this->_cell_details[parent.row][parent.col].g + 1.0;
        h_new = calculateHValue(successor.row, successor.col);
        f_new = g_new + h_new;

        // If it isn’t on the open list, add it to
        // the open list. Make the current square
        // the parent of this square. Record the
        // f, g, and h costs of the square cell
        //                OR
        // If it is on the open list already, check
        // to see if this path to that square is better,
        // using 'f' cost as the measure.
        if (this->_cell_details[successor.row][successor.col].f == FLT_MAX ||
            this->_cell_details[successor.row][successor.col].f > f_new) {
            this->_open_list.insert(std::make_pair(f_new, std::make_pair(successor.row, successor.col)));

            // Update the details of this cell
            this->_cell_details[successor.row][successor.col].f = f_new;
            this->_cell_details[successor.row][successor.col].g = g_new;
            this->_cell_details[successor.row][successor.col].h = h_new;
            this->_cell_details[successor.row][successor.col].parent.row = parent.row;
            this->_cell_details[successor.row][successor.col].parent.col = parent.col;
        }
    }

    return false;
}

// A Utility Function to trace the path from the source
// to destination
std::stack<AStar::GridPoint> AStar::tracePath()
{
    int row = this->_goal.row;
    int col = this->_goal.col;

//    std::vector<geometry_msgs::PoseStamped> poses;
    std::stack<GridPoint> path;

    while (!(this->_cell_details[row][col].parent.row == row
             && this->_cell_details[row][col].parent.col == col ))
    {
//        geometry_msgs::Point position;
//        position.x = col;
//        position.y = row;
//
//        geometry_msgs::Pose pose;
//        pose.position = position;
//
//        geometry_msgs::PoseStamped pose_stamped;
//        pose_stamped.pose = pose;

//        poses.push_back(pose_stamped);
        path.push( GridPoint(col, row) );

        row = this->_cell_details[row][col].parent.row;
        col = this->_cell_details[row][col].parent.col;
    }

//    geometry_msgs::Point position;
//    position.x = col;
//    position.y = row;
//
//    geometry_msgs::Pose pose;
//    pose.position = position;
//
//    geometry_msgs::PoseStamped pose_stamped;
//    pose_stamped.pose = pose;
//
//    poses.push_back(pose_stamped);
//
//    nav_msgs::Path path;
//    path.poses = poses;
//
//    return path;
    path.push( GridPoint(col, row) );

    return path;
}

bool AStar::isUnBlocked(int row, int col) {
    return this->_grid[row*this->_num_cols + col] == GRID_FREE;
}

bool AStar::isDestination(int row, int col) {
    return this->_goal.row == row && this->_goal.col == col;
}

// A Utility Function to calculate the 'h' heuristics.
double AStar::calculateHValue(int row, int col) {
    // Return using the diagonal distance formula
    return std::max(
            abs(row - this->_goal.row),
            abs(col - this->_goal.col)
    );
}

bool AStar::isValid(int row, int col) {
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < this->_num_rows) &&
           (col >= 0) && (col < this->_num_cols);
}
