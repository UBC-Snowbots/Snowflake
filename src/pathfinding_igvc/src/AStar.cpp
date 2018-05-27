/*
 * Created By: Min Gyo Kim
 * Created On: May 9th 2018
 * Description: Class that implements A* algorithm to find the shortest path
 * between two points given an occupancy grid
 */

#include <AStar.h>
using namespace std;

AStar::AStar(nav_msgs::OccupancyGrid occupancy_grid,
             GridPoint start,
             GridPoint goal) {
    this->_num_cols = occupancy_grid.info.width;
    this->_num_rows = occupancy_grid.info.height;
    this->_goal     = goal;
    this->_start    = start;
    this->_grid     = occupancy_grid.data;

    this->_cell_details = std::vector<std::vector<CellDetail>>(
    this->_num_rows, std::vector<CellDetail>(this->_num_cols));

    // Create an open list
    this->_open_list = set<GridPointWithScore>();

    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    this->_closed_list = std::vector<std::vector<bool>>(
    this->_num_rows, std::vector<bool>(this->_num_cols, false));
}

std::stack<AStar::GridPoint> AStar::run(nav_msgs::OccupancyGrid occupancy_grid,
                                        GridPoint start,
                                        GridPoint goal) {
    // The constructor is private because a path would only be found once
    // for a given occupancy grid and start and goal points. It's also a
    // way to force initialization of the class variables every time we
    // run the algorithm.
    return AStar(occupancy_grid, start, goal).search();
}

std::stack<AStar::GridPoint> AStar::search() {
    GridPoint start = this->_start;

    // Initialising the parameters of the starting node
    this->_cell_details[start.row][start.col].f      = 0.0;
    this->_cell_details[start.row][start.col].g      = 0.0;
    this->_cell_details[start.row][start.col].h      = 0.0;
    this->_cell_details[start.row][start.col].parent = start;

    this->_open_list.insert(
    std::make_pair(0.0, std::make_pair(start.row, start.col)));

    while (!this->_open_list.empty()) {
        GridPointWithScore p = *this->_open_list.begin();

        // Remove this vertex from the open list
        this->_open_list.erase(this->_open_list.begin());

        // Add this vertex to the open list
        int row                      = p.second.first;
        int col                      = p.second.second;
        this->_closed_list[row][col] = true;

        GridPoint parent(row, col);
        /*
        Generating all the 8 successor of this cell

            N.W   N   N.E
              \   |   /
               \  |  /
          W----CellDetail----E
                / | \
              /   |  \
           S.W    S   S.E

        CellDetail-->Popped CellDetail (row, col)
        N -->  North       (row-1, col)
        S -->  South       (row+1, col)
        E -->  East        (row, col+1)
        W -->  West           (row, col-1)
        N.E--> North-East  (row-1, col+1)
        N.W--> North-West  (row-1, col-1)
        S.E--> South-East  (row+1, col+1)
        S.W--> South-West  (row+1, col-1)*/

        //----------- 1st Successor (North) ------------
        if (processSuccessor(GridPoint(row - 1, col), parent)) {
            return tracePath();
        }

        //----------- 2nd Successor (South) ------------
        if (processSuccessor(GridPoint(row + 1, col), parent)) {
            return tracePath();
        }

        //----------- 3rd Successor (East) ------------

        if (processSuccessor(GridPoint(row, col + 1), parent)) {
            return tracePath();
        }

        //----------- 4th Successor (West) ------------

        if (processSuccessor(GridPoint(row, col - 1), parent)) {
            return tracePath();
        }

        //----------- 5th Successor (North-East) ------------

        if (processSuccessor(GridPoint(row - 1, col + 1), parent)) {
            return tracePath();
        }

        //----------- 6th Successor (North-West) ------------

        if (processSuccessor(GridPoint(row - 1, col - 1), parent)) {
            return tracePath();
        }

        //----------- 7th Successor (South-East) ------------

        if (processSuccessor(GridPoint(row + 1, col + 1), parent)) {
            return tracePath();
        }

        //----------- 8th Successor (South-West) ------------

        if (processSuccessor(GridPoint(row + 1, col - 1), parent)) {
            return tracePath();
        }
    }

    return std::stack<GridPoint>();
}

bool AStar::processSuccessor(GridPoint successor, GridPoint parent) {
    if (!isValid(successor)) return false;

    // If the destination cell is the same as the
    // current successor
    if (isDestination(successor)) {
        // Set the Parent of the destination cell
        this->_cell_details[successor.row][successor.col].parent = parent;
        return true;
    }
    // If the successor is already on the closed
    // list or if it is blocked, then ignore it.
    // Else do the following
    if (!this->_closed_list[successor.row][successor.col] &&
        isUnBlocked(successor)) {
        double g_new, h_new, f_new;
        g_new = this->_cell_details[parent.row][parent.col].g + 1.0;
        h_new = calculateHValue(successor);
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
            this->_open_list.insert(std::make_pair(
            f_new, std::make_pair(successor.row, successor.col)));

            // Update the details of this cell
            this->_cell_details[successor.row][successor.col].f      = f_new;
            this->_cell_details[successor.row][successor.col].g      = g_new;
            this->_cell_details[successor.row][successor.col].h      = h_new;
            this->_cell_details[successor.row][successor.col].parent = parent;
        }
    }

    return false;
}

// A Utility Function to trace the path from the source
// to destination
std::stack<AStar::GridPoint> AStar::tracePath() {
    int row = this->_goal.row;
    int col = this->_goal.col;

    std::stack<GridPoint> path;

    while (!(this->_cell_details[row][col].parent.row == row &&
             this->_cell_details[row][col].parent.col == col)) {
        path.push(GridPoint(row, col));

        int temp_row = this->_cell_details[row][col].parent.row;
        int temp_col = this->_cell_details[row][col].parent.col;

        row = temp_row;
        col = temp_col;
    }

    path.push(GridPoint(row, col));

    return path;
}

bool AStar::isUnBlocked(AStar::GridPoint point) {
    return this->_grid[point.row * this->_num_cols + point.col] == GRID_FREE;
}

bool AStar::isDestination(AStar::GridPoint point) {
    return this->_goal.row == point.row && this->_goal.col == point.col;
}

// A Utility Function to calculate the 'h' heuristics.
double AStar::calculateHValue(AStar::GridPoint point) {
    // Return using the euclidean distance formula
    return sqrt(pow(point.row - this->_goal.row, 2) +
                pow(point.col - this->_goal.col, 2));
}

bool AStar::isValid(AStar::GridPoint point) {
    // Returns true if row number and column number
    // is in range
    return (point.row >= 0) && (point.row < this->_num_rows) &&
           (point.col >= 0) && (point.col < this->_num_cols);
}
