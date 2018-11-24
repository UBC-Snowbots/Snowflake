//
// Created by william on 24/11/18.
//

#ifndef PROJECT_CONVEXHULL_H
#define PROJECT_CONVEXHULL_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>


class ConvexHull {
public:

    static geometry_msgs::Polygon convexHull(std::vector<geometry_msgs::Point32> points);



};


#endif //PROJECT_CONVEXHULL_H
