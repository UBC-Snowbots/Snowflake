/**
 * Created by William Gu on 24/03/18
 * Implementation of Cone Identification methods for identifying cones from a laser msg
 */

#include <ConeIdentification.h>


std::vector<mapping_igvc::ConeObstacle> ConeIdentification::identifyCones(const sensor_msgs::LaserScan &laser_msg, double dist_tol, double radius_exp, double radius_tol, int min_points_in_cone, double ang_threshold){
    std::vector<mapping_igvc::ConeObstacle> identified_cones;
    std::vector<mapping_igvc::Point2D> edge_points; //Represents points in a potential cluster
    std::string frame_id = laser_msg.header.frame_id;

    int numIndices = (laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment;
    for (int i=0; i<numIndices; i++){
        if (laser_msg.ranges[i] > laser_msg.range_max || laser_msg.ranges[i] < laser_msg.range_min) { //Check if curr laserscan point is in invalid range
            addConesInEdgeCluster(identified_cones, edge_points, radius_exp, radius_tol, min_points_in_cone, ang_threshold, frame_id);
            edge_points.clear();
        }
        else { //Laserscan point in range

            mapping_igvc::Point2D point = laserToPoint(laser_msg.ranges[i], laser_msg.angle_min + i * laser_msg.angle_increment); //Convert to x-y point

            if (!edge_points.empty() && getDist(edge_points.back(), point) > dist_tol){ //If out of dist tolerance, analyze points so far, clear, then add new point
                addConesInEdgeCluster(identified_cones, edge_points, radius_exp, radius_tol, min_points_in_cone,
                                      ang_threshold, frame_id);
                edge_points.clear();
                edge_points.push_back(point);
            }
            else if (i == numIndices - 1) { //If out of points, analyze edge points so far to create cone, and clear edge_points
                edge_points.push_back(point);
                addConesInEdgeCluster(identified_cones, edge_points, radius_exp, radius_tol, min_points_in_cone,
                                      ang_threshold, frame_id);
                edge_points.clear();
            }
            else{
                edge_points.push_back(point);
            }

            /*
            mapping_igvc::Point2D point = laserToPoint(laser_msg.ranges[i], laser_msg.angle_min + i * laser_msg.angle_increment); //Convert to x-y point
            edge_points.push_back(point);

            //If out of dist_tol or end of points, analyze edge points so far to create cone, and clear edge_points
            if ((getDist(edge_points.back(), point) > dist_tol) || i == numIndices - 1) {

                std::cout << "Dist: " << getDist(edge_points.back(), point) << std::endl;

                addConesInEdgeCluster(identified_cones, edge_points, radius_exp, radius_tol, min_points_in_cone,
                                      ang_threshold, frame_id);
                edge_points.clear();
            }*/
        }
    }
    return identified_cones;
}


void ConeIdentification::addConesInEdgeCluster(std::vector<mapping_igvc::ConeObstacle> &identified_cones, std::vector<mapping_igvc::Point2D> &edge_points, double radius_exp, double radius_tol, int min_points_in_cone, double ang_threshold, std::string frame_id){

    if (edge_points.size() >= min_points_in_cone * 2 + 1) { // Needs minimum number of points to split a potential multi-cone cluster

        std::vector<std::vector<mapping_igvc::Point2D>> split_edges = splitEdge(edge_points, min_points_in_cone, ang_threshold); //Split edge points if multiple cones detected (Temp values right now as params)

        for (int i = 0; i < split_edges.size(); i++){

            if (split_edges[i].size() >= min_points_in_cone) {
                mapping_igvc::ConeObstacle potential_cone = edgeToCone(split_edges[i]);

                if (fabs(potential_cone.radius - radius_exp) <= radius_tol) { //Within expected radius, valid cone
                    potential_cone.radius = radius_exp;
                    potential_cone.header.frame_id = frame_id;
                    potential_cone.header.stamp = ros::Time::now();
                    identified_cones.push_back(potential_cone);
                }
                /*
                potential_cone.header.frame_id = frame_id;
                potential_cone.header.stamp = ros::Time::now();
                if (std::isfinite(potential_cone.radius))
                    identified_cones.push_back(potential_cone);*/
            }
        }
    }
    else if (edge_points.size() >= min_points_in_cone) { //Directly make the cone if not enough points to split (cone still needs at least 3 edge points though)
        mapping_igvc::ConeObstacle potential_cone = edgeToCone(edge_points);

        if (fabs(potential_cone.radius - radius_exp) <= radius_tol){ //Within expected radius, valid cone
            potential_cone.radius = radius_exp;
            potential_cone.header.frame_id = frame_id;
            potential_cone.header.stamp = ros::Time::now();
            identified_cones.push_back(potential_cone);
        }
        /*
        potential_cone.header.frame_id = frame_id;
        potential_cone.header.stamp = ros::Time::now();
        if (std::isfinite(potential_cone.radius))
            identified_cones.push_back(potential_cone);*/
    }
}


mapping_igvc::Point2D ConeIdentification::laserToPoint(double dist, double ang){
    mapping_igvc::Point2D point = mapping_igvc::Point2D();
    point.x = dist * cos(ang);
    point.y = dist * sin(ang);
    return point;
}


double ConeIdentification::getDist(const mapping_igvc::Point2D &p1, const mapping_igvc::Point2D &p2){
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}


std::vector<std::vector<mapping_igvc::Point2D>> ConeIdentification::splitEdge(const std::vector<mapping_igvc::Point2D> &edge_points, int min_points_in_cone, double ang_threshold){
    std::vector<std::vector<mapping_igvc::Point2D>> split_edges;

    std::vector<double> angles; //index 0 corresponds to index min_points_in_cone of edge_points vector
    for (int i = min_points_in_cone; i < edge_points.size() - min_points_in_cone; i++){

        /*
        std::vector<mapping_igvc::Point2D> pointGroup1(edge_points.begin() + i - min_points_in_cone, edge_points.begin() + i + 1);
        std::vector<mapping_igvc::Point2D> pointGroup2(edge_points.begin() + i, edge_points.begin() + i + min_points_in_cone + 1);
        double slope1 = getRegressionSlope(pointGroup1);
        double slope2 = getRegressionSlope(pointGroup2);
        double ang = atan((slope1 - slope2) / (1 + slope1 * slope2));
        ang = M_PI - ang;
        //if (ang < 0) ang += 2 * M_PI;
        std::cout<<"Slope1: "<<slope1<<std::endl;
        std::cout<<"Slope2: "<<slope2<<std::endl;
        std::cout<<"Angle: "<<ang<<std::endl;*/

        //Calculate the angle between the two lines we form
        double AB_x = edge_points[i].x - edge_points[i - min_points_in_cone].x;
        double AB_y = edge_points[i].y - edge_points[i - min_points_in_cone].y;
        double CB_x = edge_points[i].x - edge_points[i + min_points_in_cone].x;
        double CB_y = edge_points[i].y - edge_points[i + min_points_in_cone].y;
        double dot = AB_x * CB_x + AB_y * CB_y;
        double cross = AB_x * CB_y - AB_y * CB_x;
        double ang = -atan2(cross, dot);
        if (ang < 0) ang += 2 * M_PI;

        angles.push_back(ang);
    }

    //Find local mins, then check if they are below threshold
    std::vector<size_t> splitIndices;
    for (int i = 1; i < angles.size() - 1; i++){
        bool isLocalMin = true;
        for (int j = i - min_points_in_cone; j <= i + min_points_in_cone; j++){
            if (angles[j] < angles[i]) {
                isLocalMin = false;
                break;
            }
        }
        if (angles[i] < ang_threshold && isLocalMin)
            splitIndices.push_back(i + min_points_in_cone);
    }

    //Split edges based on qualified local mins
    size_t lastIndex = 0;
    for (int i = 0; i < splitIndices.size(); i++) {
        std::vector<mapping_igvc::Point2D> split(edge_points.begin() + lastIndex,
                                                 edge_points.begin() + splitIndices[i] + 1);
        split_edges.push_back(split);
        lastIndex = splitIndices[i];
    }
    std::vector<mapping_igvc::Point2D> lastSplit(edge_points.begin() + lastIndex, edge_points.end());// Add last cluster of edges
    split_edges.push_back(lastSplit);

    return split_edges;
}


mapping_igvc::ConeObstacle ConeIdentification::edgeToCone(const std::vector<mapping_igvc::Point2D> &edge_points){
    mapping_igvc::ConeObstacle cone = mapping_igvc::ConeObstacle();

    int i, iter, IterMAX = 99;

    double Xi, Yi, Zi;
    double Mz, Mxy, Mxx, Myy, Mxz, Myz, Mzz, Cov_xy, Var_z;
    double A0, A1, A2, A22;
    double Dy, xnew, x, ynew, y;
    double DET, Xcenter, Ycenter;

    // Compute x- and y- sample means (via a function in the class "data")
    double meanX = getMeanX(edge_points);
    double meanY = getMeanY(edge_points);

    // computing moments
    Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0.;
    for (i=0; i<edge_points.size(); i++) {
        Xi = edge_points[i].x - meanX;   //  centered x-coordinates
        Yi = edge_points[i].y - meanY;   //  centered y-coordinates
        Zi = Xi*Xi + Yi*Yi;

        Mxy += Xi*Yi;
        Mxx += Xi*Xi;
        Myy += Yi*Yi;
        Mxz += Xi*Zi;
        Myz += Yi*Zi;
        Mzz += Zi*Zi;
    }
    Mxx /= edge_points.size();
    Myy /= edge_points.size();
    Mxy /= edge_points.size();
    Mxz /= edge_points.size();
    Myz /= edge_points.size();
    Mzz /= edge_points.size();

    // computing the coefficients of the characteristic polynomial
    Mz = Mxx + Myy;
    Cov_xy = Mxx*Myy - Mxy*Mxy;
    Var_z = Mzz - Mz*Mz;

    A2 = 4*Cov_xy - 3*Mz*Mz - Mzz;
    A1 = Var_z*Mz + 4*Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
    A0 = Mxz*(Mxz*Myy - Myz*Mxy) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
    A22 = A2 + A2;

    // find the root of the characteristic polynomial using Newton's method starting at x=0
    // (it is guaranteed to converge to the right root)
    for (x=0.,y=A0,iter=0; iter<IterMAX; iter++)  // usually, 4-6 iterations are enough
    {
        Dy = A1 + x*(A22 + 16.*x*x);
        xnew = x - y/Dy;
        if ((xnew == x)||(!std::isfinite(xnew))) break;
        ynew = A0 + xnew*(A1 + xnew*(A2 + 4*xnew*xnew));
        if (abs(ynew)>=abs(y))  break;
        x = xnew;  y = ynew;
    }

    // compute parameters of the fitting circle
    DET = x*x - x*Mz + Cov_xy;
    Xcenter = (Mxz*(Myy - x) - Myz*Mxy)/DET/2;
    Ycenter = (Myz*(Mxx - x) - Mxz*Mxy)/DET/2;

    // assemble the output
    cone.center.x = Xcenter + meanX; // estimated x position
    cone.center.y = Ycenter + meanY; //estimated y position
    cone.radius = sqrt(Xcenter*Xcenter + Ycenter*Ycenter + Mz - x - x); //estimated radius

    /*
    std::cout<<"X: "<<cone.center.x<<std::endl;
    std::cout<<"Y: "<<cone.center.y<<std::endl;
    std::cout<<"R: "<<cone.radius<<std::endl;
    std::cout<<std::endl;*/

    return cone;
}


double ConeIdentification::getMeanX(const std::vector<mapping_igvc::Point2D> &edge_points){
    double tot_x = 0;
    for (int i=0; i<edge_points.size(); i++){
        tot_x += edge_points[i].x;
    }
    return tot_x / edge_points.size();
}


double ConeIdentification::getMeanY(const std::vector<mapping_igvc::Point2D> &edge_points){
    double tot_y = 0;
    for (int i=0; i<edge_points.size(); i++){
        tot_y += edge_points[i].y;
    }
    return tot_y / edge_points.size();
}


double ConeIdentification::getRegressionSlope(const std::vector<mapping_igvc::Point2D> &edge_points){
    double meanX = getMeanX(edge_points);
    double meanY = getMeanY(edge_points);
    double sumNum = 0;
    double sumDenom = 0;

    for (int i=0; i<edge_points.size(); i++){
        sumNum += (edge_points[i].x - meanX) * (edge_points[i].y - meanY);
        sumDenom += pow(edge_points[i].x - meanX, 2);
    }
    return sumNum / sumDenom;
}