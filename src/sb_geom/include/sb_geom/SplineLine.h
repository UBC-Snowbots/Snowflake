/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: TODO
 */

#ifndef SB_GEOM_SPLINELINE_H
#define SB_GEOM_SPLINELINE_H

// ROS Includes
#include <ecl/geometry.hpp>

// Snowbots Includes
#include "sb_geom/Line.h"
#include "sb_geom/PolyLine.h"

namespace sb_geom {

    // A line, represented as a n-ary spline
    class SplineLine : public Line {
    public:
        /**
         * Create a SplineLine from the given polynomial line
         * @param poly_line
         */
        SplineLine(PolyLine poly_line);
    private:
        ecl::CubicSpline
    };

}


#endif //SB_GEOM_SPLINELINE_H
