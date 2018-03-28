#ifndef SB_GEOM_TEST_UTILS_H
#define SB_GEOM_TEST_UTILS_H

#include <sb_geom/Spline.h>

class TestUtils {
public:
    /**
     * Prints out a given spline by sampling a given number of points
     *
     * @param spline the spline to print
     * @param num_points the number of points to sample from the spline
     */
    static void printSpline(sb_geom::Spline spline, unsigned int num_points) {
        for (int i = 0; i < num_points; i++){
            sb_geom::Point2D p = spline((double)i/(double)num_points);
            std::cout << i << ", " << p.x() << ", " << p.y() << std::endl;
        }
    }
};

#endif //SB_GEOM_TEST_UTILS_H
