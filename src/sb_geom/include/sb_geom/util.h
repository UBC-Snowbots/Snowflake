// TODO: Start of file comment

#ifndef SB_GEOM_UTIL_H
#define SB_GEOM_UTIL_H

#include "sb_geom/Point2D.h"
#include "sb_geom/PolynomialSegment.h"

namespace sb_geom {
    // TODO: Test me
    // TODO: Do we even need this function?
    /**
     * Computes the shortest distance between a given point and polynomial line segment
     *
     * Uses Halley Iteration
     * @param point TODO
     * @param line TODO
     * @param max_iter the maximum number of iterations to perform, more iterations
     * will lead to more accurate results, but increases computation time
     * @return the minimum distance between the point and polynomial line
     */
    double minDistanceFromPointToPolynomialSegment(
            Point2D point, PolynomialSegment line, uintmax_t max_iter = 20);
}

#endif //SB_GEOM_UTIL_H
