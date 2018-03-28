/*
 * Created By: Min Gyo Kim
 * Created On: February 17, 2018
 * Description: Helper functions for testing Regression, DBSCAN, and rostest
 */

#ifndef LINE_EXTRACTOR_IGVC_TESTUTILS_H
#define LINE_EXTRACTOR_IGVC_TESTUTILS_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace LineExtractor {
class TestUtils {
  public:
    /**
     * LineArgs is a struct representing a mathematical line.
     * @param coefficients coefficients of the mathematical line, where the index
     * of a coefficient corresponds to its degree
     * @param x_min, x_max domain of the line
     * @param x_delta the distance between adjacent points on the line
     */
    struct LineArgs {
        std::vector<float> coefficients;
        float x_min;
        float x_max;
        float x_delta;
        LineArgs(std::vector<float> coefficients,
                 float x_min,
                 float x_max,
                 float x_delta)
          : coefficients(coefficients),
            x_min(x_min),
            x_max(x_max),
            x_delta(x_delta){};
    };

    /**
     * Adds given line to the given point cloud
     * @param args struct representing a mathematical line within a domain
     * @param pcl point cloud to add the line to
     */
    static void addLineToPointCloud(LineArgs args,
                                    pcl::PointCloud<pcl::PointXYZ>& pcl) {
        for (float x = args.x_min; x <= args.x_max; x += args.x_delta) {
            pcl::PointXYZ p;
            p.x = x;
            p.y = 0;

            for (unsigned int i = 0; i < args.coefficients.size(); i++) {
                p.y += args.coefficients[i] * pow(x, i);
            }

            pcl.push_back(p);
        }
    };

    /**
     * Adds given line with added noise to the given point cloud
     * @param args struct representing a mathematical line within a domain
     * @param pcl point cloud to add the line to
     * @param max_noise_x maximum deviation of a point from the line in x direction
     * @param max_noise_y maximum deviation of a point from the line in y direction
     * @param seed seed for rand()
     */
    static void addLineToPointCloud(LineArgs args,
                                    pcl::PointCloud<pcl::PointXYZ>& pcl,
                                    float max_noise_x,
                                    float max_noise_y,
                                    unsigned int seed = 123) {
        srand(seed);

        for (float x = args.x_min; x <= args.x_max; x += args.x_delta) {
            float true_x = x;
            float true_y = 0;

            for (unsigned int i = 0; i < args.coefficients.size(); i++) {
                true_y += args.coefficients[i] * pow(x, i);
            }

            float noise_y =
            ((float) rand() / (RAND_MAX)) * max_noise_y * 2 - max_noise_y;
            float noise_x =
            ((float) rand() / (RAND_MAX)) * max_noise_x * 2 - max_noise_x;

            float deformed_x = true_x + noise_x;
            float deformed_y = true_y + noise_y;

            pcl::PointXYZ p(deformed_x, deformed_y, 0);
            pcl.push_back(p);
        }
    };

    /**
     * Gets the number of points that form the line based on its domain and distance between adjacent points
     * @param args representation of the line in a struct
     * @return the number of points that form the given line
     */
    unsigned int static getNumPoints(LineArgs args) {
        return (args.x_max - args.x_min) / args.x_delta + 1;
    }

    /**
     * Gets the minimum and maximum values of x of the point cloud
     * @param min_x variable to store the minimum value
     * @param max_x variable to store the maximum value
     * @param pcl point cloud
     */
    static void getMinAndMaxOfPointCloud(float& min_x,
                                         float& max_x,
                                         pcl::PointCloud<pcl::PointXYZ> pcl) {
        double min, max;

        if (pcl.size()) {
            min = max = pcl[0].x;
        } else {
            min_x = max_x = -1;
            return;
        }

        for (unsigned int i = 0; i < pcl.size(); i++) {
            if (pcl[i].x < min) { min = pcl[i].x; }
            if (pcl[i].x > max) { max = pcl[i].x; }
        }

        min_x = min;
        max_x = max;
    }
};
}

#endif // LINE_EXTRACTOR_TESTUTILS_H
