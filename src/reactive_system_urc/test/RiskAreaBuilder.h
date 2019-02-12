/*
 * Created By: William Gu
 * Created On: Jan 19 2019
 * Description: Helper class for unit testing, to create fake risk area ROS messages
 */

#ifndef PROJECT_RISKAREABUILDER_H
#define PROJECT_RISKAREABUILDER_H

#include <mapping_msgs_urc/RiskAreaArray.h>
#include <sb_geom_msgs/Point2D.h>

namespace RiskAreaBuilder {
class RiskAreaBuilder {
  public:
    /**
     * Initially constructs a large rectangular area (corner to corner) of risk
     * areas with 0 risk scores assigned
     * @param corner1: should be upper left corner (so corner1.x < corner2.x,
     * and corner1.y < corner2.y)
     * @param corner2: should be bottom right corner
     * @param risk_length, the length for a single risk area
     */
    RiskAreaBuilder(sb_geom_msgs::Point2D corner1,
                    sb_geom_msgs::Point2D corner2,
                    float risk_length) {
        mapping_msgs_urc::RiskAreaArray risk_areas;

        for (float i = corner1.x; i <= corner2.x; i += risk_length) {
            for (float j = corner1.y; j <= corner2.x; j += risk_length) {
                mapping_msgs_urc::RiskArea area;
                sb_geom_msgs::Point2D p1;
                sb_geom_msgs::Point2D p2;
                sb_geom_msgs::Point2D p3;
                sb_geom_msgs::Point2D p4;
                p1.x = i;
                p1.y = j;
                p2.x = i;
                p2.y = j + risk_length;
                p3.x = i + risk_length;
                p3.y = j;
                p4.x = i + risk_length;
                p4.y = j + risk_length;
                area.area.points.push_back(p1);
                area.area.points.push_back(p2);
                area.area.points.push_back(p3);
                area.area.points.push_back(p4);
                area.score.data = 0; // lowest possible risk
                risk_areas.areas.push_back(area);
            }
        }
        risk_areas_ = risk_areas;
    }

    /**
     * Adds a risk zone to the current risk area map
     * @param risk_center: center of risk
     * @param radius: how close risk areas must be to center to get assigned
     * risk
     * @param score: score to assign each risk area near risk center
     */
    void
    addRiskZone(sb_geom_msgs::Point2D risk_center, float radius, float score) {
        for (int i = 0; i < risk_areas_.areas.size(); i++) {
            if (isWithinRadius(risk_areas_.areas[i], risk_center, radius)) {
                risk_areas_.areas[i].score.data = score;
            }
        }
    }

    mapping_msgs_urc::RiskAreaArray getRiskArray() { return risk_areas_; }

  private:
    mapping_msgs_urc::RiskAreaArray risk_areas_;

    bool isWithinRadius(mapping_msgs_urc::RiskArea area,
                        sb_geom_msgs::Point2D risk_center,
                        float radius) {
        for (sb_geom_msgs::Point2D point : area.area.points) {
            if (pow(point.x - risk_center.x, 2) +
                pow(point.y - risk_center.y, 2) <
                pow(radius, 2)) {
                return true;
            }
        }
        return false;
    }
};
}

#endif // PROJECT_RISKAREABUILDER_H
