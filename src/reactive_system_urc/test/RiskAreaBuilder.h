//
// Created by william on 19/01/19.
//

#ifndef PROJECT_RISKAREABUILDER_H
#define PROJECT_RISKAREABUILDER_H

#include <sb_geom_msgs/Point2D.h>
#include <mapping_msgs_urc/RiskAreaArray.h>

namespace RiskAreaBuilder {
    class RiskAreaBuilder {
    public:
        /**
         * Initially constructs a large rectangular area (corner to corner) of risk areas with 0 risk scores assigned
         * @param corner1: should be upper left corner (so corner1.x < corner2.x, and corner1.y < corner2.y)
         * @param corner2: should be bottom right corner
         * @param risk_length, the length for a single risk area
         */
        RiskAreaBuilder(sb_geom_msgs::Point2D corner1, sb_geom_msgs::Point2D corner2, float risk_length){

            for (float i = corner1.x; i <= corner2.x; i += risk_length){
                for (float j = corner1.y; j <= corner2.x; j += risk_length){
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
                    area.score.data = 0; //lowest possible risk
                    risk_areas.areas.push_back(area);
                }
            }

        }

        /**
         * Adds a risk zone to the current risk area map
         * @param risk_center: center of risk
         * @param radius: how close risk areas must be to center to get assigned risk
         * @param score: score to assign each risk area near risk center
         */
        void addRiskZone(sb_geom_msgs::Point2D risk_center, float radius, float score){
            for (mapping_msgs_urc::RiskArea area: risk_areas.areas){
                if (isWithinRadius(area, risk_center, radius)){
                    area.score.data = score;
                }
            }
        }



        mapping_msgs_urc::RiskAreaArray getRiskArray(){
            return risk_areas;
        }

    private:
        mapping_msgs_urc::RiskAreaArray risk_areas;

        bool isWithinRadius(mapping_msgs_urc::RiskArea area, sb_geom_msgs::Point2D risk_center, float radius){
            for (sb_geom_msgs::Point2D point : area.area.points){
                if (pow(point.x-risk_center.x,2) + pow(point.y-risk_center.y,2) < pow(radius,2)){
                    return true;
                }
            }
            return false;
        }
    };
}



#endif //PROJECT_RISKAREABUILDER_H
