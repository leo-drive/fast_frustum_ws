#ifndef SRC_JSKHELPER_H
#define SRC_JSKHELPER_H

#include <jsk_recognition_msgs/PolygonArray.h>
#include "PointCloudTypes.h"

class JskHelper {
public:
  static jsk_recognition_msgs::PolygonArray::Ptr CornersAndSourceToPolygons(
    const pcltype::Point &p_top_left,
    const pcltype::Point &p_top_right,
    const pcltype::Point &p_bot_left,
    const pcltype::Point &p_bot_right,
    const pcltype::Point &p_source,
    const std_msgs::Header &header);

  static bool AddCornersToPolygons(
    jsk_recognition_msgs::PolygonArray::Ptr &polygons,
    const pcltype::Point &p_top_left,
    const pcltype::Point &p_top_right,
    const pcltype::Point &p_bot_left,
    const pcltype::Point &p_bot_right,
    const pcltype::Point &p_source);

  static geometry_msgs::Point32 PclPointToGeometryPoint(const pcltype::Point &p_in);
private:

};


#endif //SRC_JSKHELPER_H
