#ifndef SRC_POINTCLOUDTYPES_H
#define SRC_POINTCLOUDTYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcltype{
  typedef pcl::PointXYZI Point;
  typedef pcl::PointCloud<Point> Cloud;

  typedef pcl::PointXYZRGB PointColored;
  typedef pcl::PointCloud<PointColored> CloudColored;
}

#endif //SRC_POINTCLOUDTYPES_H
