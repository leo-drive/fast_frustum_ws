#ifndef SRC_PCLSTUFF_H
#define SRC_PCLSTUFF_H

#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "PointCloudTypes.h"

class PclStuff {
public:
  using Cloud = pcltype::Cloud;
  using Point = pcltype::Point;

  static std::vector<Cloud::Ptr> Clusterer(Cloud::Ptr &centroids,
                                           Cloud::ConstPtr cloud_in,
                                           float tolerance,
                                           int min_point_count,
                                           int max_point_count,
                                           float x_max = 999999,
                                           float y_max = 999999,
                                           float z_max = 999999);


};


#endif //SRC_PCLSTUFF_H
