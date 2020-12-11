#include "PclStuff.h"
#include "pcl/common/centroid.h"

using Cloud = pcltype::Cloud;
using Point = pcltype::Point;

std::vector<Cloud::Ptr>
PclStuff::Clusterer(Cloud::Ptr &centroids,
                    Cloud::ConstPtr cloud_in,
                    float tolerance,
                    int min_point_count,
                    int max_point_count) {
  pcl::search::KdTree<Point>::Ptr kdtree(
    new pcl::search::KdTree<Point>);
  kdtree->setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> vec_cluster_indices;

  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(min_point_count);
  ec.setMaxClusterSize(max_point_count);
  ec.setSearchMethod(kdtree);
  ec.setInputCloud(cloud_in);
  ec.extract(vec_cluster_indices);

  std::vector<Cloud::Ptr> clusters;

  centroids.reset(new Cloud);

  int counter_random = 0;

  for (const auto &cluster_indices : vec_cluster_indices) {
    Cloud::Ptr cluster(new Cloud);
    for (const auto &index : cluster_indices.indices) {
      Point p = cloud_in->points[index];
      p.intensity = counter_random * 0.05;
      cluster->points.push_back(p);
    }
    clusters.push_back(cluster);
    counter_random++;
    Eigen::Vector4f centroid_eigen;
    pcl::compute3DCentroid(*cluster, centroid_eigen);
    Point centroid;
    centroid.x = centroid_eigen(0);
    centroid.y = centroid_eigen(1);
    centroid.z = centroid_eigen(2);
    centroids->push_back(centroid);
  }

  return clusters;
}

