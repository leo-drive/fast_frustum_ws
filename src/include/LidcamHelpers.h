#ifndef SRC_LIDCAMHELPERS_H
#define SRC_LIDCAMHELPERS_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "PointCloudTypes.h"

#include <vision_msgs/Detection2DArray.h>

class LidcamHelpers {
public:
///
/// \param point_colored
/// \param point_in_image
/// \param image
/// \param point
/// \param mat_transformer 4x4
/// \return
  static bool PointToColoredPoint(pcltype::PointColored &point_colored,
                                  cv::Point &point_in_image,
                                  pcltype::Point &point2,
                                  const cv::Mat &image,
                                  const pcltype::Point &point,
                                  const Eigen::Matrix4d &mat_transformer);

  static pcltype::Point PointImageToPoint3D(const cv::Point &point_in_image,
                                            float dist,
                                            const Eigen::Matrix4d &mat_point_transformer_inverse,
                                            const Eigen::Matrix4d &mat_velo_to_cam);

  static std::tuple<int, int, int> ColorPicker(double distance);


  // Göktuğ:
  static void fillFrustumCloud(pcltype::Cloud::Ptr cloud_in, Eigen::Matrix4d mat_point_transformer,cv::Size img_size,
                                 std::vector<std::vector<pcltype::Cloud::Ptr>>& thread_vector_cloud_frustums, unsigned int thread_id,
                                 const vision_msgs::Detection2DArray& interested_detections, int camera_id,
                                 pcltype::Cloud::iterator it_begin, pcltype::Cloud::iterator it_end);

  static std::pair<bool, cv::Point> pointInImagePlane(pcltype::Point point, Eigen::Matrix4d mat_point_transformer, cv::Size img_size);
  static bool pointInDetection(const cv::Point &point, const vision_msgs::BoundingBox2D &bbox,const cv::Size &img_size, const int camera_id);



};


#endif //SRC_LIDCAMHELPERS_H
