#ifndef SRC_FRUSTUMPROJECTOR_H
#define SRC_FRUSTUMPROJECTOR_H

#include <vision_msgs/Detection2DArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <memory>
#include "PointCloudTypes.h"
#include "QueueCloud.h"

#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>
#include <mutex>
#include <future>
#include <queue>
#include <deque>

#include<visualization_msgs/MarkerArray.h>
#include "QueueCloud.h"

class FrustumProjector {
public:
  explicit FrustumProjector(ros::NodeHandle &nh);

private:
  ros::NodeHandle &nh_;

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Subscriber sub_lidar_stitched_;

  image_transport::ImageTransport it_;

  int sub_queue_size_;
  int pub_queue_size_;

  std::string frame_lidar_;

  ros::Publisher pub_health_checker_;
  ros::Timer timer_;

  void SoftwareHealthCheckerCallback(int publisher) {
    std::cout << "Coming callback :" << publisher <<std::endl;
  }


  struct ImageDetectionsSet {

    ros::Subscriber SubDetection;

    ros::Publisher pub_polygons_detections_;
    ros::Publisher pub_center_marker_array_;
    ros::Publisher pub_cloud_frustum_;
    ros::Publisher pub_frustum_cloud_centroid_;
    ros::Publisher pub_all_frustum_all_points_;
    ros::Publisher pub_center_points_;
    ros::Publisher pub_object_pose_and_label_;

    std::string frame_camera_;

    int Id;
  };

  std::vector<ImageDetectionsSet> image_detections_sets_;

  QueueCloud::Ptr queue_cloud_;

  size_t queue_cloud_max_elemet_;

  std::vector<cv::Mat> camera_matrices_;



  void CallbackCloudStitched(const sensor_msgs::PointCloud2::ConstPtr &msg_cloud_in);


  void CallbackDetections(const vision_msgs::Detection2DArray::ConstPtr &msg_detections_in,
                          int id);


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif //SRC_FRUSTUMPROJECTOR_H
