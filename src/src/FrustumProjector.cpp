#include "FrustumProjector.h"
#include "LidcamHelpers.h"
#include "HelperRosRelated.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <X11/Intrinsic.h>
#include <opencv2/core/eigen.hpp>
#include "PclStuff.h"
#include "JskHelper.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"

#include <std_msgs/Int8.h>
#include "SoftwareHealthChecker.h"

#include <thrust/host_vector.h>

using pcltype::Point;
using pcltype::Cloud;
using pcltype::PointColored;
using pcltype::CloudColored;


FrustumProjector::FrustumProjector(ros::NodeHandle &nh) :
  nh_(nh),
  it_(nh) {

  int count_projections = 10;

  std::vector<std::string> vector_topics_sub_detections{
    "/classification_2d_node/cam_fm_01/detection_2d_array",
    "/classification_2d_node/cam_fr_01/detection_2d_array",
    "/detection_2d_node/cam_fr_02/detection_2d_array",
    "/detection_2d_node/cam_fr_03/detection_2d_array",
    "/classification_2d_node/cam_fl_01/detection_2d_array",
    "/detection_2d_node/cam_fl_02/detection_2d_array",
    "/detection_2d_node/cam_fl_03/detection_2d_array",
    "/detection_2d_node/cam_bm_01/detection_2d_array",
    "/thermal_2d_detection_node/cam_thermal_left/detection_2d_array",
    "/thermal_2d_detection_node/cam_thermal_right/detection_2d_array",
  };

  assert(vector_topics_sub_detections.size() == count_projections);

  std::vector<std::string> vector_topics_pub_polygons_detections{
    "/polygons_detections/cam_fm_01",
    "/polygons_detections/cam_fr_01",
    "/polygons_detections/cam_fr_02",
    "/polygons_detections/cam_fr_03",
    "/polygons_detections/cam_fl_01",
    "/polygons_detections/cam_fl_02",
    "/polygons_detections/cam_fl_03",
    "/polygons_detections/cam_bm_01",
    "/polygons_detections/cam_flir_left",
    "/polygons_detections/cam_flir_right",

  };

  assert(vector_topics_pub_polygons_detections.size() == count_projections);

  std::vector<std::string> vector_frame_cameras{
    "cam_fm_01",
    "cam_fr_01",
    "cam_fr_02",
    "cam_fr_03",
    "cam_fl_01",
    "cam_fl_02",
    "cam_fl_03",
    "cam_bm_01",
    "cam_flir_left",
    "cam_flir_right",
  };

  assert(vector_frame_cameras.size() == count_projections);

  std::vector<std::string> topic_cloud_frustum_all_centroid{
    "/topic_cloud_frustum_all_centroid/cam_fm_01",
    "/topic_cloud_frustum_all_centroid/cam_fr_01",
    "/topic_cloud_frustum_all_centroid/cam_fr_02",
    "/topic_cloud_frustum_all_centroid/cam_fr_03",
    "/topic_cloud_frustum_all_centroid/cam_fl_01",
    "/topic_cloud_frustum_all_centroid/cam_fl_02",
    "/topic_cloud_frustum_all_centroid/cam_fl_03",
    "/topic_cloud_frustum_all_centroid/cam_bm_01",
    "/topic_cloud_frustum_all_centroid/cam_flir_left",
    "/topic_cloud_frustum_all_centroid/cam_flir_right",
  };

  assert(topic_cloud_frustum_all_centroid.size() == count_projections);

  std::vector<std::string> topic_all_frustum_all_points{
    "/all_frustum_all_points/cam_fm_01",
    "/all_frustum_all_points/cam_fr_01",
    "/all_frustum_all_points/cam_fr_02",
    "/all_frustum_all_points/cam_fr_03",
    "/all_frustum_all_points/cam_fl_01",
    "/all_frustum_all_points/cam_fl_02",
    "/all_frustum_all_points/cam_fl_03",
    "/all_frustum_all_points/cam_bm_01",
    "/all_frustum_all_points/cam_flir_left",
    "/all_frustum_all_points/cam_flir_right",
  };

  assert(topic_all_frustum_all_points.size() == count_projections);

  std::vector<std::string> topics_frustum_closest_object_center_points{
    "/frustum_closest_object_center_points/cam_fm_01",
    "/frustum_closest_object_center_points/cam_fr_01",
    "/frustum_closest_object_center_points/cam_fr_02",
    "/frustum_closest_object_center_points/cam_fr_03",
    "/frustum_closest_object_center_points/cam_fl_01",
    "/frustum_closest_object_center_points/cam_fl_02",
    "/frustum_closest_object_center_points/cam_fl_03",
    "/frustum_closest_object_center_points/cam_bm_01",
    "/frustum_closest_object_center_points/cam_flir_left",
    "/frustum_closest_object_center_points/cam_flir_right",
  };

  assert(topics_frustum_closest_object_center_points.size() == count_projections);

  std::vector<std::string> topics_publish_object_pose_and_label{
    "/detected_object3D/cam_fm_01",
    "/detected_object3D/cam_fr_01",
    "/detected_object3D/cam_fr_02",
    "/detected_object3D/cam_fr_03",
    "/detected_object3D/cam_fl_01",
    "/detected_object3D/cam_fl_02",
    "/detected_object3D/cam_fl_03",
    "/detected_object3D/cam_bm_01",
    "/detected_object3D/cam_flir_left",
    "/detected_object3D/cam_flir_right",
  };

  assert(topics_publish_object_pose_and_label.size() == count_projections);

  camera_matrices_ = {
    //CAMERA FM_01
    (cv::Mat1d(3, 3)
      << 1779.138856, 0.000000, 947.973100,
      0.000000, 1783.247671, 579.780687,
      0.000000, 0.000000, 1.000000),
    //CAMERA FR_01
    (cv::Mat1d(3, 3)
      << 1769.271357, 0.000000, 953.305761,
      0.000000, 1769.554343, 610.233132,
      0.000000, 0.000000, 1.000000),
    //CAMERA FR_02
    (cv::Mat1d(3, 3)
      << 1799.178453, 0.000000, 995.357045,
      0.000000, 1795.555471, 609.529343,
      0.000000, 0.000000, 1.000000),
    //CAMERA FR_03
    (cv::Mat1d(3, 3)
      << 1789.703587, 0.000000, 990.802969,
      0.000000, 1790.396500, 585.589159,
      0.000000, 0.000000, 1.000000),
    //CAMERA FL_01
    (cv::Mat1d(3, 3)
      << 1793.871478, 0.000000, 975.114861,
      0.000000, 1799.296938, 559.274364,
      0.000000, 0.000000, 1.000000),
    //CAMERA FL_02
    (cv::Mat1d(3, 3)
      << 1819.094091, 0.000000, 978.244817,
      0.000000, 1822.157631, 650.701181,
      0.000000, 0.000000, 1.000000),
    //CAMERA FL_03
    (cv::Mat1d(3, 3)
      << 1778.059851, 0.000000, 993.390744,
      0.000000, 1777.259419, 636.253364,
      0.000000, 0.000000, 1.000000),
    //CAMERA BM_01
    (cv::Mat1d(3, 3)
      << 498.706520, 0.000000, 956.500203,
      0.000000, 499.068444, 558.439097,
      0.000000, 0.000000, 1.000000),
    //CAMERA FLIR LEFT
    (cv::Mat1d(3, 3)
      << 517.0124, -0.0621, 317.9847,
      0.0, 520.5896, 220.8468,
      0.0, 0.0, 1.0000),
    //CAMERA FLIR RIGHT
    (cv::Mat1d(3, 3)
      << 517.0124, -0.0621, 317.9847,
      0.0, 520.5896, 220.8468,
      0.0, 0.0, 1.0000),

  };

  assert(camera_matrices_.size() == count_projections);

  pub_health_checker_ = nh_.advertise<std_msgs::Int8>(
    "/frustum_projector/health_message",
    1);

  timer_ =  nh_.createTimer(
    ros::Duration(0.2),
    boost::bind(
      (void(*)(ros::Publisher&))&SoftwareHealthChecker::SoftwareHealthCheckerCallback,
      pub_health_checker_
    ),
    false,
    true
  );


  sub_queue_size_ = 3;
  pub_queue_size_ = 1;
  frame_lidar_ = "vls128_01_forward";

  queue_cloud_max_elemet_ = 5;

//  queue_cloud_ = QueueCloud(queue_cloud_max_elemet_);
  queue_cloud_ = std::make_shared<QueueCloud>(queue_cloud_max_elemet_);

//  sub_lidar_stitched_ = nh_.subscribe("/cloud_stitched",
  sub_lidar_stitched_ = nh_.subscribe("/segmenter/points_nonground",
                                      sub_queue_size_,
                                      &FrustumProjector::CallbackCloudStitched,
                                      this);

  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

  image_detections_sets_.resize(count_projections);

  for (size_t i = 0; i < count_projections; ++i) {
    auto &image_detections_set = image_detections_sets_[i];
    image_detections_set.Id = i;
    image_detections_set.frame_camera_ = vector_frame_cameras[i];

    image_detections_set.pub_polygons_detections_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>(
      vector_topics_pub_polygons_detections[i],
      pub_queue_size_);

    image_detections_set.pub_frustum_cloud_centroid_ = nh_.advertise<sensor_msgs::PointCloud2>(
      topic_cloud_frustum_all_centroid[i],
      pub_queue_size_);

    image_detections_set.pub_all_frustum_all_points_ = nh_.advertise<sensor_msgs::PointCloud2>(
      topic_all_frustum_all_points[i],
      pub_queue_size_);


    image_detections_set.pub_center_points_ = nh_.advertise<sensor_msgs::PointCloud2>(
      topics_frustum_closest_object_center_points[i],
      pub_queue_size_);

    image_detections_set.pub_object_pose_and_label_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(
      topics_publish_object_pose_and_label[i],
      pub_queue_size_);


    image_detections_set.SubDetection = nh_.subscribe<
      vision_msgs::Detection2DArray>
      (vector_topics_sub_detections[i], sub_queue_size_,
       boost::bind(&FrustumProjector::CallbackDetections,
                   this, boost::placeholders::_1, image_detections_set.Id));

  }

}


void FrustumProjector::CallbackCloudStitched(
  const sensor_msgs::PointCloud2::ConstPtr &msg_cloud_in) {

  Cloud::Ptr cloud_in(new pcltype::Cloud);
  pcl::fromROSMsg(*msg_cloud_in, *cloud_in);

  QueueCloud::CloudAndHeader cloud_and_header;
  cloud_and_header.Header = msg_cloud_in->header;
  cloud_and_header.Cloud = cloud_in;

  queue_cloud_->Push(cloud_and_header);

}

void FrustumProjector::CallbackDetections(
  const vision_msgs::Detection2DArray::ConstPtr &msg_detections_in,
  int id) {


  if (queue_cloud_->GetSize() <= 0)
    return;

  auto &image_detections_set = image_detections_sets_[id];

  vision_msgs::Detection2DArray interested_detections;

  for (const auto &detection : msg_detections_in->detections) {

//    if (detection.results[0].id < 1000)
//      continue;

    if (detection.results[0].id > 2000 && detection.results[0].score < 0.75)
      continue;

    if ((detection.results[0].id - 2000) == 18)
      continue;

//    if (detection.results[0].id < 1000 )
//      continue;

    interested_detections.detections.push_back(detection);

  }

  if (interested_detections.detections.empty())
    return;

  QueueCloud::CloudAndHeader cloud_and_header;

  int index;
  cloud_and_header = queue_cloud_->GetBestCloud(msg_detections_in->header, index);

  std::cout << "index: " << index << std::endl;
  if (index == 0) {
    return;
  }

  std_msgs::Header header_cloud_in = cloud_and_header.Header;
  Cloud::Ptr cloud_in = cloud_and_header.Cloud;

  double cloud_n_sec = cloud_and_header.Header.stamp.toNSec();
  double msg_n_sec = msg_detections_in->header.stamp.toNSec();

  double time_difference = abs(cloud_n_sec - msg_n_sec) / 1000000;

  cloud_and_header = queue_cloud_->GetBestCloud(msg_detections_in->header, index);
  cloud_in = cloud_and_header.Cloud;
  header_cloud_in = cloud_and_header.Header;

//  while (time_difference > 100) { // > 65 ms , lidar frequence
//
//    if (index == 0) {
//      return;
//    }
//    cloud_n_sec = cloud_and_header.Header.stamp.toNSec();
//    msg_n_sec = msg_detections_in->header.stamp.toNSec();
//    time_difference = abs(cloud_n_sec - msg_n_sec) / 1000000;
//
//    ros::Duration(0.002).sleep();
//  }

  geometry_msgs::TransformStamped trans_velo_to_cam;
  try {
    trans_velo_to_cam = tf_buffer_.lookupTransform(image_detections_set.frame_camera_, frame_lidar_, msg_detections_in->header.stamp);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  Eigen::Affine3d affine_velo_to_cam_ = tf2::transformToEigen(trans_velo_to_cam);

  cv::Mat mat_cam_cv;

  mat_cam_cv = camera_matrices_[id];

  Eigen::Matrix3d mat_cam_intrinsic_3x3;
  cv::cv2eigen(camera_matrices_[id], mat_cam_intrinsic_3x3);

  Eigen::Matrix4d mat_velo_to_cam = affine_velo_to_cam_.matrix();

  Eigen::MatrixXd mat_projector_3x4 = mat_cam_intrinsic_3x3 * mat_velo_to_cam.topLeftCorner(3, 4);

  Eigen::Matrix4d mat_point_transformer = Eigen::Matrix4d::Identity();

  mat_point_transformer.topLeftCorner(3, 4) = mat_projector_3x4;

  Eigen::Matrix4d mat_point_transformer_inverse = mat_point_transformer.inverse();

  cv::Size img_size;

  if (id == 8 || id == 9) {
    img_size = cv::Size2d(1920, 1200);
  } else {
    img_size = cv::Size2d(1920, 1200);
  }


  cv::Point p_top_left(0, 0);
  cv::Point p_top_right(img_size.width, 0);
  cv::Point p_bot_left(0, img_size.height);
  cv::Point p_bot_right(img_size.width, img_size.height);

  float dist_projection{70};

  pcltype::Point p1 = LidcamHelpers::PointImageToPoint3D(p_top_left, 1, mat_point_transformer_inverse,
                                                         affine_velo_to_cam_.matrix());
  pcltype::Point p2 = LidcamHelpers::PointImageToPoint3D(p_top_right, 1, mat_point_transformer_inverse,
                                                         affine_velo_to_cam_.matrix());
  pcltype::Point p3 = LidcamHelpers::PointImageToPoint3D(p_bot_left, 1, mat_point_transformer_inverse,
                                                         affine_velo_to_cam_.matrix());
  pcltype::Point p4 = LidcamHelpers::PointImageToPoint3D(p_bot_right, 1, mat_point_transformer_inverse,
                                                         affine_velo_to_cam_.matrix());

  Eigen::Vector3d translation_camera = affine_velo_to_cam_.inverse().translation();

  pcltype::Point p_camera;
  p_camera.x = translation_camera(0);
  p_camera.y = translation_camera(1);
  p_camera.z = translation_camera(2);

  header_cloud_in.frame_id = frame_lidar_;

  jsk_recognition_msgs::PolygonArray::Ptr polygons = JskHelper::CornersAndSourceToPolygons(p1, p2, p3, p4, p_camera,
                                                                                           header_cloud_in);

  size_t count_detections = interested_detections.detections.size();

  // Iterate detections for frustum calculation
  for (size_t i = 0; i < count_detections; i++) {

    std::vector<Cloud::Ptr> cloud_clusters(count_detections);

    cloud_clusters[i].reset(new Cloud);
    const auto &detection = interested_detections.detections[i];
    const auto &bbox_orig = detection.bbox;
    auto bbox = bbox_orig;
    float scale_factor;

    if (id == 8 || id == 9) {
      scale_factor = 1.0;
    } else {
      scale_factor = 1.66666;
    }


    bbox.center.x *= scale_factor;
    bbox.center.y *= scale_factor;
    bbox.size_x *= scale_factor;
    bbox.size_y *= scale_factor;
    cv::Point2f top_left(bbox.center.x - bbox.size_x, bbox.center.y - bbox.size_y);
    cv::Point2f bot_right(bbox.center.x + bbox.size_x, bbox.center.y + bbox.size_y);
    cv::Rect2f rect_detection(top_left, bot_right);

    cv::Point p_top_left(top_left);
    cv::Point p_top_right(bot_right.x, top_left.y);
    cv::Point p_bot_left(top_left.x, bot_right.y);
    cv::Point p_bot_right(bot_right);

    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << detection.results[0].score;
    std::string score = stream.str();

    pcltype::Point p_projected_far_1 = LidcamHelpers::PointImageToPoint3D(p_top_left, dist_projection,
                                                                          mat_point_transformer_inverse,
                                                                          affine_velo_to_cam_.matrix());
    pcltype::Point p_projected_far_2 = LidcamHelpers::PointImageToPoint3D(p_top_right, dist_projection,
                                                                          mat_point_transformer_inverse,
                                                                          affine_velo_to_cam_.matrix());
    pcltype::Point p_projected_far_3 = LidcamHelpers::PointImageToPoint3D(p_bot_left, dist_projection,
                                                                          mat_point_transformer_inverse,
                                                                          affine_velo_to_cam_.matrix());
    pcltype::Point p_projected_far_4 = LidcamHelpers::PointImageToPoint3D(p_bot_right, dist_projection,
                                                                          mat_point_transformer_inverse,
                                                                          affine_velo_to_cam_.matrix());

    JskHelper::AddCornersToPolygons(polygons,
                                    p_projected_far_1,
                                    p_projected_far_2,
                                    p_projected_far_3,
                                    p_projected_far_4,
                                    p_camera);
  }


  image_detections_set.pub_polygons_detections_.publish(*polygons);

  auto point_is_within_rect = [](const cv::Point &point,
                                 const vision_msgs::BoundingBox2D &bbox,
                                 const cv::Size &img_size,
                                 const int camera_id) {
    const auto &bbox_orig = bbox;
    auto new_bbox = bbox;

    float scale_factor;
    if (camera_id == 8 || camera_id == 9) {
      scale_factor = 1.0;
    } else {
      scale_factor = 1.66666;
    }


    new_bbox.center.x *= scale_factor;
    new_bbox.center.y *= scale_factor;
    new_bbox.size_x *= scale_factor;
    new_bbox.size_y *= scale_factor;

//        float x_min = new_bbox.center.x - new_bbox.size_x / 2;
//        float x_max = new_bbox.center.x + new_bbox.size_x / 2;
//        float y_min = new_bbox.center.y - new_bbox.size_y / 2;
//        float y_max = new_bbox.center.y + new_bbox.size_y / 2;

    double re_align_size = 5;

    double x_min = new_bbox.center.x - new_bbox.size_x / 2;
    x_min = ((x_min - re_align_size) >= 0 ? (x_min - re_align_size) : 0);
    double x_max = new_bbox.center.x + new_bbox.size_x / 2;
    x_max = ((x_max + re_align_size) >= img_size.width ? img_size.width : (x_max + re_align_size));
    double y_min = new_bbox.center.y - new_bbox.size_y / 2;
    y_min = ((y_min - re_align_size) > 0 ? (y_min - re_align_size) : 0);
    double y_max = new_bbox.center.y + new_bbox.size_y / 2;
    y_max = ((y_max + re_align_size) > img_size.height ? img_size.height : (y_max + re_align_size));

    return point.x > x_min &&
           point.y > y_min &&
           point.x < x_max &&
           point.y < y_max;
  };

  std::vector<Cloud::Ptr> vector_cloud_frustums(count_detections);
  for (auto &cloud : vector_cloud_frustums) {
    cloud.reset(new Cloud);
  }

  // Iterate all points for projection and coloring and populating frustum vector

  for (const auto &point : cloud_in->points) {

    double distance = sqrt(pow(point.x, 2) + pow(point.y, 2));
    if (distance <= 0.2)
      continue;

    cv::Point point_in_image;

    pcltype::Point p_uv;

    {
      Eigen::Vector4d pos;
      pos << point.x, point.y, point.z, 1;

      Eigen::Vector4d vec_image_plane_coords = mat_point_transformer * pos;

      if (vec_image_plane_coords(2) <= 0)
        continue;

      point_in_image.x = (int) (vec_image_plane_coords(0) / vec_image_plane_coords(2));
      point_in_image.y = (int) (vec_image_plane_coords(1) / vec_image_plane_coords(2));
      p_uv.x = vec_image_plane_coords(2) * 1000;
      p_uv.y = -vec_image_plane_coords(0);
      p_uv.z = -vec_image_plane_coords(1);


      if (point_in_image.x < 0
          || point_in_image.y < 0
          || point_in_image.x >= img_size.width
          || point_in_image.y >= img_size.height)
        continue;


      for (size_t j = 0; j < count_detections; ++j) {

        const auto &detection = interested_detections.detections[j];

        if (point_is_within_rect(point_in_image, detection.bbox, img_size, id)) {
          vector_cloud_frustums[j]->points.push_back(point);
        }
      }

    }

  }

  auto distance_of_point = [](const Point &p) -> float {
    return std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2));
  };
  Cloud::Ptr all_frustum_all_points(new Cloud);
  for (auto &cloud : vector_cloud_frustums) {
    *all_frustum_all_points += *cloud;
  }

  HelperRosRelated::PublishCloud<Point>(all_frustum_all_points, image_detections_set.pub_all_frustum_all_points_,
                                        header_cloud_in.stamp, frame_lidar_);


  autoware_msgs::DetectedObjectArray detected_object_array;
  Cloud::Ptr frustum_closest_objects_center(new Cloud);

  for (size_t i = 0; i < vector_cloud_frustums.size(); ++i) {

    Cloud::Ptr cloud_frustum = vector_cloud_frustums[i];

    if (cloud_frustum->points.empty())
      continue;

    Cloud::Ptr frustum_cloud_all_centroids;
    std::vector<Cloud::Ptr> vector_clusters = PclStuff::Clusterer(frustum_cloud_all_centroids,
                                                                  cloud_frustum,
                                                                  2 * 0.25,
                                                                  1,
                                                                  999999);

    if (vector_clusters.empty())
      break;

    PclStuff::Point closest_point;

    double distance = 9999;

    for (const auto &point : frustum_cloud_all_centroids->points) {

      if (distance > distance_of_point(point)) {
        distance = distance_of_point(point);
        closest_point.x = point.x;
        closest_point.y = point.y;
        closest_point.z = point.z;
      }
    }

    autoware_msgs::DetectedObject object;
    object.pose.position.x = closest_point.x;
    object.pose.position.y = closest_point.y;
    object.pose.position.z = closest_point.z;
    object.id = interested_detections.detections[i].results[0].id;
    object.score = interested_detections.detections[i].results[0].score;

    detected_object_array.objects.push_back(object);

    frustum_closest_objects_center->points.push_back(closest_point);

    HelperRosRelated::PublishCloud<Point>(frustum_cloud_all_centroids,
                                          image_detections_set.pub_frustum_cloud_centroid_,
                                          header_cloud_in.stamp, frame_lidar_);
  }


  detected_object_array.header.stamp = header_cloud_in.stamp;
  image_detections_set.pub_object_pose_and_label_.publish(detected_object_array);

  if (!frustum_closest_objects_center->points.empty()) {
    HelperRosRelated::PublishCloud<Point>(frustum_closest_objects_center, image_detections_set.pub_center_points_,
                                          header_cloud_in.stamp, frame_lidar_);
  }


}
