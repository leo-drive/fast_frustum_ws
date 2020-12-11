#include "LidcamHelpers.h"

using ::pcltype::Cloud;
using ::pcltype::Point;
using ::pcltype::CloudColored;
using ::pcltype::PointColored;

bool LidcamHelpers::PointToColoredPoint(pcltype::PointColored &point_colored,
                                        cv::Point &point_in_image,
                                        pcltype::Point &point2,
                                        const cv::Mat &image,
                                        const pcltype::Point &point,
                                        const Eigen::Matrix4d &mat_transformer) {
  Eigen::Vector4d pos;
  pos << point.x, point.y, point.z, 1;

  Eigen::Vector4d vec_image_plane_coords = mat_transformer * pos;

  if (vec_image_plane_coords(2) <= 0)
    return false;

  point_in_image.x = (int) (vec_image_plane_coords(0) / vec_image_plane_coords(2));
  point_in_image.y = (int) (vec_image_plane_coords(1) / vec_image_plane_coords(2));
  point2.x = vec_image_plane_coords(2) * 1000;
  point2.y = -vec_image_plane_coords(0);
  point2.z = -vec_image_plane_coords(1);
//  std::cout << "vec_image_plane_coords: " << vec_image_plane_coords << std::endl;

  if (point_in_image.x < 0
      || point_in_image.y < 0
      || point_in_image.x >= image.size().width
      || point_in_image.y >= image.size().height)
    return false;

  cv::Vec3b color_of_point = image.at<cv::Vec3b>(point_in_image);
  point_colored = pcl::PointXYZRGB(color_of_point[2],
                                   color_of_point[1],
                                   color_of_point[0]);
  point_colored.x = point.x;
  point_colored.y = point.y;
  point_colored.z = point.z;
  return true;
}


std::tuple<int, int, int> LidcamHelpers::ColorPicker(double distance) {

  double hh, p, q, t, ff;
  long i;
  double v = 0.75;
  double s = 0.75;
  double r, g, b;
  double h = ((30.0 - distance) / 30.0) * 360.0;
//  double h = ((255.0 - distance) / 255.0) * 360.0;
  hh = h;
  if (hh >= 360.0) hh = 0.0;
  hh /= 60.0;
  i = (long) hh;
  ff = hh - i;
  p = v * (1.0 - s);
  q = v * (1.0 - (s * ff));
  t = v * (1.0 - (s * (1.0 - ff)));

  switch (i) {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;

    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    case 5:
    default:
      r = v;
      g = p;
      b = q;
      break;
  }
  return std::make_tuple((int) (r * 255), (int) (g * 255), (int) (b * 255));
}

pcltype::Point LidcamHelpers::PointImageToPoint3D(const cv::Point &point_in_image,
                                                  float dist,
                                                  const Eigen::Matrix4d &mat_point_transformer_inverse,
                                                  const Eigen::Matrix4d &mat_velo_to_cam) {

  Eigen::Vector4d vec_image_plane_coords(point_in_image.x, point_in_image.y, 1, 1);
  Eigen::Vector4d pos = mat_point_transformer_inverse * vec_image_plane_coords;

  Eigen::Vector4d pos_cam_origin = mat_velo_to_cam.inverse().col(3);

//  Eigen::Vector3d difference = pos.topRows(3) - pos_cam_origin.topRows(3);

  Eigen::Vector3d unit_vector = (pos.topRows(3) - pos_cam_origin.topRows(3)).normalized();

  Eigen::Vector3d pos_scaled = pos_cam_origin.topRows(3) + unit_vector * dist;

//  Eigen::Vector4d pos_in_velo = mat_velo_to_cam.inverse() * pos;
//  pos_in_velo *= dist;

//  Eigen::Vector4d pos_back_in_cam = mat_velo_to_cam * pos_in_velo;
//  Eigen::Vector4d pos_back_in_cam = pos_in_velo;

  pcltype::Point p;
  p.x = pos_scaled(0);
  p.y = pos_scaled(1);
  p.z = pos_scaled(2);
  return p;
}
