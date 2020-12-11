#include "JskHelper.h"


jsk_recognition_msgs::PolygonArray::Ptr JskHelper::CornersAndSourceToPolygons(
  const pcltype::Point &p_top_left,
  const pcltype::Point &p_top_right,
  const pcltype::Point &p_bot_left,
  const pcltype::Point &p_bot_right,
  const pcltype::Point &p_source,
  const std_msgs::Header &header) {
  jsk_recognition_msgs::PolygonArray::Ptr msg_polygons(new jsk_recognition_msgs::PolygonArray);
  msg_polygons->header = header;

  AddCornersToPolygons(msg_polygons,
                       p_top_left,
                       p_top_right,
                       p_bot_left,
                       p_bot_right,
                       p_source);

  return msg_polygons;
}

bool JskHelper::AddCornersToPolygons(
  jsk_recognition_msgs::PolygonArray::Ptr &polygons,
  const pcltype::Point &p_top_left,
  const pcltype::Point &p_top_right,
  const pcltype::Point &p_bot_left,
  const pcltype::Point &p_bot_right,
  const pcltype::Point &p_source) {
  if (!polygons) {
    std::cout << "polygons is nullptr." << std::endl;
    return false;
  }

  geometry_msgs::PolygonStamped polygon_viewport;
  polygon_viewport.header = polygons->header;
  polygon_viewport.polygon.points.push_back(PclPointToGeometryPoint(p_top_left));
  polygon_viewport.polygon.points.push_back(PclPointToGeometryPoint(p_top_right));
  polygon_viewport.polygon.points.push_back(PclPointToGeometryPoint(p_bot_right));
  polygon_viewport.polygon.points.push_back(PclPointToGeometryPoint(p_bot_left));
  polygons->polygons.push_back(polygon_viewport);

  auto add_as_triangle = [&polygons](const pcltype::Point &p1,
                                     const pcltype::Point &p2,
                                     const pcltype::Point &p3) {
    geometry_msgs::PolygonStamped polygon_left;
    polygon_left.header = polygons->header;
    polygon_left.polygon.points.push_back(PclPointToGeometryPoint(p1));
    polygon_left.polygon.points.push_back(PclPointToGeometryPoint(p2));
    polygon_left.polygon.points.push_back(PclPointToGeometryPoint(p3));
    polygons->polygons.push_back(polygon_left);
  };

  // left triangle
  add_as_triangle(p_top_left, p_bot_left, p_source);
  // bottom triangle
  add_as_triangle(p_bot_left, p_bot_right, p_source);
  // right triangle
  add_as_triangle(p_bot_right, p_top_right, p_source);
  // top triangle
  add_as_triangle(p_top_right, p_top_left, p_source);

  return true;
}

geometry_msgs::Point32 JskHelper::PclPointToGeometryPoint(const pcltype::Point &p_in) {
  geometry_msgs::Point32 p_geom;
  p_geom.x = p_in.x;
  p_geom.y = p_in.y;
  p_geom.z = p_in.z;
  return p_geom;
}
