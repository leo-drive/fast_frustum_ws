#ifndef INFERIDER_HELPERROSRELATED_H
#define INFERIDER_HELPERROSRELATED_H

#include <pcl_conversions/pcl_conversions.h>

class HelperRosRelated {
public:
  template<typename TypePoint>
  static void PublishCloud(const typename pcl::PointCloud<TypePoint>::ConstPtr &cloud,
                           const ros::Publisher &publisher,
                           const ros::Time &time_stamp,
                           const std::string &frame_id) {
    sensor_msgs::PointCloud2 cloud_to_publish;
    pcl::toROSMsg(*cloud, cloud_to_publish);
    cloud_to_publish.header.stamp = time_stamp;
    cloud_to_publish.header.frame_id = frame_id;
    publisher.publish(cloud_to_publish);
  }

private:
};


#endif //INFERIDER_HELPERROSRELATED_H
