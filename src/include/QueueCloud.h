//
// Created by kaan on 24.05.2020.
//

#ifndef SRC_QUEUECLOUD_H
#define SRC_QUEUECLOUD_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <deque>
#include <mutex>
#include "PointCloudTypes.h"

class QueueCloud {

public:
  using Ptr = std::shared_ptr<QueueCloud>;

  explicit QueueCloud(size_t maxQueueSize);

  struct CloudAndHeader {
    pcltype::Cloud::Ptr Cloud;
    std_msgs::Header Header;
  };

  void Push(CloudAndHeader &new_element);

  QueueCloud::CloudAndHeader GetBestCloud(const std_msgs::Header &header, int & index);

  size_t GetSize();


private:
  std::deque<CloudAndHeader> queue_;
  size_t max_queue_size_;
  std::mutex mutex_;


};


#endif //SRC_QUEUECLOUD_H
