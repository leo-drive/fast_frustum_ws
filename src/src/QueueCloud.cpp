//
// Created by kaan on 24.05.2020.
//
#include <algorithm>
#include "QueueCloud.h"

QueueCloud::QueueCloud(size_t maxQueueSize) : max_queue_size_(maxQueueSize) {
}


void QueueCloud::Push(QueueCloud::CloudAndHeader &new_element) {
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.push_back(new_element);
  if (queue_.size() > max_queue_size_) {
    queue_.pop_front();
  }

}

QueueCloud::CloudAndHeader QueueCloud::GetBestCloud(const std_msgs::Header &header,  int & index) {

  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<double> diffs;
  std::transform(queue_.begin(), queue_.end(), std::back_inserter(diffs),
                 [&header](const CloudAndHeader &element) -> double {
                   return std::abs((header.stamp - element.Header.stamp).toSec());
                 });

  auto smallest = std::min_element(diffs.begin(), diffs.end());
  index = std::distance(diffs.begin(), smallest);

  return queue_[std::distance(diffs.begin(), smallest)];
}

size_t QueueCloud::GetSize() {
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();

}