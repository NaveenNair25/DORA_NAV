//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lddc_dora.h"
#include "comm/ldq.h"
#include "comm/comm.h"
#include "dora_node.h"
#include "lds_lidar.h"

#include <inttypes.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdint.h>
#include <nlohmann/json.hpp>

namespace livox_ros {

/** Lidar Data Distribute Control for Dora--------------------------------------------*/
LddcDora::LddcDora(int format, int multi_topic, int data_src, int output_type,
    double frq, std::string &frame_id, DoraNode* dora_node)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id),
      dora_node_(dora_node) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
  
  std::cout << "LddcDora initialized with publish frequency: " << publish_frq_ << " Hz" << std::endl;
}

LddcDora::~LddcDora() {
  PrepareExit();
  std::cout << "LddcDora destroyed" << std::endl;
}

int LddcDora::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

void LddcDora::DistributePointCloudData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributePointCloudData is RequestExit" << std::endl;
    return;
  }
  
  lds_->pcd_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarDataQueue *p_queue = &lidar->data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarPointCloudData(lidar_id, lidar);    
  }
}

void LddcDora::DistributeImuData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributeImuData is RequestExit" << std::endl;
    return;
  }
  
  lds_->imu_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarImuDataQueue *p_queue = &lidar->imu_data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarImuData(lidar_id, lidar);
  }
}

void LddcDora::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->data;
  if (p_queue == nullptr || p_queue->storage_packet == nullptr) {
    return;
  }

  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) {
    PublishPointcloud2(p_queue, index);
  }
}

void LddcDora::PollingLidarImuData(uint8_t index, LidarDevice *lidar) {
  LidarImuDataQueue& p_queue = lidar->imu_data;
  while (!lds_->IsRequestExit() && !p_queue.Empty()) {
    PublishImuData(p_queue, index);
  }
}

void LddcDora::PrepareExit(void) {
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

void LddcDora::PublishPointcloud2(LidarDataQueue *queue, uint8_t index) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish point cloud2 failed, the pkg points is empty.\n");
      continue;
    }

    // Serialize to JSON and publish
    nlohmann::json json_data = SerializePointCloudToJson(pkg);
    PublishPointCloudJson(json_data, index);
  }
}

void LddcDora::PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index) {
  ImuData imu_data;
  if (!imu_data_queue.Pop(imu_data)) {
    return;
  }

  // Serialize to JSON and publish
  nlohmann::json json_data = SerializeImuToJson(imu_data);
  PublishImuJson(json_data, index);
}

nlohmann::json LddcDora::SerializePointCloudToJson(const StoragePacket& pkg) {
  nlohmann::json json_data;
  
  // Header information
  json_data["header"]["frame_id"] = frame_id_;
  json_data["header"]["timestamp"] = pkg.base_time / 1e9;  // Convert nanoseconds to seconds
  json_data["header"]["seq"] = static_cast<uint32_t>(pkg.base_time / 1000000); // Simple sequence number
  
  // Point cloud data
  json_data["width"] = pkg.points_num;
  json_data["height"] = 1;
  json_data["is_dense"] = true;
  
  // Points array with XYZI format
  json_data["points"] = nlohmann::json::array();
  
  for (size_t i = 0; i < pkg.points_num; ++i) {
    nlohmann::json point;
    point["x"] = pkg.points[i].x;
    point["y"] = pkg.points[i].y;
    point["z"] = pkg.points[i].z;
    point["intensity"] = pkg.points[i].intensity;
    
    // Additional Livox-specific fields
    point["tag"] = pkg.points[i].tag;
    point["line"] = pkg.points[i].line;
    point["offset_time"] = pkg.points[i].offset_time / 1e9;  // Convert nanoseconds to seconds
    
    json_data["points"].push_back(point);

    /* Lidar Data Debug */

    // std::cout << pkg.points[i].x << '\t' << pkg.points[i].y << '\t' << pkg.points[i].z << std::endl; 

    /********************/
  }
  
  return json_data;
}

nlohmann::json LddcDora::SerializeImuToJson(const ImuData& imu_data) {
  nlohmann::json json_data;
  
  // Header information
  json_data["header"]["frame_id"] = frame_id_;
  json_data["header"]["timestamp"] = imu_data.time_stamp / 1e9;  // Convert nanoseconds to seconds
  
  // Angular velocity (gyroscope data)
  json_data["angular_velocity"]["x"] = imu_data.gyro_x;
  json_data["angular_velocity"]["y"] = imu_data.gyro_y;
  json_data["angular_velocity"]["z"] = imu_data.gyro_z;
  
  // Linear acceleration (accelerometer data)
  json_data["linear_acceleration"]["x"] = imu_data.acc_x;
  json_data["linear_acceleration"]["y"] = imu_data.acc_y;
  json_data["linear_acceleration"]["z"] = imu_data.acc_z;

  /* IMU Data Debug */
  
  // std::cout << imu_data.gyro_x << '\t' << imu_data.gyro_y << '\t' << imu_data.gyro_z << std::endl;

  /******************/
  
  return json_data;
}

void LddcDora::PublishPointCloudJson(const nlohmann::json& json_data, const uint8_t index) {
  if (!dora_node_) {
    std::cerr << "Dora node is null" << std::endl;
    return;
  }
  
  std::string topic_name = GetPointCloudTopicName(index);
  std::string json_string = json_data.dump();
  
  int result = dora_node_->SendOutput(topic_name, json_string);
  if (result != 0) {
    std::cerr << "Failed to publish point cloud data to topic: " << topic_name << std::endl;
  }
}

void LddcDora::PublishImuJson(const nlohmann::json& json_data, const uint8_t index) {
  if (!dora_node_) {
    std::cerr << "Dora node is null" << std::endl;
    return;
  }
  
  std::string topic_name = GetImuTopicName(index);
  std::string json_string = json_data.dump();
  
  int result = dora_node_->SendOutput(topic_name, json_string);
  if (result != 0) {
    std::cerr << "Failed to publish IMU data to topic: " << topic_name << std::endl;
  }
}

std::string LddcDora::GetPointCloudTopicName(uint8_t index) {
  if (use_multi_topic_) {
    return "pointcloud_" + std::to_string(index);
  } else {
    return "pointcloud";
  }
}

std::string LddcDora::GetImuTopicName(uint8_t index) {
  if (use_multi_topic_) {
    return "imu_" + std::to_string(index);
  } else {
    return "imu";
  }
}

}  // namespace livox_ros