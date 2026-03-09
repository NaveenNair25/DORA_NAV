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

#ifndef LIVOX_DORA_DRIVER2_LDDC_H_
#define LIVOX_DORA_DRIVER2_LDDC_H_

#include "include/livox_ros_driver2.h"
#include "dora_node.h"
#include "lds.h"
#include <nlohmann/json.hpp>

namespace livox_ros {

/** Send pointcloud message Data to dora subscriber */
typedef enum {
  kOutputToRos = 0,  // Reuse this enum for Dora output
} DestinationOfMessageOutput;

/** The message type of transfer */
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg = 2,
  kLivoxImuMsg = 3,
} TransferType;

class DoraNode;

class LddcDora final {
 public:
  LddcDora(int format, int multi_topic, int data_src, int output_type, double frq,
      std::string &frame_id, DoraNode* dora_node);
  ~LddcDora();

  int RegisterLds(Lds *lds);
  void DistributePointCloudData(void);
  void DistributeImuData(void);
  void PrepareExit(void);

  uint8_t GetTransferFormat(void) { return transfer_format_; }
  uint8_t IsMultiTopic(void) { return use_multi_topic_; }
  void SetPublishFrq(uint32_t frq) { publish_frq_ = frq; }

 public:
  Lds *lds_;

 private:
  void PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar);
  void PollingLidarImuData(uint8_t index, LidarDevice *lidar);

  void PublishPointcloud2(LidarDataQueue *queue, uint8_t index);
  void PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index);

  // JSON serialization methods
  nlohmann::json SerializePointCloudToJson(const StoragePacket& pkg);
  nlohmann::json SerializeImuToJson(const ImuData& imu_data);
  
  // Dora publishing methods
  void PublishPointCloudJson(const nlohmann::json& json_data, const uint8_t index);
  void PublishImuJson(const nlohmann::json& json_data, const uint8_t index);

  std::string GetPointCloudTopicName(uint8_t index);
  std::string GetImuTopicName(uint8_t index);

 private:
  uint8_t transfer_format_;
  uint8_t use_multi_topic_;
  uint8_t data_src_;
  uint8_t output_type_;
  double publish_frq_;
  uint32_t publish_period_ns_;
  std::string frame_id_;

  DoraNode* dora_node_;
};

}  // namespace livox_ros

#endif // LIVOX_DORA_DRIVER2_LDDC_H_