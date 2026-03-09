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

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>
#include <string>
#include "livox_ros_driver2.h"
#include "dora_node.h"
#include "lddc_dora.h"
#include "lds_lidar.h"

extern "C"{
#include "node_api.h"
}

using namespace livox_ros;

int main(int argc, char **argv) {
  std::cout << "Livox Dora Driver Version: " << LIVOX_ROS_DRIVER2_VERSION_STRING << std::endl;

  // Initialize Dora context
  void *dora_context = init_dora_context_from_env();
  if (dora_context == NULL) {
    std::cerr << "Failed to initialize Dora context" << std::endl;
    return -1;
  }

  std::cout << "Dora context initialized successfully" << std::endl;

  // Create Dora node
  livox_ros::DoraNode dora_node(dora_context);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;  // Use PointCloud2 format for JSON serialization
  int multi_topic = 0;                // Single topic mode
  int data_src = kSourceRawLidar;     // Raw lidar data source
  double publish_freq = 10.0;         // 10Hz as requested
  int output_type = kOutputToRos;     // Output to Dora (reuse this enum)
  std::string frame_id = "livox_frame";
  bool lidar_bag = false;             // No bag file support in Dora version
  bool imu_bag = false;

  std::cout << "Data source: " << data_src << std::endl;
  std::cout << "Publish frequency: " << publish_freq << " Hz" << std::endl;

  /** Lidar data distribute control and lidar data source set */
  dora_node.lddc_ptr_ = std::make_unique<LddcDora>(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, &dora_node);

  if (data_src == kSourceRawLidar) {
    std::cout << "Data Source is raw lidar." << std::endl;

    // Use default config path for MID360
    std::string user_config_path = "livox_driver/config/MID360_config.json";
    std::cout << "Config file: " << user_config_path << std::endl;

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    dora_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      std::cout << "Init lds lidar successfully!" << std::endl;
    } else {
      std::cerr << "Init lds lidar failed!" << std::endl;
      free_dora_context(dora_context);
      return -1;
    }
  } else {
    std::cerr << "Invalid data src (" << data_src << "), only raw lidar is supported" << std::endl;
    free_dora_context(dora_context);
    return -1;
  }

  std::cout << "Starting Dora event loop..." << std::endl;

  // Main Dora event loop - event-driven data publishing
  bool running = true;
  while (running) {
    void *event = dora_next_event(dora_context);
    if (event == NULL) {
      std::cerr << "ERROR: unexpected end of event" << std::endl;
      break;
    }

    enum DoraEventType ty = read_dora_event_type(event);

    if (ty == DoraEventType_Input) {
      char *data;
      size_t data_len;
      char *id;
      size_t id_len;
      read_dora_input_data(event, &data, &data_len);
      read_dora_input_id(event, &id, &id_len);
      
      if (strncmp(id, "timer", id_len) == 0) {
        // Timer input received - publish data at exact 10Hz frequency
        dora_node.lddc_ptr_->DistributePointCloudData();
        dora_node.lddc_ptr_->DistributeImuData();
      }
    }
    else if (ty == DoraEventType_Stop) {
      std::cout << "Received stop event" << std::endl;
      running = false;
    }
    else {
      std::cout << "Received unexpected event: " << ty << std::endl;
    }

    free_dora_event(event);
  }

  std::cout << "Shutting down..." << std::endl;

  // Cleanup
  free_dora_context(dora_context);
  std::cout << "Livox Dora Driver finished successfully" << std::endl;

  return 0;
}