//
// Created by ubuntu on 3/23/24.
//

#ifndef MAP_CREATOR_TEST_MAPPING3DTEST_H_
#define MAP_CREATOR_TEST_MAPPING3DTEST_H_

#include <gtest/gtest.h>
#include "../CartographerModule3D.h"

class Mapping3DTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Initialize test data
    dir_ = "test_dir";
    file_ = "test_file";
    poseUpdateCallback_ = [](auto) {};
    lidarScanTimeHz_ = 10.0f;
    imuNames_ = {"imu1", "imu2"};
    odomNames_ = {"odom1", "odom2"};
    rangeNames_ = {"range1", "range2"};
  }

  void TearDown() override {
    // Clean up after each test
  }

  std::string dir_;
  std::string file_;
  PoseUpdateCallback3D poseUpdateCallback_;
  float lidarScanTimeHz_;
  std::vector<std::string> imuNames_;
  std::vector<std::string> odomNames_;
  std::vector<std::string> rangeNames_;
};

#endif //MAP_CREATOR_TEST_MAPPING3DTEST_H_
