/**
 * Copyright 2021 The Kali Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "hardware_driver/indoor_hardware_driver.h"
#include "hardware_driver/outdoor_hardware_driver.h"
#include <gtest/gtest.h>
#include "database/code_table.h"
#include "database/database.h"

namespace hardware_driver {

class HardwareTest : public ::testing::Test {
protected:
	
  void SetUp() override {

    indoor_hardware_ptr = std::make_shared<IndoorHardware>(nh,com_name,robot_id,robot_type);
    outdoor_hardware_ptr = std::make_shared<OutdoorHardware>(nh,com_name,robot_id,robot_type);
    database_ptr = std::make_shared<database::Database>(nh);
    
    outdoor_hardware_ptr->switch_state.bits = 8;
    int ultrasonic_state = outdoor_hardware_ptr->switch_state.ultrasonic_state;
    int exploration_ditch_switch_state = outdoor_hardware_ptr->switch_state.exploration_ditch_switch_state;
    int wheel1_enable_state = outdoor_hardware_ptr->switch_state.wheel1_enable_state;
    int wheel2_enable_state = outdoor_hardware_ptr->switch_state.wheel2_enable_state;
    int wheel3_enable_state = outdoor_hardware_ptr->switch_state.wheel3_enable_state;
    int wheel4_enable_state = outdoor_hardware_ptr->switch_state.wheel4_enable_state;
    // expect 0_0_0_1_0_0
    LOG(INFO) << ultrasonic_state << "_" << exploration_ditch_switch_state << "_" << wheel1_enable_state << "_" << wheel2_enable_state << "_" << wheel3_enable_state << "_" << wheel4_enable_state;
  
    outdoor_hardware_ptr->wheel_state.bits = 0;
    outdoor_hardware_ptr->wheel_state.wheel1_enable_state = 1;
    outdoor_hardware_ptr->wheel_state.wheel2_enable_state = 1;
    outdoor_hardware_ptr->wheel_state.wheel3_enable_state = 1;
    outdoor_hardware_ptr->wheel_state.wheel4_enable_state = 1;
    // expect 15
    LOG(INFO) << (int)outdoor_hardware_ptr->wheel_state.bits;
  
  }

	void TearDown() override {
		outdoor_hardware_ptr.reset();
	}
  
  std::shared_ptr<OutdoorHardware> outdoor_hardware_ptr;
  std::shared_ptr<IndoorHardware> indoor_hardware_ptr;
  std::shared_ptr<database::Database> database_ptr;
  std::string robot_id = "1";
  std::string robot_type = "1";
  ros::NodeHandle nh;
  std::string com_name;
};


TEST_F(HardwareTest, hardware_driver_device_open) {

}
}// namespace hardware_driver

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "hardware_driver_test");
  return RUN_ALL_TESTS();
}