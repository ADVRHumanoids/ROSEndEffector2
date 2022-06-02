/*
 * Copyright (C) 2019 IIT-HHCM
 * Author: Luca Muratore
 * email:  luca.muratore@iit.it
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include <end_effector/HAL/DummyHalPlugin.h>

void ROSEE::DummyHalPlugin::initialize (rclcpp::Node::SharedPtr node) {
    
    ROSEE::EEHalPlugin::initialize(node);
    
    _hal_joint_state_pub = _node->create_publisher<sensor_msgs::msg::JointState>("/dummyHal/joint_command", 1);
    _hal_joint_state_sub = _node->create_subscription<sensor_msgs::msg::JointState>("/dummyHal/joint_states", 1, std::bind(&ROSEE::DummyHalPlugin::hal_js_clbk, this,_1));
}


bool ROSEE::DummyHalPlugin::sense() {
    //do nothing, it is the hal_js_clbk who "sense"
    return true;
}

bool ROSEE::DummyHalPlugin::move() {
    _hal_joint_state_pub->publish(_mr_msg);
    return true;
}

void ROSEE::DummyHalPlugin::hal_js_clbk(const sensor_msgs::msg::JointState::SharedPtr msg) {
    
    _js_msg = *msg;
}

