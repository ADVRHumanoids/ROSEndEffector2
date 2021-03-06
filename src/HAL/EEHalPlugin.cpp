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

#include <end_effector/HAL/EEHalPlugin.h>

void ROSEE::EEHalPlugin::initialize(rclcpp::Node::SharedPtr node) {
    
    _node = node;
    
    //init sub to receive reference from UniversalROSEEEX
    //TODO take topic name from roslaunch
    std::string motor_reference_topic  = "motor_reference_pos";
    
    _motor_reference_sub = _node->create_subscription<sensor_msgs::msg::JointState>(motor_reference_topic, 1,
                                          std::bind(&ROSEE::EEHalPlugin::motor_reference_clbk, this, _1));
    
    std::string joint_state_topic = "/ros_end_effector/joint_states";
    
    _joint_state_pub = _node->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);    
    
    _hand_info_present = parseHandInfo();
   
    if (_hand_info_present) { 
        _hand_info_service_name = "hand_info";
    }
    
    _pressure_active = false; // if a derived class want to use this, it must call initPressureSensing()
    

    
}

bool ROSEE::EEHalPlugin::checkCommandPub() {
    
    return (_node->count_publishers(_motor_reference_sub->get_topic_name()) > 0 && _mr_msg.name.size() != 0);
}

bool ROSEE::EEHalPlugin::isHandInfoPresent() { return _hand_info_present; }

void ROSEE::EEHalPlugin::motor_reference_clbk(const sensor_msgs::msg::JointState::SharedPtr msg) {
    
    _mr_msg = *msg;
    
}

bool ROSEE::EEHalPlugin::publish_joint_state() {
    
    //NOTE _js_msg must be filled by the derived class
    _joint_state_pub->publish(_js_msg);
    
    return true;
    
}

bool ROSEE::EEHalPlugin::getFingersNames(std::vector<std::string> &fingers_names){
    
    if (this->fingers_names.size() != 0) {
    
        fingers_names = this->fingers_names;
        return true;

    } else {
        return false;
    }
    
}
bool ROSEE::EEHalPlugin::getMotorsNames(std::vector<std::string> &motors_names){
    
    if (this->motors_names.size() != 0) {
    
        motors_names = this->motors_names;
        return true;

    } 
    
    return false;
}

bool ROSEE::EEHalPlugin::getMotorStiffness(Eigen::VectorXd &motors_stiffness){
    
    if (this->motors_stiffness.size() != 0) {
    
        motors_stiffness = this->motors_stiffness;
        return true;

    } 
    
    return false;
}

bool ROSEE::EEHalPlugin::getTipsFrictions(Eigen::VectorXd &tips_frictions){
    
    if (this->tips_frictions.size() != 0) {
    
        tips_frictions = this->tips_frictions;
        return true;

    }
    
    return false; 
}

bool ROSEE::EEHalPlugin::getMotorTorqueLimits(Eigen::VectorXd &motors_torque_limits){
    
    if (this->motors_torque_limits.size() != 0) {
    
        motors_torque_limits = this->motors_torque_limits;
        return true;

    }
    return false;
}

bool ROSEE::EEHalPlugin::getTipJointToTipFrameX(Eigen::VectorXd &tip_joint_to_tip_frame_x) {
   
    if (this->tip_joint_to_tip_frame_x.size() != 0) {

        tip_joint_to_tip_frame_x = this->tip_joint_to_tip_frame_x;
        return true;

    } 
    return false;
}

bool ROSEE::EEHalPlugin::getTipJointToTipFrameY(Eigen::VectorXd &tip_joint_to_tip_frame_y) {
   
    if (this->tip_joint_to_tip_frame_y.size() != 0) {

        tip_joint_to_tip_frame_y = this->tip_joint_to_tip_frame_y;
        return true;

    } 
    
    return false;
 
}

bool ROSEE::EEHalPlugin::parseHandInfo() {
    
    std::string _rosee_config_path;
    _node->declare_parameter("ros_ee_config_path", "");
    if (! _node->get_parameter( "ros_ee_config_path", _rosee_config_path )) {
        return false;
    }
    
    std::ifstream ifile(_rosee_config_path);
    if (! ifile) {
        RCLCPP_WARN_STREAM (_node->get_logger(), "EEHALExecutor: config file " << _rosee_config_path << " not found");
        return false;
    }
    
    YAML::Node node = YAML::LoadFile(_rosee_config_path);
    
    if (! node["hand_info"]) {
        RCLCPP_WARN_STREAM (_node->get_logger(),"EEHALExecutor: config file " << _rosee_config_path << " does not contain "
            << "hand_info node. I will not parse hand information");
        
        return false;
    }
    
    if(node["hand_info"]["fingers_names"]){

       fingers_names = node["hand_info"]["fingers_names"].as<std::vector<std::string>>();
    }
    
    if(node["hand_info"]["motors_names"]){

       motors_names = node["hand_info"]["motors_names"].as<std::vector<std::string>>();
    }
    
    if(node["hand_info"]["motors_stiffness"]){

       motors_stiffness = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["motors_stiffness"]);
    }
    
    if(node["hand_info"]["tips_frictions"]){

       tips_frictions = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["tips_frictions"]);
    }
    
    if(node["hand_info"]["motors_torque_limits"]){

       motors_torque_limits = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["motors_torque_limits"]);
    }
    
    if(node["hand_info"]["tip_joint_to_tip_frame_x"]){

       tip_joint_to_tip_frame_x = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["tip_joint_to_tip_frame_x"]);
    }
    
    if(node["hand_info"]["tip_joint_to_tip_frame_y"]){

       tip_joint_to_tip_frame_y = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["tip_joint_to_tip_frame_y"]);
    }
    
    return true;
}

bool ROSEE::EEHalPlugin::init_hand_info_response() {
    
    _hand_info_response.fingers_names = fingers_names;
    
    _hand_info_response.motors_names = motors_names;
        
    _hand_info_response.motors_stiffness =
        ROSEE::Utils::eigenVectorToStdVector(motors_stiffness);
        
    _hand_info_response.tips_frictions =
        ROSEE::Utils::eigenVectorToStdVector(tips_frictions);
        
    _hand_info_response.motors_torque_limits =
        ROSEE::Utils::eigenVectorToStdVector(motors_torque_limits);
    
    return true;
}

bool ROSEE::EEHalPlugin::setHandInfoCallback() {
    
    _hand_info_server = _node->create_service<rosee_msg::srv::HandInfo>(_hand_info_service_name, 
                                                                        std::bind(&ROSEE::EEHalPlugin::handInfoEEHalCallback, this, _1, _2));
    return true;
}

bool ROSEE::EEHalPlugin::handInfoEEHalCallback (
    const std::shared_ptr<rosee_msg::srv::HandInfo::Request> request,
    std::shared_ptr<rosee_msg::srv::HandInfo::Response> response) {
   
    //generic hal does not read the request
    
    response = std::make_shared<rosee_msg::srv::HandInfo::Response> (_hand_info_response);
    
    return true;
}

bool ROSEE::EEHalPlugin::initPressureSensing()
{
    
    std::string topic_name = "/ros_end_effector/pressure_phalanges";
    
    _pressure_pub = _node->create_publisher<rosee_msg::msg::MotorPhalangePressure>(topic_name, 10);

    _pressure_active = true;
    
    return true;
}

bool ROSEE::EEHalPlugin::publish_pressure() {
    
    //NOTE _pressure_msg must be filled by the derived class
    _pressure_pub->publish(_pressure_msg);
    
    return true;
    
}

Eigen::VectorXd ROSEE::EEHalPlugin::getMotorReference() const {
    
    Eigen::VectorXd motorRef;
    motorRef.resize(_mr_msg.name.size());
    for (size_t i=0; i<_mr_msg.name.size(); i++ ) {
        
        motorRef(i) = _mr_msg.position.at(i);
    }
    
    return motorRef;
}

Eigen::VectorXd ROSEE::EEHalPlugin::getJointPosition() const {

    Eigen::VectorXd jointPos;
    jointPos.resize(_js_msg.name.size());
    for (size_t i=0; i<_js_msg.name.size(); i++ ) {
        
        jointPos(i) = _js_msg.position.at(i);
    }
    
    return jointPos;
}

Eigen::VectorXd ROSEE::EEHalPlugin::getJointEffort() const {

    Eigen::VectorXd jointEffort;
    jointEffort.resize(_js_msg.name.size());
    for (size_t i=0; i<_js_msg.name.size(); i++ ) {
        
        jointEffort(i) = _js_msg.effort.at(i);
    }
    
    return jointEffort;
}

Eigen::MatrixXd ROSEE::EEHalPlugin::getPressure() const {

    Eigen::MatrixXd pressure;
    pressure.resize(4, _pressure_msg.pressure_finger1.size()); //message has 4 finger field
    for (size_t i=0; i<_pressure_msg.pressure_finger1.size(); i++ ) {

        pressure(0, i) = _pressure_msg.pressure_finger1.at(i);
        pressure(1, i) = _pressure_msg.pressure_finger2.at(i);
        pressure(2, i) = _pressure_msg.pressure_finger3.at(i);
        pressure(3, i) = _pressure_msg.pressure_thumb.at(i);
    }
    
    return pressure;
    
}
