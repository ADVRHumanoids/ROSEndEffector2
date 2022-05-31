/*
 * Copyright 2020 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROSSERVICEHANDLER_H
#define ROSSERVICEHANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <end_effector/MapActionHandler.h>
#include <rosee_msg/msg/grasping_primitive_aggregated.hpp>
#include <rosee_msg/srv/grasping_primitive_aggregated_available.hpp>
#include <rosee_msg/srv/selectable_pair_info.hpp>
#include <rosee_msg/srv/grasping_actions_available.hpp>
#include <rosee_msg/msg/grasping_action.hpp>
#include <rosee_msg/msg/motor_position.hpp>
#include <rosee_msg/srv/hand_info.hpp>
#include <rosee_msg/msg/new_generic_grasping_action.hpp>
#include <rosee_msg/srv/new_generic_grasping_action_srv.hpp>


namespace ROSEE {
/**
 * @todo write docs
 */
class RosServiceHandler
{
public:
    /**
     * Default constructor
     */
    RosServiceHandler(rclcpp::Node::SharedPtr node, ROSEE::MapActionHandler::Ptr, std::string path2saveYamlGeneric);
    bool init(unsigned int nFinger);

    //this response is filled by UROSEE in the initialization
    rosee_msg::srv::HandInfo::Response handInfoResponse;
private:
    
    MapActionHandler::Ptr _mapActionHandler;
    std::string _path2saveYamlGeneric;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Service<rosee_msg::srv::GraspingPrimitiveAggregatedAvailable>::SharedPtr _serverPrimitiveAggregated;
    rclcpp::Service<rosee_msg::srv::SelectablePairInfo>::SharedPtr _server_selectablePairInfo;
    rclcpp::Service<rosee_msg::srv::GraspingActionsAvailable>::SharedPtr _serverGraspingActions;
    rclcpp::Service<rosee_msg::srv::NewGenericGraspingActionSrv>::SharedPtr _serverNewGraspingAction;
    
    rclcpp::Service<rosee_msg::srv::HandInfo>::SharedPtr _serverHandInfo;
    
    
    unsigned int nFinger;
    bool selectablePairInfoCallback ( const std::shared_ptr<rosee_msg::srv::SelectablePairInfo::Request> request,
                                      std::shared_ptr<rosee_msg::srv::SelectablePairInfo::Response> response);
    
    bool graspingActionsCallback(const std::shared_ptr<rosee_msg::srv::GraspingActionsAvailable::Request> request,   
                                std::shared_ptr<rosee_msg::srv::GraspingActionsAvailable::Response> response);
  
    rosee_msg::msg::GraspingAction fillGraspingActionMsg(ROSEE::ActionPrimitive::Ptr primitive);
    rosee_msg::msg::GraspingAction fillGraspingActionMsg(ROSEE::ActionGeneric::Ptr generic);
    rosee_msg::msg::GraspingAction fillGraspingActionMsg(ROSEE::ActionTimed::Ptr timed);

    /**
     * @brief Internal function called by each of the fillGraspingActionMsg, it fills the GraspingAction message
     * with the info that are always present in any kind of action (like name and type). 
     * Each specific \ref fillGraspingActionMsg will insert fill the field specific for the action type 
     * (like time margins for timed actions)
     *
     * @param action the action that must be sent as response
     * @param msg the ROS message that must be filled with action info
     */
    void fillCommonInfoGraspingActionMsg(ROSEE::Action::Ptr action, rosee_msg::msg::GraspingAction* msg);

    
    bool primitiveAggregatedCallback(
        const std::shared_ptr<rosee_msg::srv::GraspingPrimitiveAggregatedAvailable::Request> request,
        std::shared_ptr<rosee_msg::srv::GraspingPrimitiveAggregatedAvailable::Response> response);

    rosee_msg::msg::GraspingPrimitiveAggregated fillPrimitiveAggregatedMsg(
                        ROSEE::MapActionHandler::ActionPrimitiveMap primitiveMap);
    rosee_msg::msg::GraspingPrimitiveAggregated fillPrimitiveAggregatedMsg(
                        ROSEE::ActionPrimitive::Ptr primitive);
    
    bool handInfoCallback(
        const std::shared_ptr<rosee_msg::srv::HandInfo::Request> request,
        std::shared_ptr<rosee_msg::srv::HandInfo::Response> response);

    bool newGraspingActionCallback(
        const std::shared_ptr<rosee_msg::srv::NewGenericGraspingActionSrv::Request> request,
        std::shared_ptr<rosee_msg::srv::NewGenericGraspingActionSrv::Response> response);
};

} //end namespace

#endif // ROSSERVICEHANDLER_H
