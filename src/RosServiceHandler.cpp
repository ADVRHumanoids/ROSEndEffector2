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

#include <end_effector/RosServiceHandler.h>

ROSEE::RosServiceHandler::RosServiceHandler( rclcpp::Node::SharedPtr node, ROSEE::MapActionHandler::Ptr mapActionHandler, std::string path2saveYamlGeneric) {
    
    if (mapActionHandler == nullptr) {
        RCLCPP_ERROR_STREAM (_node->get_logger(), "[RosServiceHandler " << __func__ << " ] the mapActionHandler in not initialized");
        return;
    }
    
    this->_mapActionHandler = mapActionHandler;
    this->_path2saveYamlGeneric = path2saveYamlGeneric;

    this->_node = node;
    
}

//TODO see if this argument can be avoided, maybe use extract_keys_merged in map action handler?
bool ROSEE::RosServiceHandler::init(unsigned int nFinger) {
    
    this->nFinger = nFinger;
    
    std::string graspingActionsSrvName, actionInfoServiceName, 
      selectablePairInfoServiceName, handInfoServiceName, newGraspingActionServiceName;

    _node->declare_parameter("grasping_action_srv_name","grasping_actions_available");
    _node->declare_parameter("primitive_aggregated_srv_name","primitives_aggregated_available");
    _node->declare_parameter("selectable_finger_pair_info","selectable_finger_pair_info");
    _node->declare_parameter("hand_info","hand_info");
    _node->declare_parameter("new_grasping_action_srv_name","new_generic_grasping_action");
    
    _node->get_parameter("grasping_action_srv_name", graspingActionsSrvName);
    _node->get_parameter("primitive_aggregated_srv_name", actionInfoServiceName);
    _node->get_parameter("selectable_finger_pair_info", selectablePairInfoServiceName);
    _node->get_parameter("hand_info", handInfoServiceName);
    _node->get_parameter("new_grasping_action_srv_name", newGraspingActionServiceName);
    
    using std::placeholders::_1;
    using std::placeholders::_2;
    
    _serverGraspingActions = _node->create_service<rosee_msg::srv::GraspingActionsAvailable>(graspingActionsSrvName,
                                                   std::bind(&RosServiceHandler::graspingActionsCallback, this, _1, _2));
                                                   
    _serverPrimitiveAggregated = _node->create_service<rosee_msg::srv::GraspingPrimitiveAggregatedAvailable>(actionInfoServiceName,
                                                   std::bind(&RosServiceHandler::primitiveAggregatedCallback, this, _1, _2));
                                                   
    _server_selectablePairInfo = _node->create_service<rosee_msg::srv::SelectablePairInfo>(selectablePairInfoServiceName,
                                                   std::bind(&RosServiceHandler::selectablePairInfoCallback, this, _1, _2));
                                                   
    _serverHandInfo = _node->create_service<rosee_msg::srv::HandInfo>(handInfoServiceName,
                                                   std::bind(&RosServiceHandler::handInfoCallback, this, _1, _2));
                                                   
    _serverNewGraspingAction = _node->create_service<rosee_msg::srv::NewGenericGraspingActionSrv>(newGraspingActionServiceName,
                                                   std::bind(&RosServiceHandler::newGraspingActionCallback, this, _1, _2));
    
    return true;
}

bool ROSEE::RosServiceHandler::graspingActionsCallback(
    const std::shared_ptr<rosee_msg::srv::GraspingActionsAvailable::Request> request,   
    std::shared_ptr<rosee_msg::srv::GraspingActionsAvailable::Response> response) {
    
    switch (request->action_type) {
        
        case 0 : { //PRIMITIVE
            
            //NOTE if both primitive type and action name are set in the request, the action name is not considered

            if (request->primitive_type == 0) { 
                
                if (request->action_name.size() == 0 ) {            
                    for (auto primitiveContainers : _mapActionHandler->getAllPrimitiveMaps() ) {
                        
                        //iterate all the primitive of a type
                        for (auto primitive : primitiveContainers.second) {
                            response->grasping_actions.push_back(fillGraspingActionMsg(primitive.second));
                        }
                    }
                    
                } else {
                    if (request->elements_involved.size() == 0) {
                        
                        for (auto primitive : _mapActionHandler->getPrimitiveMap(request->action_name)) {
                            response->grasping_actions.push_back(fillGraspingActionMsg(primitive.second));
                        }
                        
                    } else {
        
                        auto primitive =_mapActionHandler->getPrimitive(request->action_name, request->elements_involved);
                        response->grasping_actions.push_back(fillGraspingActionMsg(primitive));
                    }
                }
                
            } else {
               
                if (request->elements_involved.size() == 0) {
                        
                    //NOTE -1 because in the srv 0 is for all primitives, then the enum is scaled by one
                    for (auto primitives : _mapActionHandler->getPrimitiveMap(
                            static_cast<ROSEE::ActionPrimitive::Type>(request->primitive_type-1))) {
                        
                        for (auto primitive : primitives) {
                            response->grasping_actions.push_back(fillGraspingActionMsg(primitive.second));
                        }
                    }
                              
                } else {
                     
                    for (auto primitive : _mapActionHandler->getPrimitive(
                            static_cast<ROSEE::ActionPrimitive::Type>(request->primitive_type-1), request->elements_involved)) {
                        
                            response->grasping_actions.push_back(fillGraspingActionMsg(primitive));
                    }
                }
            }
            
            break;
        }
        
        case 1 : { //GENERIC_and_COMPOSED
            
            //NOTE for these some fields are ignored 
            if (request->action_name.size() == 0) {
                for (auto action : _mapActionHandler->getAllGenerics()) {
                    response->grasping_actions.push_back(fillGraspingActionMsg(action.second));
                }
                
            } else {
                response->grasping_actions.push_back(fillGraspingActionMsg(_mapActionHandler->getGeneric(request->action_name)));
            }
            break;
        }
        
        case 2 : { //TIMED
            if (request->action_name.size() == 0) {
                for (auto action : _mapActionHandler->getAllTimeds()) {
                    response->grasping_actions.push_back(fillGraspingActionMsg(action.second));
                }
                
            } else {
                response->grasping_actions.push_back(fillGraspingActionMsg(_mapActionHandler->getTimed(request->action_name)));
            }
            break;
        }
        
        default : {
            RCLCPP_ERROR_STREAM (_node->get_logger(), "[RosServiceHandler " << __func__ << " ] request->actionType can only be 0(ALL), 1(PRIMITIVE), "
                << "2(GENERIC_and_COMPOSED), or 3(TIMED); I have received " << request->action_type);
            return false;
            
        }
    }
    
    return true;    
}

rosee_msg::msg::GraspingAction ROSEE::RosServiceHandler::fillGraspingActionMsg(ROSEE::ActionPrimitive::Ptr primitive) {
    
    rosee_msg::msg::GraspingAction primitiveMsg;
    
    if (primitive == nullptr) {
        return primitiveMsg;
    }

    fillCommonInfoGraspingActionMsg(primitive, &primitiveMsg);

    primitiveMsg.primitive_type = primitive->getPrimitiveType();
    
    auto elements = primitive->getKeyElements();
    primitiveMsg.elements_involved.assign(elements.begin(), elements.end());
    
    return primitiveMsg;
}

rosee_msg::msg::GraspingAction ROSEE::RosServiceHandler::fillGraspingActionMsg(ROSEE::ActionGeneric::Ptr generic) {
    
    rosee_msg::msg::GraspingAction genericActionMsg;
    if (generic == nullptr) {
        return genericActionMsg;
    }
    
    fillCommonInfoGraspingActionMsg(generic, &genericActionMsg);

    genericActionMsg.primitive_type = genericActionMsg.PRIMITIVE_NONE;
    
    ActionComposed::Ptr composedCasted = std::dynamic_pointer_cast<ActionComposed>(generic);
    if ( composedCasted != nullptr) {
        genericActionMsg.inner_actions = composedCasted->getInnerActionsNames();
    }
    
    return genericActionMsg;
}

rosee_msg::msg::GraspingAction ROSEE::RosServiceHandler::fillGraspingActionMsg(ROSEE::ActionTimed::Ptr timed) {
    
    rosee_msg::msg::GraspingAction timedActionMsg;
    if (timed == nullptr) {
        return timedActionMsg;
    }
    
    fillCommonInfoGraspingActionMsg(timed, &timedActionMsg);

    timedActionMsg.primitive_type = timedActionMsg.PRIMITIVE_NONE;
    
    timedActionMsg.inner_actions = timed->getInnerActionsNames();

    for (auto innerMargin : timed->getAllActionMargins()){
        timedActionMsg.before_time_margins.push_back(innerMargin.first);
        timedActionMsg.after_time_margins.push_back(innerMargin.second);
    }
    
    return timedActionMsg;
}

void ROSEE::RosServiceHandler::fillCommonInfoGraspingActionMsg(ROSEE::Action::Ptr action, 
                                                               rosee_msg::msg::GraspingAction* graspingMsg) {
    
    graspingMsg->action_type = action->getType();
    graspingMsg->action_name = action->getName();
    
    rosee_msg::msg::JointsInvolvedCount motorCountMsg;
    for ( auto motorCount : action->getJointsInvolvedCount()) { 
        motorCountMsg.name.push_back(motorCount.first);
        motorCountMsg.count.push_back(motorCount.second);
    }
    graspingMsg->action_motor_count = motorCountMsg;
    
    for ( auto motorPosMultiple : action->getAllJointPos()) {

        //iterate over the single motor positions
        rosee_msg::msg::MotorPosition motorPosMsg;
        for ( auto motorPos : motorPosMultiple) { 
            motorPosMsg.name.push_back(motorPos.first);
            motorPosMsg.position.push_back(motorPos.second.at(0)); //at(0). because multiple dof is considered in general

        }
        graspingMsg->action_motor_positions.push_back(motorPosMsg); 
    }
    
    for (auto elementInvolved : action->getFingersInvolved()) {
        graspingMsg->elements_involved.push_back(elementInvolved);
    }
}

bool ROSEE::RosServiceHandler::primitiveAggregatedCallback(
    const std::shared_ptr<rosee_msg::srv::GraspingPrimitiveAggregatedAvailable::Request> request,
    std::shared_ptr<rosee_msg::srv::GraspingPrimitiveAggregatedAvailable::Response> response) {
    
     if (request->primitive_type == 0) { 
                
        if (request->action_name.size() == 0 ) {
            // return all primitives
                        
            for (auto primitiveMaps : _mapActionHandler->getAllPrimitiveMaps() ) {
                
                response->primitives_aggregated.push_back(fillPrimitiveAggregatedMsg(primitiveMaps.second));
            }
            
        } else {
            if (request->elements_involved.size() == 0) {
                
                auto primitiveMap = _mapActionHandler->getPrimitiveMap(request->action_name);                     
                response->primitives_aggregated.push_back(fillPrimitiveAggregatedMsg(primitiveMap));

                
            } else {

                auto primitive =_mapActionHandler->getPrimitive(request->action_name, request->elements_involved);
                response->primitives_aggregated.push_back(fillPrimitiveAggregatedMsg(primitive));
            }
        }
        
    } else {
        
        if (request->elements_involved.size() == 0) {
                
            //NOTE -1 because in the srv 0 is for all primitives, then the enum is scaled by one
            for (auto primitiveMap : _mapActionHandler->getPrimitiveMap(
                    static_cast<ROSEE::ActionPrimitive::Type>(request->primitive_type-1))) {
            
                response->primitives_aggregated.push_back(fillPrimitiveAggregatedMsg(primitiveMap));

            }
                        
        } else {
                
            for (auto primitive : _mapActionHandler->getPrimitive(
                    static_cast<ROSEE::ActionPrimitive::Type>(request->primitive_type-1), request->elements_involved)) {
                                    
                response->primitives_aggregated.push_back(fillPrimitiveAggregatedMsg(primitive));
            }
        }
    }
    return true;    
}


rosee_msg::msg::GraspingPrimitiveAggregated ROSEE::RosServiceHandler::fillPrimitiveAggregatedMsg(
    ROSEE::MapActionHandler::ActionPrimitiveMap primitiveMap) {
    
    rosee_msg::msg::GraspingPrimitiveAggregated primitiveMsg;

    primitiveMsg.action_name = primitiveMap.begin()->second->getName();
    primitiveMsg.primitive_type = primitiveMap.begin()->second->getPrimitiveType();
    //until now, there is not a primitive that does not have "something" to select
    // (eg pinch has 2 fing, trig one fing, singleJointMultipleTips 1 joint...). 
    //Instead generic action has always no thing to select (next for loop)
    primitiveMsg.max_selectable = primitiveMap.begin()->first.size();
    //TODO extract the keys with another mapActionHandler function?
    primitiveMsg.selectable_names =
        ROSEE::Utils::extract_keys_merged(primitiveMap, nFinger);

    return primitiveMsg;
}

rosee_msg::msg::GraspingPrimitiveAggregated ROSEE::RosServiceHandler::fillPrimitiveAggregatedMsg(
    ROSEE::ActionPrimitive::Ptr primitive) {

    rosee_msg::msg::GraspingPrimitiveAggregated primitiveMsg;
    primitiveMsg.action_name = primitive->getName();
    primitiveMsg.primitive_type = primitive->getPrimitiveType();
    
    auto elements = primitive->getKeyElements();
    primitiveMsg.max_selectable = elements.size();
    primitiveMsg.selectable_names.assign(elements.begin(), elements.end());

    return primitiveMsg;
}


bool ROSEE::RosServiceHandler::selectablePairInfoCallback(
    const std::shared_ptr<rosee_msg::srv::SelectablePairInfo::Request> request,
    std::shared_ptr<rosee_msg::srv::SelectablePairInfo::Response> response) 
{
    
    std::set<std::string> companionFingers;
    if (request->action_name.compare ("pinchTight") == 0) {
        companionFingers =
            _mapActionHandler->getFingertipsForPinch(request->element_name,
                ROSEE::ActionPrimitive::Type::PinchTight) ;
                
    } else if (request->action_name.compare ("pinchLoose") == 0) {
        companionFingers =
            _mapActionHandler->getFingertipsForPinch(request->element_name,
                ROSEE::ActionPrimitive::Type::PinchLoose) ;
                
    } else {
        RCLCPP_ERROR_STREAM (_node->get_logger(), "Received" << request->action_name << " that is not" <<
            "a recognizible action name to look for finger companions" );
        return false;
    }
    
    if (companionFingers.size() == 0) {
        return false;
    }
    
    //push the elements of set into the vector
    for (auto fing : companionFingers ) {
        response->pair_elements.push_back (fing);
    }
    
    return true;        
}

bool ROSEE::RosServiceHandler::handInfoCallback(        
    const std::shared_ptr<rosee_msg::srv::HandInfo::Request> request,
    std::shared_ptr<rosee_msg::srv::HandInfo::Response> response) 
{

//     if (! ros::service::exists("/EEHalExecutor/hand_info", false) ) {
//         return false;
//     }
    
    rclcpp::Client<rosee_msg::srv::HandInfo>::SharedPtr handInfoClient = 
        _node->create_client<rosee_msg::srv::HandInfo>("/EEHalExecutor/hand_info");
        
    auto result = handInfoClient->async_send_request(request);
    if (rclcpp::spin_until_future_complete(_node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        response = result.get();
        
    } else {
        return false;
    }
    
    return true;
}
        
//TODO error msg useless becaus if return false the response is not send
//at today (2020) it seems there not exist a method to return false plus an error message.
//NOTE : this was a note regarding ros1.
bool ROSEE::RosServiceHandler::newGraspingActionCallback(        
    const std::shared_ptr<rosee_msg::srv::NewGenericGraspingActionSrv::Request> request,
    std::shared_ptr<rosee_msg::srv::NewGenericGraspingActionSrv::Response> response){
    
    response->accepted = false;
    response->emitted = false;
    
    if (request->new_action.action_name.empty()) {
        
        response->error_msg = "action_name can not be empty";
        RCLCPP_ERROR_STREAM (_node->get_logger(), "[RosServiceHandler " << __func__ << " ] " << response->error_msg);
        return true; //so the client receive the response
    }
    
    if (request->new_action.action_motor_position.name.size() == 0 ||
        request->new_action.action_motor_position.position.size() == 0 ||
        request->new_action.action_motor_position.position.size() != request->new_action.action_motor_position.name.size()) {
        
        response->error_msg = "action_motor_position is empty or badly formed";
        RCLCPP_ERROR_STREAM (_node->get_logger(), "[RosServiceHandler " << __func__ << " ] " << response->error_msg);

        return true; //so the client receive the response
    }
    
    if (request->new_action.action_motor_count.name.size() != request->new_action.action_motor_count.count.size()) {
        
        response->error_msg = "action_motor_count is badly formed, name and count have different sizes";
        RCLCPP_ERROR_STREAM (_node->get_logger(), "[RosServiceHandler " << __func__ << " ] " << response->error_msg);

        return true; //so the client receive the response
    }
    
    // TODO request->new_action.action_motor_count : if empty, ActionGeneric costructor will consider all joint
    // with 0 position as not used. This may change in future when we will support not 0 default joint positions
    
    if (_mapActionHandler->getGeneric(request->new_action.action_name, false) != nullptr) {
        
        response->error_msg = "A generic action with name '" + request->new_action.action_name + "' already exists";
        RCLCPP_ERROR_STREAM (_node->get_logger(), "[RosServiceHandler " << __func__ << " ] " << response->error_msg);

        return true; //so the client receive the response
        
    }

    ROSEE::ActionGeneric::Ptr newAction;
    ROSEE::JointPos jp;
    ROSEE::JointsInvolvedCount jic;
    std::set<std::string> elementInvolved;
    
    for (size_t i = 0; i < request->new_action.action_motor_position.name.size(); i++){
        
        std::vector<double> one_dof{request->new_action.action_motor_position.position.at(i)};        
        jp.insert(std::make_pair(request->new_action.action_motor_position.name.at(i),
                                 one_dof));
    }
    
    for (size_t i = 0; i < request->new_action.action_motor_count.name.size(); i++){
        
        jic.insert(std::make_pair(request->new_action.action_motor_count.name.at(i),
                                  request->new_action.action_motor_count.count.at(i)));
    }
    
    for (size_t i = 0; i< request->new_action.elements_involved.size(); i++) {
        
        elementInvolved.insert(request->new_action.elements_involved.at(i));  
    }
    
    //costructor will handle jpc and elementInvolved also if empty
    try { 
        newAction = std::make_shared<ROSEE::ActionGeneric>(request->new_action.action_name,
                                                             jp,
                                                             jic,
                                                             elementInvolved);
    
    } catch (const ROSEE::Utils::DifferentKeysException<ROSEE::JointPos, ROSEE::JointsInvolvedCount>&) {
        
        response->error_msg = "action_motor_position and action_motor_count have different names element. They must be the same because they refer to actuator of the end-effector";
        RCLCPP_ERROR_STREAM (_node->get_logger(), "[RosServiceHandler " << __func__ << " ] " << response->error_msg);

        return true; //so the client receive the response
    } 
    
    //u rosee main node use always mapActionHandler to check if an action exists. Thus, we need to add this new 
    // action into the mapActionHandler "database" (ie private member map of the generic actions)
    if (! _mapActionHandler->insertSingleGeneric(newAction)){
        
        response->error_msg = "error by mapActionHandler when inserting the new generic action";
        RCLCPP_ERROR_STREAM (_node->get_logger(), "[RosServiceHandler " << __func__ << " ] " << response->error_msg);

        return true; //so the client receive the response
    }
    
    RCLCPP_INFO_STREAM (_node->get_logger(), "[RosServiceHandler " << __func__ << " ] The new action '"<< newAction->getName() << "' is inserted in the system");

    
    if (request->emit_yaml) {
        
        ROSEE::YamlWorker yamlWorker;
        auto path = yamlWorker.createYamlFile(newAction, _path2saveYamlGeneric);
        RCLCPP_INFO_STREAM (_node->get_logger(), "[RosServiceHandler " << __func__ << " ] The received new action '"<< newAction->getName() << "' has been stored in " << path);
        response->emitted = true;
    }
    
    response->accepted = true;
    return true;
}
