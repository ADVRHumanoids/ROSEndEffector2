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

#include <end_effector/RosActionServer.h>

ROSEE::RosActionServer::RosActionServer (std::string topicForAction, rclcpp::Node::SharedPtr node) {
    
    this->_node = node;
    this->topicForAction = topicForAction;
    goalInExecution = false;
    newGoal = false;
    
    using namespace std::placeholders;

    _actionServer = rclcpp_action::create_server<rosee_msg::action::ROSEECommand>(
        _node,
        topicForAction,
        std::bind(&RosActionServer::handle_goal, this, _1, _2),
        std::bind(&RosActionServer::handle_cancel, this, _1),
        std::bind(&RosActionServer::handle_accepted, this, _1));
    
}

bool ROSEE::RosActionServer::hasGoal () const {
    return goalInExecution;
}

bool ROSEE::RosActionServer::hasNewGoal () const {
    return newGoal;
}

double ROSEE::RosActionServer::getWantedNormError() const {
    return wantedNormError;
}

rosee_msg::msg::ROSEEActionControl ROSEE::RosActionServer::getGoal() {
    
    if (goalInExecution) {
        newGoal = false;
        return actual_goal.goal_action;
    } else {
        RCLCPP_WARN_STREAM (_node->get_logger(),"ROSACTION SERVER no goal is in execution (no one has" <<
            "arrived or the last one is already completed. EMPTY GOAL RETURNING");
        return rosee_msg::msg::ROSEEActionControl();
    }
}


rclcpp_action::GoalResponse ROSEE::RosActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const rosee_msg::action::ROSEECommand::Goal> goal) {
    
    //RCLCPP_INFO_STREAM (_node->get_logger(),goal_action.
    goalInExecution = true;
    newGoal = true;
    this->actual_goal.goal_action = goal->goal_action;
    
    wantedNormError = actual_goal.goal_action.error_norm == 0 ? DEFAULT_ERROR_NORM : actual_goal.goal_action.error_norm;

    RCLCPP_INFO (_node->get_logger(),"ROSACTION SERVER received goal: '%s'", this->actual_goal.goal_action.action_name.c_str());
    (void)uuid;
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ROSEE::RosActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<rosee_msg::action::ROSEECommand>> goal_handle) 
{
    
    RCLCPP_INFO(_node->get_logger(), "Received request to cancel goal");
    goalInExecution = false;
    newGoal = false;
    wantedNormError = DEFAULT_ERROR_NORM;
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
    
}

void ROSEE::RosActionServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<rosee_msg::action::ROSEECommand>> goal_handle)
{
    //We execute the goal in the universalrosendeffector class
    actual_goal_handle = goal_handle;
}

void ROSEE::RosActionServer::abortGoal(std::string errorMsg) {
    
    if (goalInExecution) {
        RCLCPP_INFO(_node->get_logger(), "ROSACTION SERVER Aborted goal");
        std::shared_ptr<rosee_msg::action::ROSEECommand::Result> result = 
            std::make_shared<rosee_msg::action::ROSEECommand::Result>();
        
        result->completed_action = actual_goal.goal_action;

        actual_goal_handle->abort(result);
        newGoal = false;
        goalInExecution = false;
        wantedNormError = DEFAULT_ERROR_NORM;
    }

}

void ROSEE::RosActionServer::sendFeedback(double completation_percentage, std::string currentAction) {
    
    //RCLCPP_INFO(_node->get_logger(), "ROSACTION SERVER Sending feedback " << completation_percentage );
    if (goalInExecution) {
        std::shared_ptr<rosee_msg::action::ROSEECommand::Feedback> feedback = std::make_shared<rosee_msg::action::ROSEECommand::Feedback>();
        feedback->completation_percentage = completation_percentage;
        if (currentAction.size() == 0) {
            feedback->action_name_current = actual_goal.goal_action.action_name;
        } else {
            feedback->action_name_current = currentAction;
        }
        
        actual_goal_handle->publish_feedback(feedback);
    }
    
}

void ROSEE::RosActionServer::sendComplete ()  {
    
    RCLCPP_INFO(_node->get_logger(), "ROSACTION SERVER Sending final result completed ");
    goalInExecution = false;
    newGoal = false; //even if here should be already false
    wantedNormError = DEFAULT_ERROR_NORM;

    std::shared_ptr<rosee_msg::action::ROSEECommand::Result> result = 
        std::make_shared<rosee_msg::action::ROSEECommand::Result>();
        
    result->completed_action = actual_goal.goal_action;
    
    actual_goal_handle->succeed(result);

}

