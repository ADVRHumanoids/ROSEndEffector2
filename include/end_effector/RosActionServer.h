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

#ifndef ROSACTIONSERVER_H
#define ROSACTIONSERVER_H

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosee_msg/action/rosee_command.hpp>

#define DEFAULT_ERROR_NORM 0.01

namespace ROSEE {
/**
 * @todo write docs
 */
class RosActionServer {
    
public:
    RosActionServer(std::string topicForAction, rclcpp::Node::SharedPtr node);
    ~RosActionServer() {};
    
    rosee_msg::msg::ROSEEActionControl getGoal();
    bool hasGoal() const;
    bool hasNewGoal() const;
    
    /**
     * @brief in message there is a field where norm error of joint position can be set
     *   If it is not present in the message sent (thus it is 0) the DEFAULT_ERROR_NORM is used
     */
    double getWantedNormError() const;

    /** 
     * @brief send Feedback to the client who has requested the goal.
     * 
     * @param completation_percentage the percentae that tells how much we have completed 
     * the action requested
     * @param currentAction current action that is running. If it is empty, it is taken the name
     *    of the action from member \ref actionControlMsg . You should passed not empty string when
     *    dealing with timed action (passing the inner action of the timed one)
     */
    void sendFeedback(double completation_percentage, std::string currentAction) ;
    void sendComplete () ;
    void abortGoal(std::string errorMsg = "");

    
protected:
    rclcpp::Node::SharedPtr _node;
    std::string topicForAction;
    rclcpp_action::Server<rosee_msg::action::ROSEECommand>::SharedPtr _actionServer;
    rosee_msg::action::ROSEECommand::Goal actual_goal;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rosee_msg::action::ROSEECommand>> actual_goal_handle;
    bool goalInExecution;
    bool newGoal;
    double wantedNormError;
    
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const rosee_msg::action::ROSEECommand::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<rosee_msg::action::ROSEECommand>> goal_handle);
    
    
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<rosee_msg::action::ROSEECommand>> goal_handle);


};

}

#endif // ROSACTIONSERVER_H
