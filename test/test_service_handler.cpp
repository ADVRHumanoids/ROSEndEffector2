#include <gtest/gtest.h>
#include "testUtils.h"

#include <rclcpp/rclcpp.hpp>

#include <end_effector/Parser.h>
#include <end_effector/EEInterface.h>

#include <end_effector/RosServiceHandler.h>
#include <end_effector/MapActionHandler.h>

#include <rosee_msg/srv/new_generic_grasping_action_srv.hpp>

#include <chrono>
using namespace std::literals::chrono_literals;


namespace {

/**
 * @brief Test for the RosServiceHandler Class
 * It simply create the server and some clients which will test the server functions.
 * Hence, in this test a lot of ROSEE main class are not used like the FindAction
 */
class testServiceHandler: public ::testing::Test {


protected:

    testServiceHandler()  {
    }

    virtual ~testServiceHandler() {
    }
    
    virtual void SetUp() override {
        
    }

    virtual void SetUp(int argc, char **argv) {
        
    
        node = ROSEE::TestUtils::prepareROSForTests ( argc, argv, "testServiceHandler");

        ASSERT_NE(node, nullptr);

        std::string robot_name = argv[1];
        
        setenv("HAND_NAME",robot_name.c_str(),1);

        roseeExecutor.reset(new ROSEE::TestUtils::Process({"ros2", "launch", "end_effector", "test_rosee_startup_launch.py"}));
        
    }

    virtual void TearDown() override {
    }

    std::string robot_name;
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<ROSEE::TestUtils::Process> roseeExecutor;

    template <class clientType>
    bool initClient( std::shared_ptr<rclcpp::Client<clientType>> &rosee_client, std::string serviceName) {

        rosee_client = node->create_client<clientType>(serviceName);

        RCLCPP_INFO(node->get_logger(), "Waiting for '%s' service for max 5 seconds...", serviceName.c_str());
        if (!rosee_client->wait_for_service(5s)) {
            RCLCPP_INFO(node->get_logger(), "Service'%s' not ready after seconds", serviceName.c_str());
            return false;
        }
        RCLCPP_INFO(node->get_logger(), "Service '%s' ready", serviceName.c_str());

        return true;

    };

};

/**
 * We call the service to include a new generic action multiple time, with both wrong and fallace requests, to see if the errors are 
 * detected by the server, and correct action are accepted
 */
TEST_F ( testServiceHandler, callNewAction ) {

    SetUp(argc_g, argv_g);
    
    sleep(1); //without this joint state publisher crashes I do not know why (it is useless in this test, but annoying prints on traceback would appear)

    rclcpp::Client<rosee_msg::srv::NewGenericGraspingActionSrv>::SharedPtr rosee_client; 
    ASSERT_TRUE(initClient<rosee_msg::srv::NewGenericGraspingActionSrv>(rosee_client, "/new_generic_grasping_action"));

    auto newActionSrv = std::make_shared<rosee_msg::srv::NewGenericGraspingActionSrv::Request>();

    //empty request, error
    auto result = rosee_client->async_send_request(newActionSrv);
    std::cout << rclcpp::spin_until_future_complete(node, result) << std::endl;
    //EXPECT_EQ( rclcpp::spin_until_future_complete(node, result), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE( result.get()->accepted );
    EXPECT_FALSE( result.get()->emitted );
    EXPECT_TRUE( result.get()->error_msg.size() > 0 );
    
    //empty joint pos, error
    newActionSrv->new_action.action_name = "newAction";
    result = rosee_client->async_send_request(newActionSrv);
    EXPECT_EQ( rclcpp::spin_until_future_complete(node, result), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE( result.get()->accepted );
    EXPECT_FALSE( result.get()->emitted );
    EXPECT_TRUE( result.get()->error_msg.size() > 0 );

    //correct request
    newActionSrv->new_action.action_motor_position.name.push_back("joint_1");
    newActionSrv->new_action.action_motor_position.position.push_back(1);
    result = rosee_client->async_send_request(newActionSrv);
    EXPECT_EQ( rclcpp::spin_until_future_complete(node, result), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE( result.get()->accepted );
    EXPECT_FALSE( result.get()->emitted );
    EXPECT_FALSE( result.get()->error_msg.size() > 0  );

    //same request, this time is must fail because a "test1" action is already present
    result = rosee_client->async_send_request(newActionSrv);
    EXPECT_EQ( rclcpp::spin_until_future_complete(node, result), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE( result.get()->accepted );
    EXPECT_FALSE(result.get()->emitted );
    EXPECT_TRUE( result.get()->error_msg.size() > 0 );
    
    //  error, the joint names in motor pos and count  are not the same
    //NOTE we use the time so if this test is run multiple time on same machine, the action will have a different name
    newActionSrv->new_action.action_name = "newAction_" + std::to_string(rclcpp::Clock().now().seconds());
    newActionSrv->new_action.action_motor_count.name.push_back("error_joint_1");
    newActionSrv->new_action.action_motor_count.count.push_back(1);
    result = rosee_client->async_send_request(newActionSrv);
    EXPECT_EQ( rclcpp::spin_until_future_complete(node, result), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE(  result.get()->accepted  );
    EXPECT_FALSE( result.get()->emitted );
    EXPECT_TRUE( result.get()->error_msg.size() > 0 );
    
    //  correcting previous error, also emitting on yaml
    newActionSrv->new_action.action_motor_count.name.at(0) = "joint_1";
    newActionSrv->new_action.action_motor_count.count.at(0) = 1;
    newActionSrv->emit_yaml = true;
    result = rosee_client->async_send_request(newActionSrv);
    EXPECT_EQ( rclcpp::spin_until_future_complete(node, result), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE( result.get()->accepted );
    EXPECT_TRUE( result.get()->emitted );
    EXPECT_FALSE( result.get()->error_msg.size() > 0 );
    
}

/***
 * Here we send some new action to newGraspingActionServer, and we see if we can retrieve them with another server, GraspingActionAvailable
 */
TEST_F ( testServiceHandler, callNewActionAndRetrieve ) {
    
    SetUp(argc_g, argv_g);
    
    sleep(1); //without this joint state publisher crashes I do not know why (it is useless in this test, but annoying prints on traceback would appear)
    
    rclcpp::Client<rosee_msg::srv::NewGenericGraspingActionSrv>::SharedPtr rosee_client_new_action; 
    rclcpp::Client<rosee_msg::srv::GraspingActionsAvailable>::SharedPtr rosee_client_actions_available; 

    initClient<rosee_msg::srv::NewGenericGraspingActionSrv>(rosee_client_new_action, "/new_generic_grasping_action");
    initClient<rosee_msg::srv::GraspingActionsAvailable>(rosee_client_actions_available, "/grasping_actions_available");

    auto newActionSrv = std::make_shared<rosee_msg::srv::NewGenericGraspingActionSrv::Request>();
    
    std::string requestActionName = "newAction_TEST";
    newActionSrv->new_action.action_name = requestActionName;
    newActionSrv->new_action.action_motor_position.name.push_back("joint_1");
    newActionSrv->new_action.action_motor_position.position.push_back(1);
    newActionSrv->new_action.elements_involved.push_back("an_element");
    newActionSrv->emit_yaml = false;
    auto result = rosee_client_new_action->async_send_request(newActionSrv);
    EXPECT_EQ( rclcpp::spin_until_future_complete(node, result), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE( result.get()->accepted  );
    EXPECT_FALSE( result.get()->emitted );
    EXPECT_FALSE( result.get()->error_msg.size() > 0 );
    
    auto graspActionAvailable = std::make_shared<rosee_msg::srv::GraspingActionsAvailable::Request>();
    
    //we ask for a not existing action, and check for the error
    graspActionAvailable->action_name = "newAction_TEST_NOT_EXISTENT";
    auto result2 = rosee_client_actions_available->async_send_request(graspActionAvailable);
    EXPECT_EQ( rclcpp::spin_until_future_complete(node, result2), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result2.get()->grasping_actions.size(), 0);
    
    //we ask for the requestActionName, and check if fields are the same sent by us
    graspActionAvailable->action_name = requestActionName;
    //Compulsory field, otherwise primitive is asked
    graspActionAvailable->action_type = 1;
    result2 = rosee_client_actions_available->async_send_request(graspActionAvailable);
    EXPECT_EQ( rclcpp::spin_until_future_complete(node, result2), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(1, result2.get()->grasping_actions.size());
    if (result2.get()->grasping_actions.size() > 0) {
        auto receivedAction = result2.get()->grasping_actions.at(0);
        EXPECT_EQ(receivedAction.action_name, requestActionName);
        EXPECT_EQ(receivedAction.action_motor_positions.at(0).name.size(),  newActionSrv->new_action.action_motor_position.name.size());
        EXPECT_EQ(receivedAction.action_motor_positions.at(0).name.at(0),  newActionSrv->new_action.action_motor_position.name.at(0));
        EXPECT_EQ(receivedAction.action_motor_positions.at(0).position.at(0),  newActionSrv->new_action.action_motor_position.position.at(0));
        //motor count was not filled here but it is filled by default by action costructor
        EXPECT_EQ(1, receivedAction.action_motor_count.name.size());
        EXPECT_EQ(receivedAction.action_motor_count.name.at(0), newActionSrv->new_action.action_motor_position.name.at(0));
        EXPECT_EQ(1, receivedAction.action_motor_count.count.at(0));
        EXPECT_EQ(1, receivedAction.elements_involved.size());
        if (receivedAction.elements_involved.size() > 0){
            EXPECT_EQ(receivedAction.elements_involved.at(0), newActionSrv->new_action.elements_involved.at(0));
        }
    }  
}



} //namespace

int main ( int argc, char **argv ) {
    
    if (argc < 2 ){
        
        std::cout << "[TEST ERROR] Insert hand name as argument" << std::endl;
        return -1;
    }
    
    rclcpp::init ( argc, argv );
    
    ::testing::InitGoogleTest ( &argc, argv );
    ::testing::AddGlobalTestEnvironment(new MyTestEnvironment(argc, argv));

    return RUN_ALL_TESTS();
}
