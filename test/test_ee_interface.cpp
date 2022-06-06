#include <gtest/gtest.h>
#include "testUtils.h"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <end_effector/Parser.h>
#include <end_effector/EEInterface.h>

namespace {

class testEEInterface: public ::testing::Test {


protected:

    testEEInterface() {
    }

    virtual ~testEEInterface() {
    }
    
    virtual void SetUp() override {
        
    }

    virtual void SetUp(int argc, char **argv) {
    
        node = ROSEE::TestUtils::prepareROSForTests ( argc, argv, "testEEInterface");
        
        ASSERT_NE(node, nullptr);

        ROSEE::Parser p ( node );
        p.init ( ament_index_cpp::get_package_share_directory("end_effector") + "/configs/urdf/test_ee.urdf",
                 ament_index_cpp::get_package_share_directory("end_effector") + "/configs/srdf/test_ee.srdf",
                 "ROSEE/actions/test_ee"
        );
        p.printEndEffectorFingerJointsMap();

        ee = std::make_shared<ROSEE::EEInterface>(p);
    }

    virtual void TearDown() {
    }

    ROSEE::EEInterface::Ptr ee;
    rclcpp::Node::SharedPtr node;
};


TEST_F ( testEEInterface, checkFingers ) {

    SetUp(argc_g, argv_g);
    
    std::vector<std::string> fingers;
    fingers = ee->getFingers();

    EXPECT_FALSE (fingers.empty());

    RCLCPP_INFO_STREAM (node->get_logger(), "Fingers in EEInterface: " );
    for ( auto& f : fingers ) {
        RCLCPP_INFO_STREAM (node->get_logger(), f );
    }

    EXPECT_TRUE ( ee->isFinger ( "finger_1" ) );

    EXPECT_FALSE ( ee->isFinger ( "finger_4" ) );



}

TEST_F ( testEEInterface, checkActuatedJointsNum ) {

    SetUp(argc_g, argv_g);
    
    int joint_num = ee->getActuatedJointsNum();
    EXPECT_EQ ( 6, joint_num );

    EXPECT_FALSE ( joint_num < 0 );

}

TEST_F ( testEEInterface, checkEEFingerJoints ) {

    SetUp(argc_g, argv_g);
    
    int joint_num_finger1 = ee->getActuatedJointsNumInFinger("finger_1");

    int joint_num = ee->getActuatedJointsNum();

    EXPECT_TRUE ( joint_num >= joint_num_finger1 );

    int joint_num_counter = 0;
    std::vector<std::string> fingers = ee->getFingers();



    for ( auto& f : fingers ) {
        joint_num_counter += ee->getActuatedJointsNumInFinger(f);
    }

    EXPECT_TRUE ( joint_num == joint_num_counter );

}

TEST_F ( testEEInterface, checkJointLimits) {

    SetUp(argc_g, argv_g);
    
    Eigen::VectorXd upperLimits = ee->getUpperPositionLimits();
    Eigen::VectorXd lowerLimits = ee->getLowerPositionLimits();
    
    ASSERT_EQ (upperLimits.size(), lowerLimits.size()); //stop if it fails here
    
    EXPECT_TRUE (upperLimits.size() > 0);
    
    for (int i=0; i<upperLimits.size(); i++) {
    
        EXPECT_GE (upperLimits(i), lowerLimits(i)); //greater or equal than
        RCLCPP_INFO_STREAM (node->get_logger(), "Joint " << std::to_string(i) << " limits:  " <<
                          upperLimits(i) <<  ", " << lowerLimits(i) );

    }
    
}

TEST_F ( testEEInterface, checkIdJoints ) {

    SetUp(argc_g, argv_g);
    
    std::vector<std::string> actJoints = ee->getActuatedJoints();
   // ASSERT_FALSE (actJoints.empty());  //a hand can have no actuated joints?
    
    // check if ids are unique
    int id = -1;
    int idPrevious = -1;
    for ( auto& j : actJoints ) {
        EXPECT_TRUE ( ee->getInternalIdForJoint (j, id) ); //return false if joint does not exist
        EXPECT_NE ( id, idPrevious );
        idPrevious = id;
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
