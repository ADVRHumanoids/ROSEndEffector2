#include <gtest/gtest.h>
#include "testUtils.h"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <end_effector/Parser.h>
#include <end_effector/EEInterface.h>
#include <end_effector/Utils.h>

namespace {

class testEEInterface: public ::testing::Test {


protected:

    testEEInterface() {
    }

    virtual ~testEEInterface() {
    }

    virtual void SetUp() {



        ROSEE::Parser p ( node );
        p.init ( ROSEE::Utils::getPackagePath() + "/configs/urdf/test_ee.urdf",
                 ROSEE::Utils::getPackagePath() + "/configs/srdf/test_ee.srdf",
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

    int joint_num = ee->getActuatedJointsNum();
    EXPECT_EQ ( 6, joint_num );

    EXPECT_FALSE ( joint_num < 0 );

}

TEST_F ( testEEInterface, checkEEFingerJoints ) {

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

    if (argc < 2 ) {

        std::cout << "[TEST ERROR] Insert hand name as argument" << std::endl;
        return -1;
    }

    if ( ROSEE::TestUtils::prepareROSForTests ( argc, argv, "testEEInterface" ) != 0 ) {

        std::cout << "[TEST ERROR] Prepare Funcion failed" << std::endl;
        return -1;
    }

    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
