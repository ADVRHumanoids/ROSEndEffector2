#include <gtest/gtest.h>
#include "testUtils.h"

#include <rclcpp/rclcpp.hpp>

#include <end_effector/Parser.h>
#include <end_effector/ParserMoveIt.h>
#include <end_effector/FindActions.h>
#include <end_effector/EEInterface.h>
#include <end_effector/GraspingActions/ActionGeneric.h>
#include <end_effector/Utils.h>
#include <end_effector/YamlWorker.h>

#include <rosee_msg/action/rosee_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <random>

using std::placeholders::_1;
using std::placeholders::_2;

namespace {

/**
 * @brief This Tests are to check if actions are sent correctly. Check each TEST_F for better explanation
 * 
 * @warning @todo There are a lot of prints that seems to be not in the right order. Probably is because we use 
 *   the ROS STREAM that effectively prints only when the spin is called...
 *   BTW it is strange that the parser prints are printed at the end, and not while setup function is running (where they
 *   should be because we init the parser there. This does not cause problems at the test.
 */
class testSendAction: public ::testing::Test {

    

protected:

    testSendAction()  {
    }

    virtual ~testSendAction() {
    }
    
    virtual void SetUp() override {
        
    }

    virtual void SetUp(int argc, char **argv) {
    
        node = ROSEE::TestUtils::prepareROSForTests ( argc, argv, "testSendAction");
        
        ASSERT_NE(node, nullptr);
                
        std::string robot_name = argv[1];
        
        ROSEE::Parser p ( node );
        if (! p.init ( ament_index_cpp::get_package_share_directory("end_effector") + "/configs/urdf/" + robot_name + ".urdf",
                       ament_index_cpp::get_package_share_directory("end_effector") + "/configs/srdf/" + robot_name + ".srdf",
                       std::string(std::getenv("HOME")) + "/.ROSEE2/actions/"+robot_name+ "/") ) 
        {
            
            std::cout << "[TEST SEND ACTIONS] parser FAIL: some config file missing]" << std::endl;
            return;
        }
        
        ee = std::make_shared<ROSEE::EEInterface>(p);
        
        folderForActions = p.getActionPath();
        if ( folderForActions.size() == 0 ){ //if no action path is set in the yaml file...
            std::cout << "[TEST SEND ACTIONS] parser FAIL: action_path in the config file is missing" << std::endl;
            return;
        }

        //note: test on the findActions part is done in other test files
        parserMoveIt = std::make_shared<ROSEE::ParserMoveIt>(node);
        if (! parserMoveIt->init ("robot_description", false) ) {

            std::cout << "[TEST SEND ACTIONS] FAILED parserMoveit Init, stopping execution"  << std::endl; 
            return;
        }
                
        //note: calls to find*** (like findTrig) are done in the specific testu
        actionsFinder = std::make_shared<ROSEE::FindActions>(parserMoveIt);

                                                                          
        }

    virtual void TearDown() override {
    }
    
    void sendAndTest(ROSEE::Action::Ptr action, double percentageWanted = 1.0);

    rclcpp::Node::SharedPtr node;
    std::string folderForActions;
    std::unique_ptr<ROSEE::TestUtils::Process> roseeExecutor;
    ROSEE::EEInterface::Ptr ee;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr receiveRobStateSub;
    ROSEE::YamlWorker yamlWorker;
    rclcpp_action::Client<rosee_msg::action::ROSEECommand>::SharedPtr action_client;
    ROSEE::ParserMoveIt::Ptr parserMoveIt;
    std::shared_ptr<ROSEE::FindActions> actionsFinder;
    
    struct ClbkHelper {
    
        ClbkHelper() : js(), completed(false) {};
    
        void jointStateClbk(const sensor_msgs::msg::JointState::SharedPtr msg) {
            
            js.name = msg->name;
            js.position = msg->position;
            js.velocity = msg->velocity;
            js.effort = msg->effort;
        }

        void actionDoneClbk(const rclcpp_action::ClientGoalHandle<rosee_msg::action::ROSEECommand>::WrappedResult & result) { 
            
            completed = true; 
            goalState = result.code;
            actionCompleted = result.result->completed_action;
        }

        void actionFeedbackClbk(rclcpp_action::ClientGoalHandle<rosee_msg::action::ROSEECommand>::SharedPtr,
                                const std::shared_ptr<const rosee_msg::action::ROSEECommand::Feedback> feedback) {
            
            feedback_percentage = feedback->completation_percentage;   
        }

        void actionActiveClbk(std::shared_future<rclcpp_action::ClientGoalHandle<rosee_msg::action::ROSEECommand>::SharedPtr> future) {
            
            completed = false; 
        }
        
        sensor_msgs::msg::JointState js;
        bool completed;
        double feedback_percentage;
        rosee_msg::msg::ROSEEActionControl actionCompleted;
        rclcpp_action::ResultCode goalState; 
        
            
    };

    ClbkHelper clbkHelper;
    
    void setMainNode();
    void sendAction( ROSEE::Action::Ptr action, double percentageWanted);
    void testAction( ROSEE::Action::Ptr actionSent, double percentageWanted);
    
};

void testSendAction::setMainNode() {
    
    setenv("HAND_NAME",ee->getName().c_str(),1);

    roseeExecutor.reset(new ROSEE::TestUtils::Process({"ros2", "launch", "end_effector", "test_rosee_startup_launch.py"}));

    //TODO put a checkReady service instead of sleeping?
    sleep(5); // lets wait for test_rosee_startup to be ready
    std::string topic_name_js = "/joint_states";    
    
    receiveRobStateSub = node->create_subscription<sensor_msgs::msg::JointState>(topic_name_js, 1, std::bind(&ClbkHelper::jointStateClbk, &clbkHelper, _1));
    
    action_client = 
        rclcpp_action::create_client<rosee_msg::action::ROSEECommand>(node, "/action_command");

    action_client->wait_for_action_server();
        
}
    
void testSendAction::sendAction( ROSEE::Action::Ptr action, double percentageWanted) {

    rosee_msg::action::ROSEECommand::Goal goal;
    goal.goal_action.seq = 0 ;
    goal.goal_action.stamp = rclcpp::Clock().now();
    goal.goal_action.percentage = percentageWanted;
    goal.goal_action.action_name = action->getName();
    goal.goal_action.action_type = action->getType() ;

    if (action->getType() == ROSEE::Action::Type::Primitive) {
        ROSEE::ActionPrimitive::Ptr primitivePtr = std::static_pointer_cast<ROSEE::ActionPrimitive>(action);
        goal.goal_action.action_primitive_type = primitivePtr->getPrimitiveType() ;

        for (auto it : primitivePtr->getKeyElements()) {
            goal.goal_action.selectable_items.push_back(it);
        }
    }

    auto send_goal_options = rclcpp_action::Client<rosee_msg::action::ROSEECommand>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ClbkHelper::actionActiveClbk, &clbkHelper, _1);
    send_goal_options.feedback_callback =
      std::bind(&ClbkHelper::actionFeedbackClbk, &clbkHelper, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ClbkHelper::actionDoneClbk, &clbkHelper, _1);
      
    action_client->async_send_goal(goal, send_goal_options);

}

void testSendAction::testAction(ROSEE::Action::Ptr actionSent, double percentageWanted) {
         
    rclcpp::Rate r(10);
    while (!clbkHelper.completed) {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "Test Send Actions: Waiting for action completion...");
        rclcpp::spin_some(node);
        r.sleep();
    }
    
    //lets wait one sec more so subscriber for joint state receive the very last state
    rclcpp::Rate r2(1);
    r2.sleep();
    rclcpp::spin_some(node);

    
    //finally, lets test if the pos set in the actions are the same of the robot when the action is completed
    for (int i=0; i < clbkHelper.js.name.size(); i++) {
        
        auto wantedJointsPosMap = actionSent->getJointPos();
        
        auto findJoint = wantedJointsPosMap.find(clbkHelper.js.name[i]);
        
        if (findJoint == wantedJointsPosMap.end()){
            continue; //this is not an error, in clbkHelper we have joint pos of all joints, not only actuated
        }
        
        double wantedPos = (findJoint->second.at(0))*percentageWanted;
        double realPos = clbkHelper.js.position[i];
        
        EXPECT_NEAR (wantedPos, realPos, 0.02);  //norm of the accepted error in roseeExecutor is <0.01

        //the percentage must be exaclty 100 instead (apart double precisions errors, handled by the macro)
        EXPECT_DOUBLE_EQ (clbkHelper.feedback_percentage, 100); 
        EXPECT_EQ (rclcpp_action::ResultCode::SUCCEEDED, clbkHelper.goalState);
        
        //test if the action completed is the same sent
        EXPECT_DOUBLE_EQ (percentageWanted, clbkHelper.actionCompleted.percentage);
        EXPECT_EQ (actionSent->getName(), clbkHelper.actionCompleted.action_name);
        EXPECT_EQ (actionSent->getType(), clbkHelper.actionCompleted.action_type);
        if (actionSent->getType() == ROSEE::Action::Type::Primitive) {
            ROSEE::ActionPrimitive::Ptr primitivePtr = std::static_pointer_cast<ROSEE::ActionPrimitive>(actionSent);
            EXPECT_EQ (primitivePtr->getPrimitiveType(), clbkHelper.actionCompleted.action_primitive_type);
        }
    }
}

void testSendAction::sendAndTest(ROSEE::Action::Ptr action, double percentageWanted) {
    
    setMainNode();
    sendAction(action, percentageWanted);
    testAction(action, percentageWanted);
    
}

/**
 * @brief These Tests create a Generic Action and see if it is sent correctly.
 *  1 - An ActionGeneric is created, where the actuated Joints are set
 * 
 *  2 - The sendAndTest function is called:
 * 
 *    -[setMainNode] The ROSEE main node (which receives action commands and output specific joint pos references) is launched
 *       such that the action created is parsed by him
 *    -[sendAction] The action is commanded.
 *    -[testAction] After rosee says the action is completed, some checks are done to see if the final state is the same as the 
 *      action created
 */

TEST_F ( testSendAction, sendSimpleGeneric ) {
    
    SetUp(argc_g, argv_g);
    
    ROSEE::JointPos jp;
    ROSEE::JointsInvolvedCount jpc;

    std::vector<std::string> actJoints = ee->getActuatedJoints();
    
    ASSERT_FALSE (actJoints.empty());
    
    for (int i = 0; i<actJoints.size(); i++) {
                
        int id = -1;
        ee->getInternalIdForJoint(actJoints.at(i), id);
        
        //create an action where all joints move toward their upper limit
        std::vector<double> one_dof;
        one_dof.push_back ( ee->getUpperPositionLimits()[id] );
        jp.insert ( std::make_pair(actJoints.at(i), one_dof) );
        jpc.insert (std::make_pair(actJoints.at(i), 1));
        
    }

    //ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionGeneric>("testAllUpperLim", jp, jpc);
    ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionGeneric>("RandomActionGeneric", jp, jpc);
    //emit the yaml so roseeExecutor can find the action
    yamlWorker.createYamlFile( action.get(), folderForActions + "/generics/" );

    sendAndTest(action);
    
}

TEST_F ( testSendAction, sendSimpleGeneric2 ) {
    
    SetUp(argc_g, argv_g);
    
    ROSEE::JointPos jp;
    ROSEE::JointsInvolvedCount jpc;

    std::vector<std::string> actJoints = ee->getActuatedJoints();
    
    ASSERT_FALSE (actJoints.empty());
    
    for (int i = 0; i<actJoints.size(); i++) {
                
        int id = -1;
        ee->getInternalIdForJoint(actJoints.at(i), id);
        
        //create an action where all joints move toward their lower limit
        std::vector<double> one_dof;
        one_dof.push_back ( ee->getLowerPositionLimits()[id] );
        jp.insert ( std::make_pair(actJoints.at(i), one_dof) );
        jpc.insert (std::make_pair(actJoints.at(i), 1));
        
    }

    ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionGeneric>("testAllLowerLim", jp, jpc);
    //emit the yaml so roseeExecutor can find the action
    yamlWorker.createYamlFile( action.get(), folderForActions + "/generics/" );

    sendAndTest(action);
    
}


TEST_F ( testSendAction, sendSimpleGeneric3 ) {
    
    SetUp(argc_g, argv_g);
    
    ROSEE::JointPos jp;
    ROSEE::JointsInvolvedCount jpc;

    std::vector<std::string> actJoints = ee->getActuatedJoints();
    
    ASSERT_FALSE (actJoints.empty());
    
    for (int i = 0; i<actJoints.size(); i++) {
                
        int id = -1;
        ee->getInternalIdForJoint(actJoints.at(i), id);
        
        //create an action where all joints move, but make sure to stay within limits
        std::vector<double> one_dof;
        one_dof.push_back ( ee->getLowerPositionLimits()[id]*0.15 + 0.1 );
        jp.insert ( std::make_pair(actJoints.at(i), one_dof) );
        jpc.insert (std::make_pair(actJoints.at(i), 1));
        
    }

    ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionGeneric>("testAllUpperLim2", jp, jpc);
    //emit the yaml so roseeExecutor can find the action
    yamlWorker.createYamlFile( action.get(), folderForActions + "/generics/" );

    sendAndTest(action, 0.33);
    
}

/**
 * @brief This test take a random trig (among the one found for the ee loaded) and command it
 * The check is done after the action completion as before (in the sendAndTest function), 
 * but here we also check that the final pose of the moved finger finds all the actuated joint of that finger
 * in the bigger bound (as it should be by definition of trig)
 */
TEST_F ( testSendAction, sendTrig ) {

    SetUp(argc_g, argv_g);
    
    ROSEE::ActionTrig::Map trigMap = actionsFinder->findTrig (ROSEE::ActionPrimitive::Type::Trig, folderForActions + "/primitives/") ;

    if (trigMap.size()>0){
        std::vector<std::string> keys = ROSEE::Utils::extract_keys(trigMap);
        //lets pick a random trig
        srand((unsigned int)time(NULL));
        int i = rand() % keys.size();
        ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionTrig>(trigMap.at(keys.at(i)));

        sendAndTest(action);
        
        //other than "default" check done in sendAndTest, we check that effectively the trig joints are gone
        //toward their limit (from definition of trig)
        std::vector<std::string> actJointsInvolved;
        ee->getActuatedJointsInFinger(keys.at(i), actJointsInvolved);
        
        for (auto jointName : actJointsInvolved) {
            //at O single dof joint
            double bigBound = parserMoveIt->getBiggerBoundFromZero(jointName).at(0);
            
            for (int k = 0; k<clbkHelper.js.name.size(); k++) {
                
                if (clbkHelper.js.name[i].compare(jointName) == 0 ) {
                    EXPECT_NEAR(bigBound, clbkHelper.js.position[i], 0.02);
                    break;
                } 
            }
        }
    }
}

TEST_F ( testSendAction, sendTipFlex ) {
    
    SetUp(argc_g, argv_g);
    
    auto tipFlexMap = actionsFinder->findTrig (ROSEE::ActionPrimitive::Type::TipFlex, folderForActions + "/primitives/");

    if (tipFlexMap.size()>0) {
        std::vector<std::string> keys = ROSEE::Utils::extract_keys(tipFlexMap);
        //lets pick a random 
        srand((unsigned int)time(NULL));
        int i = rand() % keys.size();
        ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionTrig>(tipFlexMap.at(keys.at(i)));

        sendAndTest(action);
        
        //other than "default" check done in sendAndTest, we check that effectively the joint is gone to the limit
        
        //Workaround to take the single joint used by this action
        auto jic = action->getJointsInvolvedCount();
        std::string jointInvolved;
        for (auto it : jic) {
            if (it.second == 1 ){
                jointInvolved = it.first;
            }
        }
        
        EXPECT_TRUE(jointInvolved.size() > 0);
        
        //at O single dof joint
        double bigBound = parserMoveIt->getBiggerBoundFromZero(jointInvolved).at(0);
        
        for (int k = 0; k<clbkHelper.js.name.size(); k++) {
            
            if (clbkHelper.js.name[i].compare(jointInvolved) == 0 ) {
                EXPECT_NEAR(bigBound, clbkHelper.js.position[i], 0.02);
                break;
                
            }
        }        
    }
}

TEST_F ( testSendAction, sendFingFlex ) {

    SetUp(argc_g, argv_g);
    
    auto fingFlexMap = actionsFinder->findTrig (ROSEE::ActionPrimitive::Type::FingFlex, folderForActions + "/primitives/");

    if (fingFlexMap.size()>0) {
        std::vector<std::string> keys = ROSEE::Utils::extract_keys(fingFlexMap);
        //lets pick a random 
        srand((unsigned int)time(NULL));
        int i = rand() % keys.size();
        ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionTrig>(fingFlexMap.at(keys.at(i)));

        sendAndTest(action);
        
       //other than "default" check done in sendAndTest, we check that effectively the joint is gone to the limit
        
        //Workaround to take the single joint used by this action
        auto jic = action->getJointsInvolvedCount();
        std::string jointInvolved;
        for (auto it : jic) {
            if (it.second == 1 ){
                jointInvolved = it.first;
            }
        }
        
        EXPECT_TRUE(jointInvolved.size() > 0);
        
        //at O single dof joint
        double bigBound = parserMoveIt->getBiggerBoundFromZero(jointInvolved).at(0);
        
        for (int k = 0; k<clbkHelper.js.name.size(); k++) {
            
            if (clbkHelper.js.name[i].compare(jointInvolved) == 0 ) {
                EXPECT_NEAR(bigBound, clbkHelper.js.position[i], 0.02);
                break;
                
            }
        }
    }
}


TEST_F ( testSendAction, sendPinches ) {
    
    SetUp(argc_g, argv_g);
    
    auto pinchTightMap = actionsFinder->findPinch(folderForActions + "/primitives/").first;

    if (pinchTightMap.size()>0){
        std::vector<std::pair<std::string,std::string>> keys = ROSEE::Utils::extract_keys(pinchTightMap);
        //lets pick a random 
        srand((unsigned int)time(NULL));
        int i = rand() % keys.size();
        ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionPinchTight>(pinchTightMap.at(keys.at(i)));

        sendAndTest(action);
        
    }
}

TEST_F ( testSendAction, sendPinchLoose ) {
    
    SetUp(argc_g, argv_g);
    
    auto pinchLooseMap = actionsFinder->findPinch(folderForActions + "/primitives/").second;

    if (pinchLooseMap.size()>0){
        std::vector<std::pair<std::string,std::string>> keys = ROSEE::Utils::extract_keys(pinchLooseMap);
        //lets pick a random 
        srand((unsigned int)time(NULL));
        int i = rand() % keys.size();
        ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionPinchLoose>(pinchLooseMap.at(keys.at(i)));

        sendAndTest(action);
        
    }
}

TEST_F (testSendAction, sendMultiplePinchStrict ) {
    
    SetUp(argc_g, argv_g);
           
    std::vector < ROSEE::ActionMultiplePinchTight::Map > multiplePinchMapsStrict;
    for (int i = 3; i <= ee->getFingersNumber(); i++) {
        auto multiplePinchMapStrict = actionsFinder->findMultiplePinch(i, folderForActions + "/primitives/", true );
        
        //keep only if it is not empty
        if (multiplePinchMapStrict.size() > 0 ) {
            multiplePinchMapsStrict.push_back (multiplePinchMapStrict);
        }    
    }

    if ( multiplePinchMapsStrict.size() > 0 ) {
        
        srand((unsigned int)time(NULL));
        int j = rand() % multiplePinchMapsStrict.size();
        
        std::vector<std::set<std::string>> keys = ROSEE::Utils::extract_keys( multiplePinchMapsStrict.at(j) );
        //lets pick a random trig
        srand((unsigned int)time(NULL));
        int i = rand() % keys.size();
        ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionMultiplePinchTight>(multiplePinchMapsStrict.at(j).at(keys.at(i)));

        sendAndTest(action);
        
    }
    
}

TEST_F (testSendAction, sendMultiplePinchNoStrict ) {
    
    SetUp(argc_g, argv_g);
    
    std::vector < ROSEE::ActionMultiplePinchTight::Map > multiplePinchMapsNOStrict;
    for (int i = 3; i <= ee->getFingersNumber(); i++) {
        auto multiplePinchMapNOStrict = actionsFinder->findMultiplePinch(i, folderForActions + "/primitives/", false );
        
        //keep only if it is not empty
        if (multiplePinchMapNOStrict.size() > 0 ) {
            multiplePinchMapsNOStrict.push_back (multiplePinchMapNOStrict);
        }

    }

    if ( multiplePinchMapsNOStrict.size() > 0 ) {
        
        srand((unsigned int)time(NULL));
        int j = rand() % multiplePinchMapsNOStrict.size();
        
        std::vector<std::set<std::string>> keys = ROSEE::Utils::extract_keys( multiplePinchMapsNOStrict.at(j) );
        //lets pick a random trig
        srand((unsigned int)time(NULL));
        int i = rand() % keys.size();
        ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionMultiplePinchTight>(multiplePinchMapsNOStrict.at(j).at(keys.at(i)));

        sendAndTest(action);
        
    }
    
}

TEST_F (testSendAction, sendSingleJointMultipleTips ) {
    
    SetUp(argc_g, argv_g);
    
    std::vector < ROSEE::ActionSingleJointMultipleTips::Map > singleJointMultipleTipsMaps;
    
    for (int i = 1; i<=ee->getFingersNumber(); i++) {
            
        std::map < std::string, ROSEE::ActionSingleJointMultipleTips> singleJointMultipleTipsMap = 
            actionsFinder->findSingleJointMultipleTips (i, folderForActions + "/primitives/") ;
            
        if (singleJointMultipleTipsMap.size()>0) {
            singleJointMultipleTipsMaps.push_back(singleJointMultipleTipsMap);
        }

    }

    if ( singleJointMultipleTipsMaps.size() > 0 ) {
        
        srand((unsigned int)time(NULL));
        int j = rand() % singleJointMultipleTipsMaps.size();
        
        std::vector<std::string> keys = ROSEE::Utils::extract_keys( singleJointMultipleTipsMaps.at(j) );
        //lets pick a random trig
        srand((unsigned int)time(NULL));
        int i = rand() % keys.size();
        ROSEE::Action::Ptr action = std::make_shared<ROSEE::ActionSingleJointMultipleTips>(singleJointMultipleTipsMaps.at(j).at(keys.at(i)));

        sendAndTest(action);
            
    }
}


/***
 * @brief Here we create a randomic action composed by some primitives
 */
TEST_F (testSendAction, sendComposedAction ) {

    SetUp(argc_g, argv_g);
    
    ROSEE::ActionTrig::Map trigMap = actionsFinder->findTrig (ROSEE::ActionPrimitive::Type::Trig, folderForActions + "/primitives/") ;
    auto fingFlexMap = actionsFinder->findTrig (ROSEE::ActionPrimitive::Type::FingFlex, folderForActions + "/primitives/");
    auto pinchTightMap = actionsFinder->findPinch(folderForActions + "/primitives/").first;
    
    ROSEE::ActionComposed actionComposed ( "TestComposed", false) ;
    
    for (auto trig : trigMap) {
        std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
            std::make_shared <ROSEE::ActionTrig> ( trig.second );
            
        double lower_bound = 0;
        double upper_bound = 0.8; //low, we do not want to go out of joint limits
        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
        std::default_random_engine re;
        double random = unif(re);
        EXPECT_TRUE(actionComposed.sumAction ( pointer, random )); 
    }
    
    for (auto trig : fingFlexMap) {
        std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
            std::make_shared <ROSEE::ActionTrig> ( trig.second );
            
        double lower_bound = 0;
        double upper_bound = 0.7; //low, we do not want to go out of joint limits
        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
        std::default_random_engine re;
        double random = unif(re);
        EXPECT_TRUE(actionComposed.sumAction ( pointer, random )); 
    }
    
    for (auto pinch : pinchTightMap) {
        
        std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
            std::make_shared <ROSEE::ActionPinchTight> ( pinch.second );
            
        double lower_bound = 0;
        double upper_bound = 0.29; //low, we do not want to go out of joint limits
        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
        std::default_random_engine re;
        double random = unif(re);
        EXPECT_TRUE(actionComposed.sumAction ( pointer, random, 2 )); 
    }
    
    if (actionComposed.numberOfInnerActions() > 0) {
        ROSEE::Action::Ptr actionPtr = std::make_shared<ROSEE::ActionGeneric>(actionComposed);
        
        //do not forget to emit the file, in sendAndTest the rosee main node need to parse it
        yamlWorker.createYamlFile( actionPtr.get(), folderForActions + "/generics/" );

        sendAndTest(actionPtr, 0.95);
    }
 
}


TEST_F (testSendAction, sendTimedAction ) {

    SetUp(argc_g, argv_g);
    
    ROSEE::ActionTrig::Map trigMap = actionsFinder->findTrig (ROSEE::ActionPrimitive::Type::Trig, folderForActions + "/primitives/") ;
    auto tipFlexMap = actionsFinder->findTrig (ROSEE::ActionPrimitive::Type::TipFlex, folderForActions + "/primitives/");
    auto pinchMaps = actionsFinder->findPinch(folderForActions + "/primitives/");
    
    ROSEE::ActionTimed actionTimed ( "TestTimed" ) ;
    
    if  (trigMap.size()>0) {
        std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
            std::make_shared <ROSEE::ActionTrig> ( trigMap.begin()->second );
            
        double lower_bound = 0;
        double upper_bound = 0.8; //low, we do not want to go out of joint limits
        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
        std::default_random_engine re;
        double random = unif(re);
        EXPECT_TRUE(actionTimed.insertAction(pointer, 0, 0.7, 0, random)); 
    }
    
    if (tipFlexMap.size() > 0) {
        std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
            std::make_shared <ROSEE::ActionTrig> ( tipFlexMap.rbegin()->second); //rbegin is the last element
            
        double lower_bound = 0;
        double upper_bound = 0.95;
        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
        std::default_random_engine re;
        double random = unif(re);
        EXPECT_TRUE(actionTimed.insertAction(pointer, 0.2, 0.32, 0, random)); 
    }
    
    if (! pinchMaps.first.empty()) {
        
        std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
            std::make_shared <ROSEE::ActionPinchTight> ( pinchMaps.first.begin()->second );
            
        double lower_bound = 0.6;
        double upper_bound = 1; 
        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
        std::default_random_engine re;
        double random = unif(re);
        EXPECT_TRUE(actionTimed.insertAction(pointer, 0.2, 0.32, 1, random)); 
    }
    
    if (! pinchMaps.second.empty()) {        
        std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
            std::make_shared <ROSEE::ActionPinchLoose> ( pinchMaps.second.rbegin()->second );
            
        double lower_bound = 0;
        double upper_bound = 1;
        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
        std::default_random_engine re;
        double random = unif(re);
        EXPECT_TRUE(actionTimed.insertAction(pointer, 0, 0, 0, random)); 
    }
    
    if (actionTimed.getInnerActionsNames().size() > 0) {
        ROSEE::Action::Ptr actionPtr = std::make_shared<ROSEE::ActionTimed>(actionTimed);
        
        //do not forget to emit the file, in sendAndTest the rosee main node need to parse it
        yamlWorker.createYamlFile( actionPtr.get(), folderForActions + "/timeds/" );
        
        sendAndTest(actionPtr);
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
