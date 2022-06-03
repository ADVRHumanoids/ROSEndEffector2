#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <end_effector/HAL/EEHalPlugin.h>
#include <pluginlib/class_loader.hpp>

#ifdef _MATLOGGER2
    #include <matlogger2/matlogger2.h>
    #include <matlogger2/utils/mat_appender.h>
#endif

int main ( int argc, char **argv ) {
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("EEHalExecutor");

    std::string hal_lib;
    node->declare_parameter("hal_library_name", "");
    if (!  node->get_parameter("hal_library_name", hal_lib) ) {
        RCLCPP_ERROR_STREAM(node->get_logger(),  "Ros parameter 'hal_library_name' not found" );
        return -1;
    }

#ifdef _MATLOGGER2
    std::string matlogger_path;
    XBot::MatLogger2::Ptr logger; /* mt logger */
    XBot::MatAppender::Ptr appender; /* mt logger */

    bool logging = false;

    node->declare_parameter("matlogger_path", "");
    if ( node->get_parameter ( "matlogger_path", matlogger_path ) && matlogger_path.size() != 0) 
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "Logging data with matlogger into " << matlogger_path  );

        logger = XBot::MatLogger2::MakeLogger(matlogger_path); // date-time automatically appended
        appender = XBot::MatAppender::MakeInstance();
        appender->add_logger(logger);
        appender->start_flush_thread();        
        logging = true;
    }
#endif

    pluginlib::ClassLoader<ROSEE::EEHalPlugin> eeHalLoader("end_effector", "ROSEE::EEHalPlugin");

    std::shared_ptr<ROSEE::EEHalPlugin> eeHalPtr = eeHalLoader.createSharedInstance(hal_lib);

    if (eeHalPtr == nullptr) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "[EEHalExecutor ERROR] in loading the EEHal Object" );
        return -1;    
    }
    
    eeHalPtr->initialize(node);
    
    RCLCPP_INFO_STREAM(node->get_logger(), "[EEHalExecutor] Loaded "<<  hal_lib << " HAL"  );   
    
    if ( eeHalPtr->isHandInfoPresent() ) 
    { 
        eeHalPtr->init_hand_info_response();
        eeHalPtr->setHandInfoCallback();
    }
    

      
    //TODO take rate from param
    rclcpp::Rate r(100);
    while(rclcpp::ok()) {
        
        //TODO check order of these
        
        //receive info from robot, and fill _js_msg
        eeHalPtr->sense();
        
        //make the robot move following the refs in _mr_msg
        //But be sure that someone has sent a motor command to the hal
        auto& clk = *node->get_clock();
        if (eeHalPtr->checkCommandPub()) {
            eeHalPtr->move();
            RCLCPP_INFO_STREAM_ONCE (node->get_logger(), "[EEHalExecutor] someone is publishing on '/ros_end_effector/motor_reference_pos', I will call the move()" );   
            
        } else {
            RCLCPP_WARN_STREAM_THROTTLE (node->get_logger(), clk, 5000, "[EEHalExecutor] no-one is publishing on '/ros_end_effector/motor_reference_pos', I will not call the move()" );   
        }
        
        //send _js_msg to external (ie to ROSEE main node)
        eeHalPtr->publish_joint_state();
        
        
        
        if (eeHalPtr->_pressure_active) {
            eeHalPtr->publish_pressure();
        } 
        
#ifdef _MATLOGGER2        
        if (logging) {
            logger->add("motor_pos_ref", eeHalPtr->getMotorReference());
            logger->add("motor_pos", eeHalPtr->getJointPosition());
            logger->add("motor_eff", eeHalPtr->getJointEffort());
            auto pressures =  eeHalPtr->getPressure();
            logger->add("pressure1", pressures.row(0));
            logger->add("pressure2", pressures.row(1));
            logger->add("pressure3", pressures.row(2));
            logger->add("pressure4", pressures.row(3));
        }
#endif

        //ros2 migration guide says to use spin_some to substitute ros1 spinOnce... even if ros2 spin_once does exist
        rclcpp::spin_some(node);
        r.sleep();
        
    }
    
    return 0;
    
}
