cmake_minimum_required(VERSION 3.5)
project(end_effector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
   add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_TESTING "Build tests" ON)

##test code coverage
option(ROSEE_ENABLE_COVERAGE "Compile with coverage information" ON)
if(${ROSEE_ENABLE_COVERAGE})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage -g")
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(srdfdom REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosee_msg REQUIRED)
find_package(pluginlib REQUIRED)
                                                                                                                                                  
find_package(Eigen3 REQUIRED)
#find_package(muparser REQUIRED) 
find_library(_MUPARSER_LIB muparser REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(matlogger2 QUIET)
if (${matlogger2_FOUND})
    message("Logging with MatLogger2")
    add_definitions(-D_MATLOGGER2)
endif (${matlogger2_FOUND})


###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

######### Offline phase libs
list(APPEND libToInstall)

add_library(ROSEEParser
    src/Parser.cpp
)
list(APPEND libToInstall ROSEEParser)

add_library(ROSEEInterface
    src/EEInterface.cpp
)
list(APPEND libToInstall ROSEEInterface)

add_library(ROSEEFindActions 
    src/FindActions.cpp
)
list(APPEND libToInstall ROSEEFindActions)

add_library(ROSEEParserMoveIt
    src/ParserMoveIt.cpp
)
list(APPEND libToInstall ROSEEParserMoveIt)

add_library(ROSEEYamlWorker
    src/YamlWorker.cpp
)
list(APPEND libToInstall ROSEEYamlWorker)

add_library(ROSEEMapActionHandler
    src/MapActionHandler.cpp
)
list(APPEND libToInstall ROSEEMapActionHandler)

add_library(ROSEEActions 
    src/GraspingActions/Action.cpp 
    src/GraspingActions/ActionPrimitive.cpp
    src/GraspingActions/ActionPinchTight.cpp
    src/GraspingActions/ActionPinchGeneric.cpp
    src/GraspingActions/ActionPinchLoose.cpp
    src/GraspingActions/ActionTrig.cpp
    src/GraspingActions/ActionSingleJointMultipleTips.cpp
    src/GraspingActions/ActionMultiplePinchTight.cpp
    src/GraspingActions/ActionGeneric.cpp 
    src/GraspingActions/ActionComposed.cpp 
    src/GraspingActions/ActionTimed.cpp 
)
list(APPEND libToInstall ROSEEActions)

ament_target_dependencies(ROSEEParser
    rclcpp
    kdl_parser
    srdfdom
)
target_link_libraries(ROSEEParser
    yaml-cpp
)

target_link_libraries(ROSEEInterface
    ROSEEParser
)

target_link_libraries(ROSEEYamlWorker
    yaml-cpp
    ROSEEActions
)

target_link_libraries(ROSEEMapActionHandler
    ROSEEActions
    ROSEEYamlWorker
)

ament_target_dependencies(ROSEEFindActions
    moveit_ros_planning_interface
)
target_link_libraries(ROSEEFindActions
    ROSEEParserMoveIt
    ROSEEYamlWorker
    ROSEEActions
    muparser
)

ament_target_dependencies(ROSEEParserMoveIt
    rclcpp
    moveit_ros_planning_interface
)

ament_target_dependencies(ROSEEActions
    moveit_ros_planning_interface
)


######### ONline phase libs

###Hal libs 

add_library(EEHalPlugin SHARED
     src/HAL/EEHalPlugin.cpp
)
ament_target_dependencies(EEHalPlugin
	rclcpp
	rosee_msg
	sensor_msgs
)
target_link_libraries(EEHalPlugin 
    yaml-cpp
)
list(APPEND libToInstall EEHalPlugin)

add_library(DummyHalPlugin SHARED
    src/HAL/DummyHalPlugin.cpp
)
ament_target_dependencies(DummyHalPlugin
    rclcpp
	rosee_msg 
	sensor_msgs
)
target_link_libraries(DummyHalPlugin
    EEHalPlugin
)
pluginlib_export_plugin_description_file(end_effector plugins.xml)
list(APPEND libToInstall DummyHalPlugin)


## Main online stuff
add_library(UniversalRosEndEffectorExecutor
    src/UniversalRosEndEffectorExecutor.cpp
)
list(APPEND libToInstall UniversalRosEndEffectorExecutor)


add_library(ROSEERosActionServer
    src/RosActionServer.cpp
)
list(APPEND libToInstall ROSEERosActionServer)

add_library(ROSEERosServiceHandler
    src/RosServiceHandler.cpp
)
list(APPEND libToInstall ROSEERosServiceHandler)


ament_target_dependencies(ROSEERosActionServer
    rclcpp 
    rosee_msg
    rclcpp_action
)
# TODO this?? add_dependencies(ROSEERosActionServer rosee_msg_generate_messages_cpp)

ament_target_dependencies(ROSEERosServiceHandler
    rclcpp
    rosee_msg
)
# TODO this??add_dependencies(ROSEERosServiceHandler rosee_msg_generate_messages_cpp)



add_executable(UniversalFindActions src/UniversalFindActions.cpp)
# add_dependencies(UniversalFindActions end_effector_generate_messages_cpp)
target_link_libraries(UniversalFindActions
    ROSEEParserMoveIt
    ROSEEParser
    ROSEEFindActions
    ROSEEActions
    ROSEEMapActionHandler
)


add_executable(EEHalExecutor src/HAL/EEHalExecutor.cpp)
ament_target_dependencies(EEHalExecutor
    rclcpp  
    rosee_msg
    pluginlib
)
if (${matlogger2_FOUND})
    target_link_libraries(EEHalExecutor
        EEHalPlugin
        matlogger2::matlogger2
    )
    
else()
    target_link_libraries(EEHalExecutor
        EEHalPlugin
    )
endif(${matlogger2_FOUND})


add_executable(UniversalRosEndEffector src/UniversalRosEndEffector.cpp)
# add_dependencies(UniversalRosEndEffectorExecutor end_effector_generate_messages_cpp)


ament_target_dependencies(UniversalRosEndEffectorExecutor
    rclcpp
    rosee_msg
)
target_link_libraries(UniversalRosEndEffectorExecutor
    ROSEEParser
    ROSEEInterface
    ROSEEYamlWorker
    ROSEEMapActionHandler
    ROSEERosActionServer
    ROSEERosServiceHandler
    
)

target_link_libraries(UniversalRosEndEffector
    UniversalRosEndEffectorExecutor 
)




#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS 
    UniversalFindActions
    EEHalExecutor
    UniversalRosEndEffector
    ${libToInstall} 
    DESTINATION lib/${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

install(DIRECTORY launch/
  DESTINATION launch
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(PROGRAMS 
    scripts/send_action.py 
    DESTINATION lib/${PROJECT_NAME}
)


install(FILES
  configs/urdf/heri_II.urdf
  configs/urdf/qbhand.urdf
  configs/urdf/robotiq_2f_140.urdf
  configs/urdf/robotiq_3f.urdf
  configs/urdf/schunk.urdf
  configs/urdf/test_ee.urdf
  configs/urdf/test_ee_spread.urdf
  configs/urdf/two_finger.urdf
  configs/urdf/two_finger_mimic.urdf
  DESTINATION share/${PROJECT_NAME}/configs/urdf
)

install(FILES
  configs/srdf/heri_II.srdf
  configs/srdf/qbhand.srdf
  configs/srdf/robotiq_2f_140.srdf
  configs/srdf/robotiq_3f.srdf
  configs/srdf/schunk.srdf
  configs/srdf/test_ee.srdf
  configs/srdf/test_ee_spread.srdf
  configs/srdf/two_finger.srdf
  configs/srdf/two_finger_mimic.srdf
  DESTINATION share/${PROJECT_NAME}/configs/srdf
)

##############
### Testing ##
##############

if(BUILD_TESTING)

    # Add gtest based cpp test target and link libraries
    #catkin_add_gtest(EEInterface_test test/test_ee_interface.cpp)
    add_executable(EEInterface_test test/test_ee_interface.cpp)
    ament_target_dependencies(EEInterface_test
		rclcpp
		sensor_msgs
		ament_index_cpp
	)
    target_link_libraries(EEInterface_test
                                           ROSEEParser
                                           ROSEEInterface
                                           gtest)
                                           
    add_executable(FindPinches_test test/test_find_pinches.cpp)
    ament_target_dependencies(FindPinches_test
		rclcpp
	)
    target_link_libraries(FindPinches_test
                                            ROSEEFindActions
                                            ROSEEActions
                                            gtest)
                                            
    add_executable(FindTrigs_test test/test_find_trigs.cpp)
	ament_target_dependencies(FindTrigs_test
		rclcpp
	)
    target_link_libraries(FindTrigs_test
                                         ROSEEFindActions
                                         ROSEEActions
                                         gtest)
                                         
    add_executable(ComposedAction_test test/test_composedAction.cpp)
    ament_target_dependencies(ComposedAction_test
		rclcpp  
	)
    target_link_libraries(ComposedAction_test
                                              ROSEEFindActions
                                              ROSEEActions
                                              gtest)
                                              
    add_executable(TimedAction_test test/test_timedAction.cpp)
    ament_target_dependencies(TimedAction_test
		rclcpp 
	)
    target_link_libraries(TimedAction_test
                                              ROSEEFindActions
                                              ROSEEActions
                                              gtest)
                                              
    add_executable(SendAction_test test/test_send_action.cpp)
    ament_target_dependencies(SendAction_test
		rclcpp
		rosee_msg
		rclcpp_action
		ament_index_cpp
	)
    target_link_libraries(SendAction_test
                                              ROSEEActions
                                              gtest
                                              ROSEEYamlWorker
                                              ROSEEParser
                                              ROSEEInterface
                                              ROSEEFindActions)  
                                              
    add_executable(ServiceHandler_test test/test_service_handler.cpp)
    ament_target_dependencies(ServiceHandler_test
		rclcpp
		rosee_msg
	)
    target_link_libraries(ServiceHandler_test
                                              gtest
                                              ROSEEParser
                                              ROSEEInterface
                                              ROSEEMapActionHandler
                                              ROSEERosServiceHandler)  
    
    list(APPEND testsExecutable 
        EEInterface_test 
         FindPinches_test 
         FindTrigs_test 
         ComposedAction_test 
         TimedAction_test
         SendAction_test 
         ServiceHandler_test
    )
    list(APPEND handsForTests
         test_ee 
         test_ee_spread 
         two_finger 
         two_finger_mimic
    )
    
    #run each test with each hand
    foreach(hand ${handsForTests})
        foreach(testEx ${testsExecutable})
        
            add_test(
                NAME ${testEx}_${hand}
                COMMAND ${testEx} ${hand}
            )

        endforeach()    
    endforeach()
    

    #for code coverage, from Arturo xbot2
    add_custom_target(test_clean_coverage
        COMMAND /bin/sh ${CMAKE_CURRENT_SOURCE_DIR}/test/scripts/clean_gcov.sh
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    
    add_custom_target(test_coverage
        COMMAND lcov -c -d . --output-file main_coverage.info
        #remove external libraries in the coverage
        COMMAND lcov -r main_coverage.info '/usr*' 'boost*' 'eigen3*' 'c++*' '/opt*' -o main_coverage.info
        COMMAND genhtml main_coverage.info --output-directory coverage_out
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
                
endif()


################################# doxygen documentation

#A macro to allow clean, readable inclusion of subdirectories
macro(optional_build name path ONOFF)
  option(BUILD_${name} ${ONOFF})
  if( BUILD_${name} )
    #We this need to make include files available examples... a bit brute force
    #include_directories("${PROJECT_SOURCE_DIR}/${path}")
    add_subdirectory(${path})
  endif()
endmacro(optional_build name path ONOFF)

optional_build(documentation doc ON)

ament_export_libraries(
  end_effector
)
ament_export_targets(
  export_${PROJECT_NAME}
)


ament_package()

