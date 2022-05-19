cmake_minimum_required(VERSION 3.5)
project(end_effector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##test code coverage
option(ROSEE_ENABLE_COVERAGE "Compile with coverage information" ON)
if(${ROSEE_ENABLE_COVERAGE})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage -g")
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rclpy REQUIRED) TODO necessary?
find_package(srdfdom REQUIRED)
find_package(kdl_parser REQUIRED)
###find_package(message_generation REQUIRED)  still necessary?                                                                   
find_package(moveit_ros_planning_interface REQUIRED)
#find_package(rosee_msg REQUIRED) rosee_msg2 ?
                                                                               
find_package(Eigen3 REQUIRED)
#TODO on my pc, find_package works. On iit pc, I need the find_library. Why?
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


list(APPEND libToInstall)

add_library(ROSEEParser
    src/Parser.cpp
)
list(APPEND libToInstall ROSEEParser)

add_library(ROSEEInterface
    src/EEInterface.cpp
)
list(APPEND libToInstall ROSEEInterface)

add_library(DummyHal
     src/HAL/DummyHal.cpp
)
list(APPEND libToInstall DummyHal)

add_library(EEHal
     src/HAL/EEHal.cpp
)
list(APPEND libToInstall EEHal)

add_library(UniversalRosEndEffectorExecutor
    src/UniversalRosEndEffectorExecutor.cpp
)
list(APPEND libToInstall UniversalRosEndEffectorExecutor)

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

add_library(ROSEERosActionServer
    src/RosActionServer.cpp
)
list(APPEND libToInstall ROSEERosActionServer)

add_library(ROSEERosServiceHandler
    src/RosServiceHandler.cpp
)
list(APPEND libToInstall ROSEERosServiceHandler)

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


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(UniversalRosEndEffector src/UniversalRosEndEffector.cpp)
add_executable(UniversalFindActions src/UniversalFindActions.cpp)
add_executable(EEHalExecutor src/HAL/EEHalExecutor.cpp)
# # add_dependencies(UniversalRosEndEffectorExecutor end_effector_generate_messages_cpp)
# # add_dependencies(UniversalFindActions end_effector_generate_messages_cpp)

## Specify libraries to link a library or executable target against

ament_target_dependencies(ROSEEParser
    rclcpp
    kdl_parser
    srdfdom
)
target_link_libraries(ROSEEParser
    yaml-cpp
)

ament_target_dependencies(ROSEEInterface
#TODO add packages previously added with ${catkin_LIBRARIES}
)
target_link_libraries(ROSEEInterface
    ROSEEParser
)

ament_target_dependencies(UniversalRosEndEffectorExecutor
#TODO add packages previously added with ${catkin_LIBRARIES}  
)
target_link_libraries(UniversalRosEndEffectorExecutor
    ROSEEParser
    ROSEEInterface
    ROSEEYamlWorker
    ROSEEMapActionHandler
    ROSEERosActionServer
    ROSEERosServiceHandler
    
)

target_link_libraries(ROSEEYamlWorker
    yaml-cpp
    ROSEEActions
)

target_link_libraries(ROSEEMapActionHandler
    ROSEEActions
    ROSEEYamlWorker
)

ament_target_dependencies(ROSEERosActionServer
#TODO add packages previously added with ${catkin_LIBRARIES}  
)
# TODO this?? add_dependencies(ROSEERosActionServer rosee_msg_generate_messages_cpp)

ament_target_dependencies(ROSEERosServiceHandler
#TODO add packages previously added with ${catkin_LIBRARIES}  
)
# TODO this??add_dependencies(ROSEERosServiceHandler rosee_msg_generate_messages_cpp)

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
    moveit_ros_planning_interface
)

target_link_libraries(UniversalRosEndEffector
    UniversalRosEndEffectorExecutor 
)

target_link_libraries(UniversalFindActions
    ROSEEParserMoveIt
    ROSEEParser
    ROSEEFindActions
    ROSEEActions
    ROSEEMapActionHandler
)

if (${matlogger2_FOUND})
	ament_target_dependencies(EEHalExecutor
		#TODO add packages previously added with ${catkin_LIBRARIES}  
	)
    target_link_libraries(EEHalExecutor
        EEHal
        matlogger2::matlogger2
    )
    
else()
	ament_target_dependencies(EEHalExecutor
		#TODO add packages previously added with ${catkin_LIBRARIES}  
	)
    target_link_libraries(EEHalExecutor
        EEHal
    )
endif(${matlogger2_FOUND})

ament_target_dependencies(EEHal
	#TODO add packages previously added with ${catkin_LIBRARIES}  
)
target_link_libraries(EEHal 
    yaml-cpp
)
# TODO this? add_dependencies(EEHal rosee_msg_generate_messages_cpp)

ament_target_dependencies(DummyHal
	#TODO add packages previously added with ${catkin_LIBRARIES}  
)
target_link_libraries(DummyHal
    EEHal 
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS UniversalRosEndEffector
                UniversalFindActions
                EEHalExecutor
                ${libToInstall} 
		DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

install(DIRECTORY launch/
  #DESTINATION launch
  DESTINATION share/${PROJECT_NAME}/
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
  DESTINATION configs/urdf
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
  DESTINATION configs/srdf
)

##############
### Testing ##
##############

if(BUILD_TESTING)

    # Add gtest based cpp test target and link libraries
    #catkin_add_gtest(EEInterface_test test/test_ee_interface.cpp)
    add_executable(EEInterface_test test/test_ee_interface.cpp)
    ament_target_dependencies(EEInterface_test
		#TODO add packages previously added with ${catkin_LIBRARIES}  
	)
    target_link_libraries(EEInterface_test
                                           ROSEEParser
                                           ROSEEInterface
                                           gtest)
                                           
    add_executable(FindPinches_test test/test_find_pinches.cpp)
    ament_target_dependencies(FindPinches_test
		#TODO add packages previously added with ${catkin_LIBRARIES}  
	)
    target_link_libraries(FindPinches_test
                                            ROSEEFindActions
                                            ROSEEActions
                                            gtest)
                                            
    add_executable(FindTrigs_test test/test_find_trigs.cpp)
	ament_target_dependencies(FindTrigs_test
		#TODO add packages previously added with ${catkin_LIBRARIES}  
	)
    target_link_libraries(FindTrigs_test
                                         ROSEEFindActions
                                         ROSEEActions
                                         gtest)
                                         
    add_executable(ComposedAction_test test/test_composedAction.cpp)
    ament_target_dependencies(ComposedAction_test
		#TODO add packages previously added with ${catkin_LIBRARIES}  
	)
    target_link_libraries(ComposedAction_test
                                              ROSEEFindActions
                                              ROSEEActions
                                              gtest)
                                              
    add_executable(TimedAction_test test/test_timedAction.cpp)
    ament_target_dependencies(TimedAction_test
		#TODO add packages previously added with ${catkin_LIBRARIES}  
	)
    target_link_libraries(TimedAction_test
                                              ROSEEFindActions
                                              ROSEEActions
                                              gtest)
                                              
    add_executable(SendAction_test test/test_send_action.cpp)
    ament_target_dependencies(SendAction_test
		#TODO add packages previously added with ${catkin_LIBRARIES}  
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
		#TODO add packages previously added with ${catkin_LIBRARIES}  
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



ament_package(
    INCLUDE_DIRS include
#  LIBRARIES end_effector
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

