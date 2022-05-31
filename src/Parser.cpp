/*
 * Copyright (C) 2019 IIT-HHCM
 * Author: Luca Muratore
 * email:  luca.muratore@iit.it
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include <end_effector/Parser.h>

#define CHAIN_PER_GROUP 1

ROSEE::Parser::Parser ( const rclcpp::Node::SharedPtr node ) :
    _node ( node ) {

}


ROSEE::Parser::~Parser() {

}

bool ROSEE::Parser::getJointsInFinger ( std::string base_link,
                                        std::string tip_link,
                                        std::string finger_name
                                      ) {

    //we add here the finger to the map because later is added only if the joint is actuated.
    //Instead we want in the map all the fingers, even if they have no actuated joint in.
    //this is necessary to not cause later problem due to have the right number of finger
    _finger_joint_map.insert(std::make_pair(finger_name, std::vector<std::string>())); 
    
    
    KDL::Chain actual_chain;
    if ( _robot_tree.getChain ( base_link, tip_link, actual_chain ) ) {

        int segments_num = actual_chain.getNrOfSegments();
        for ( int i = 0; i < segments_num; i++ ) {

            KDL::Segment actual_segment = actual_chain.getSegment ( i );
            KDL::Joint actual_joint = actual_segment.getJoint();

            bool is_valid_joint = false;

            // Check if joint is revolute or prismatic
            is_valid_joint = actual_joint.getTypeName() == "RotAxis";   
            
//                              || actual_joint.getTypeName() == "TransAxis";
            
            auto urdf_joint = _urdf_model->getJoint(actual_joint.getName());
            bool is_mimic_joint = (urdf_joint->mimic == nullptr) ? false : true;
            

            // if the joint is revolute or prismatic AND not a mimic (mimic joint is a not actuated joint)
            if ( is_valid_joint && (!is_mimic_joint) ) {

                _finger_joint_map[finger_name].push_back ( actual_joint.getName() );
                _joint_finger_map[actual_joint.getName()] = finger_name;
                _urdf_joint_map[actual_joint.getName()] = _urdf_model->getJoint ( actual_joint.getName() );

                _joints_num++;

            }
        }

        return true;
    }

    RCLCPP_ERROR_STREAM (_node->get_logger(), "chain from base_link " << base_link << " to tip_link " << tip_link << " not found in the URDF" );

    return false;
}



bool ROSEE::Parser::parseSRDF() {

    // initialize srdfdom
    _srdfdom.initFile ( *_urdf_model, _srdf_path );

    // get the end-effectors in the SRDF file
    std::vector<srdf::Model::EndEffector> srdf_end_effectors = _srdfdom.getEndEffectors();

    // NOTE only one end-effectors supported
    // TBD support multiple end-effectors
    if ( srdf_end_effectors.size() != 1 ) {

        return false;
    }

    RCLCPP_INFO_STREAM (_node->get_logger(), "ros_end_effector Parser found end_effector: " << srdf_end_effectors.at ( 0 ).name_ );

    // get all the groups in the SRDF
    std::vector<srdf::Model::Group> srdf_groups = _srdfdom.getGroups();

    // TBD will be a vector
    srdf::Model::Group fingers_group;
    // find end effector fingers chains group
    // TBD will be a vector
    int group_num = srdf_groups.size();
    // TBD will be a vector
    std::string end_effector_group_name = srdf_end_effectors.at ( 0 ).component_group_;
    for ( int i = 0; i < group_num; i++ ) {
        if ( srdf_groups[i].name_ == end_effector_group_name ) {
            fingers_group = srdf_groups[i];
            RCLCPP_INFO_STREAM (_node->get_logger(), "ros_end_effector Parser found group: " << end_effector_group_name << " in the SRDF with the following fingers: " );
        }
    }

    _fingers_num = fingers_group.subgroups_.size();
    _fingers_names.resize ( _fingers_num );
    _fingers_group_id.resize ( _fingers_num );

    // fill the chain names vector
    for ( int i = 0; i < _fingers_num; i++ ) {

        _fingers_names[i] = fingers_group.subgroups_[i];

        // find the id of the current finger group
        for ( int j = 0; j < group_num; j++ ) {
            if ( srdf_groups[j].name_ == _fingers_names[i] ) {
                _fingers_group_id[i] = j;
            }
        }
        RCLCPP_INFO_STREAM (_node->get_logger(), _fingers_names[i] );
    }

    // iterate over the fingers and find revolute joints
    for ( int i = 0; i < _fingers_num; i++ ) {
        srdf::Model::Group current_finger_group = srdf_groups[ _fingers_group_id[i] ];

        // NOTE only one chain per group
        if ( current_finger_group.chains_.size() != CHAIN_PER_GROUP )  {

            RCLCPP_ERROR_STREAM (_node->get_logger(), "for the finger chain groups you can specify only one chain per group in your SRDF check " << 
                               current_finger_group.name_.c_str() << " group" );
            return false;
        }

        // fill the enabled/disabled joints in chain map
        if ( !getJointsInFinger ( current_finger_group.chains_[0].first,
                                  current_finger_group.chains_[0].second,
                                  current_finger_group.name_
                                ) ) {
            return false;
        }
    }
    
    addNotInFingerJoints();
    
    removePassiveJoints();

    // save srdf as string 
    std::ifstream t_srdf ( _srdf_path );
    std::stringstream buffer_srdf;
    buffer_srdf << t_srdf.rdbuf();
    _srdf_string = buffer_srdf.str();
    
    return true;

}

void ROSEE::Parser::addNotInFingerJoints() {
    
    for (auto it : _urdf_model->joints_) { //this contains all joints
        
        if (it.second->mimic == nullptr) { //not a mimic joint...
        
            if (it.second->type == urdf::Joint::CONTINUOUS ||
                it.second->type == urdf::Joint::REVOLUTE ) {
                //it.second->type == urdf::Joint::PRISMATIC
            
                if (_urdf_joint_map.find(it.second->name) == _urdf_joint_map.end() ) {
                    
                    _urdf_joint_map.insert(std::make_pair(it.second->name, it.second));
                    _finger_joint_map["virtual_finger"].push_back ( it.second->name );
                    _joint_finger_map[it.second->name] = "virtual_finger";
                    _joints_num++;
                }
            }
        }
        
    }
    
}

bool ROSEE::Parser::removePassiveJoints() {
    
    std::vector<srdf::Model::PassiveJoint> passiveJointList = _srdfdom.getPassiveJoints();
    
    for (auto passiveJoint : passiveJointList) {
        //the passive can be also already deleted so we chech if we delete it(e.g. it is also mimic)
        if (_urdf_joint_map.erase(passiveJoint.name_) > 0) {
            
            //we have also to remove it from the other maps
            std::string fingerOfPassive = _joint_finger_map.at(passiveJoint.name_);
            _joint_finger_map.erase(passiveJoint.name_);
            
            for (auto it = _finger_joint_map.at(fingerOfPassive).begin(); 
                 it!= _finger_joint_map.at(fingerOfPassive).end(); ++it){
                
                if (it->compare(passiveJoint.name_) == 0 ) {
                    _finger_joint_map.at(fingerOfPassive).erase(it);
                    break;
                }
            }

            _joints_num--;
        }   
    }

    return true;
}


bool ROSEE::Parser::parseURDF() {

    std::string xml_string;
    std::fstream xml_file ( _urdf_path.c_str(), std::fstream::in );
    if ( xml_file.is_open() ) {

        while ( xml_file.good() ) {

            std::string line;
            std::getline ( xml_file, line );
            xml_string += ( line + "\n" );
        }

        xml_file.close();
        _urdf_model = urdf::parseURDF ( xml_string );

        // create the robot KDL tree from the URDF model
        if ( !kdl_parser::treeFromUrdfModel ( *_urdf_model, _robot_tree ) ) {

            RCLCPP_ERROR_STREAM (_node->get_logger(), "in " << __func__ << " Failed to construct kdl tree" );
            return false;
        }

        // save urdf and string (can be useful to have, thanks @alaurenzi!)
        std::ifstream t_urdf ( _urdf_path );
        std::stringstream buffer_urdf;
        buffer_urdf << t_urdf.rdbuf();
        _urdf_string = buffer_urdf.str();

        return true;

    } else {

        RCLCPP_ERROR_STREAM (_node->get_logger(), "in " << __func__ << " : Can NOT open " << _urdf_path << " !" );
        return false;
    }
}


bool ROSEE::Parser::configure() {

    bool ret = true;

    if ( parseURDF() ) {
        
        if ( parseSRDF() ) {
            
            RCLCPP_INFO_STREAM (_node->get_logger(), "ROSEndEffector Parser successfully configured using urdf file:  " << _urdf_path 
                << "\n\t srdf file: " << _srdf_path << "\n\t actions folder " << _action_path
            );
        
        } else {
        
            RCLCPP_ERROR_STREAM (_node->get_logger(), "ROSEndEffector Parser error while parsing SRDF");
            ret = false;
        }
        
    } else {
        
        RCLCPP_ERROR_STREAM (_node->get_logger(), "ROSEndEffector Parser error while parsing URDF");
        ret = false;
    }




    return ret;
}



bool ROSEE::Parser::init() {
    
    // in ROS2, param must be declare first to be find
    _node->declare_parameter ( "urdf_path", "" );
    _node->declare_parameter ( "srdf_path", "" );
    _node->declare_parameter ( "actions_folder_path", "" );

    if ( _node->get_parameter ( "urdf_path", _urdf_path ) && 
         _node->get_parameter ( "srdf_path", _srdf_path ) &&
         _node->get_parameter ( "actions_folder_path", _action_path )
    ) {

        //_action_path = std::string(getenv("HOME")) + "/" + _action_path;
        
        _is_initialized =  configure();
        return _is_initialized;
    }

    // error
    RCLCPP_ERROR_STREAM (_node->get_logger(), "in " << __func__ << " : '_urdf_path' and/or '_srdf_path' and/or 'actions_folder_path' not found on ROS parameter server" );
    return false;

}


bool ROSEE::Parser::init ( const std::string& urdf_path, const std::string& srdf_path, const std::string& action_path ) {

    _urdf_path = urdf_path;
    _srdf_path = srdf_path;
    _action_path = action_path;

    _is_initialized =  configure();
    return _is_initialized;
}


void ROSEE::Parser::printEndEffectorFingerJointsMap() const {

    if ( _is_initialized ) {

        RCLCPP_INFO_STREAM (_node->get_logger(), "ROS End Effector: Finger Joints Map" );
        RCLCPP_INFO_STREAM (_node->get_logger(), "-------------------------" );
        for ( auto& chain_joints: _finger_joint_map ) {
            RCLCPP_INFO_STREAM (_node->get_logger(), chain_joints.first );

            int nJointInFinger = chain_joints.second.size();
            
            if ( nJointInFinger == 0 ) {
                
                RCLCPP_INFO_STREAM (_node->get_logger(), "No actuated joint in this finger" );
                
            } else {
                
                for ( int i = 0; i < nJointInFinger ; i++ ) {
                    RCLCPP_INFO_STREAM (_node->get_logger(), chain_joints.second.at ( i ) );
                }
            }

            RCLCPP_INFO_STREAM (_node->get_logger(), "-------------------------" );
        }
    } else {

        RCLCPP_ERROR_STREAM (_node->get_logger(), "in " << __func__ << " :  ROSEE::Parser needs to be initialized. Call init() frist." );
    }
}

int ROSEE::Parser::getActuatedJointsNumber() const {
    
    return _joints_num;
}

std::map< std::string, std::vector< std::string >> ROSEE::Parser::getFingerJointMap() const {

    return _finger_joint_map;
}

std::map< std::string, std::string > ROSEE::Parser::getJointFingerMap() const {
    
    return _joint_finger_map;
}

std::map< std::string, urdf::JointConstSharedPtr > ROSEE::Parser::getUrdfJointMap() const {

    return _urdf_joint_map;
}

void ROSEE::Parser::getFingerJointMap( std::map< std::string, std::vector< std::string >>& finger_joint_map ) const {

    finger_joint_map = _finger_joint_map;
}

void ROSEE::Parser::getJointFingerMap ( std::map< std::string, std::string >& joint_finger_map ) const {

    joint_finger_map = _joint_finger_map;
}

std::string ROSEE::Parser::getEndEffectorName() const {

    return _urdf_model->getName();
}

std::string ROSEE::Parser::getUrdfString() const {
    return _urdf_string;
}

std::string ROSEE::Parser::getSrdfString() const {
    return _srdf_string;
}


std::string ROSEE::Parser::getActionPath() const {
    return _action_path;
}




