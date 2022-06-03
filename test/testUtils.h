#ifndef TESTUTILS_H
#define TESTUTILS_H

#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>

//for the function prepareROSForTests
#include <rclcpp/rclcpp.hpp>
#include <end_effector/Utils.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>


/** Utils funcion to create process to run roscore,
 * gently copied from https://github.com/ADVRHumanoids/CartesianInterface/blob/refactor2020/tests/testutils.h
 */

namespace ROSEE {
    
namespace TestUtils {

class Process
{

public:

    Process(std::vector<std::string> args);

    int wait();

    void kill(int signal = SIGTERM);

    ~Process();

private:

    std::string _name;
    pid_t _pid;

};

Process::Process(std::vector<std::string>  args):
    _name(args[0])
{
    std::vector<const char *> args_cstr;
    for(auto& a : args) args_cstr.push_back(a.c_str());
    args_cstr.push_back(nullptr);

    char ** argv = (char**)args_cstr.data();

    _pid = ::fork();

    if(_pid == -1)
    {
        perror("fork");
        throw std::runtime_error("Unable to fork()");
    }

    if(_pid == 0)
    {
        std::vector<std::string> source_args{ament_index_cpp::get_package_prefix("end_effector") + "/../setup.bash"};
        char** source_args_char = (char**)source_args.data();
        
        ::execvp("source", source_args_char);
        ::execvp(argv[0], argv);
        perror("execvp");
        throw std::runtime_error("Unknown command");
    }

}

int Process::wait()
{
    int status;
    while(::waitpid(_pid, &status, 0) != _pid);
    printf("Child process '%s' exited with status %d\n", _name.c_str(), status);
    return status;

}

void Process::kill(int signal)
{
    ::kill(_pid, signal);
    printf("Killed process '%s' with signal %d\n", _name.c_str(), signal);
}

Process::~Process()
{
    kill(SIGINT);
    wait();
}

/**
 * @brief Function to be called in the main of each test, it runs roscore and fill
 * parameter server with robot models
 * 
 * @return a not 0 if some error happens
 */
rclcpp::Node::SharedPtr prepareROSForTests ( int argc, char **argv, std::string testName ) {
    
    auto node = rclcpp::Node::make_shared(testName);
    
    //fill ros param with file models, needed by moveit parserMoveIt
    std::string modelPathURDF = ament_index_cpp::get_package_share_directory("end_effector") + "/configs/urdf/" + argv[1];
    std::string modelPathSRDF = ament_index_cpp::get_package_share_directory("end_effector") + "/configs/srdf/" + argv[1];

    //Is there a better way to parse?
    std::ifstream urdf(modelPathURDF + ".urdf");
    std::ifstream srdf(modelPathSRDF + ".srdf");
    std::stringstream sUrdf, sSrdf;
    sUrdf << urdf.rdbuf();
    sSrdf << srdf.rdbuf();

    node->declare_parameter("robot_description", sUrdf.str()); 
    node->declare_parameter("robot_description_semantic", sSrdf.str()); 
//     node->declare_parameter("robot_name", argv[1]); 
    
    node->set_parameter(rclcpp::Parameter("robot_description", sUrdf.str()));
    node->set_parameter(rclcpp::Parameter("robot_description_semantic", sSrdf.str()));
//     node->set_parameter(rclcpp::Parameter("robot_name", argv[1]));

    return node;
}


} //namespace TestUtils

} //namespace ROSEE


namespace {
    
int argc_g;    
char** argv_g;

class MyTestEnvironment : public testing::Environment {
    
public:
    
    explicit MyTestEnvironment(int argc, char **argv ) {
        argc_g = argc;
        argv_g = argv;
    }
};
    
}

#endif // TESTUTILS_H
