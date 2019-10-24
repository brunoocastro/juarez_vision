/*#include "robotis_controller/robotis_controller.h"

 Motion Module Header 

#include "op3_walking_module/walking_module.h"


using namespace op3;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "walking_controller");
    ros::NodeHandle nh;

    ROS_INFO("manager->init");
    robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

     Load ROS Parameter
    std::string offset_file = nh.param<std::string>("offset_file_path", "");
    std::string robot_file  = nh.param<std::string>("robot_file_path", "");

    std::string init_file   = nh.param<std::string>("init_file_path", "");

    if(robot_file == "")
    {
        ROS_ERROR("NO robot file path in the ROS parameters.");
        return -1;
    }

    if(controller->initialize(robot_file, init_file) == false)
    {
        ROS_ERROR("ROBOTIS Controller Initialize Fail!");
        return -1;
    }

    if(offset_file != "")
        controller->loadOffset(offset_file);

    sleep(1);

    Add Sensor Module

     Add Motion Module
   
    controller->addMotionModule((robotis_framework::MotionModule*)WalkingMotionModule::getInstance());

    controller->startTimer();

    while(ros::ok())
    {
      usleep(1000*1000);
    }

    return 0;
}
*/