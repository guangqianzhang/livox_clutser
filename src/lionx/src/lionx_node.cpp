//
// Created by zgq on 2022/5/18.
//

//
// Created by zgq on 2022/5/14.
//
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include "render.h"
#include "ProcessPointClouds.h"
#include "ProcessPointClouds.cpp" //moban

#include "glog/logging.h"
#include "ros/ros.h"

using namespace std;
typedef pcl::PointXYZ point;
std::string WORK_SPACE_PATH;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "lionx");
    ros::NodeHandle nh;
    if (nh.getParam("/lionx_node/WORK_SPACE_PATH", WORK_SPACE_PATH))
    {
        std::cout << "WORK_SPACE_PATH=" << WORK_SPACE_PATH << std::endl;
    }
    else
    {
        ROS_INFO("no get WORK_SPACE_PATH");
    }
    
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/zgq/Documents/ros/lionx/src/lionx/Log";

    FLAGS_alsologtostderr = 1;
    LOG(INFO) << FLAGS_log_dir;
    // Stream PCD
    ProcessPointClouds<point> *pointProcessorI = new ProcessPointClouds<point>(nh);
}
