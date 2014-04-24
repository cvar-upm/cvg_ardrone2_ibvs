#include <stdio.h>
#include <iostream>

// ROS
#include "ros/ros.h"

// parrotDriver
#include "DroneLogger.h"

int main(int argc, char **argv)
{
    printf("Starting ARDrone Parrot Logger...\n");

    ros::init(argc, argv, "parrotLogger");
    ros::NodeHandle n;

    //Drone
    std::string node_name("Logger");
    DroneLogger drone_logger( node_name, droneModule::active); //Bastaría con monitor???
    drone_logger.open(n, "droneLogger");

    //Loop
    while (ros::ok())
    {
        //Read messages
        ros::spinOnce();

        //Sleep
        drone_logger.sleep();
    }
    return 1;
}


