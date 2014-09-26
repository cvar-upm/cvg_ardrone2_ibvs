/*
*
*
*
*
*/


/// ROS
#include "ros/ros.h"


#include <stdio.h>
#include <iostream>


//parrotDriver
#include "Drone.h"



//State estimator
#include "droneStateEstimator.h"


//msgs
#include "droneMsgs/dronePose.h"
#include "droneMsgs/droneSpeeds.h"
#include "droneMsgs/droneNavData.h"
//poseMeasurements
//TODO
#include "droneMsgs/droneSensorData.h"
#include "communication_definition.h"

#define PI 3.1415

// DroneLogger - include
#include "DroneLogger.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "parrotStateEstimator");
    ros::NodeHandle n;


    //Drone
    ARDrone MyDrone(parrotDriver::monitor);
    MyDrone.open(n);


    //State Estimator
    DroneStateEstimator MyStateEstimator(droneModule::active, droneModule::logger);
    MyStateEstimator.open(n,MODULE_NAME_ODOMETRY_STATE_ESTIMATOR);
    std::string ekfLogMsgStr;          // DroneLogger - String
//    std::string ekfEventLogMsgStr;     // DroneLogger - String

    // DroneLogger - Object initialization
    std::string node_name("IBVS_droneEKF");
    DroneLogger drone_logger( node_name, droneModule::monitor);
    drone_logger.open(n, "droneLogger");
    // DroneLogger - Obtaining and diseminating initial timestamp
    ros::Time init_logger_timestamp;
    std::string  currentlog_path_str;
    while (!drone_logger.getCurrentLogPathInitTimeStamp( init_logger_timestamp, currentlog_path_str)) {
        ros::Duration(0.5).sleep();
    }
    MyStateEstimator.setInitTimestamp( init_logger_timestamp );

    //Init
    printf("Starting ARDrone Parrot state estimator...\n");

    //Command inputs
    droneMsgs::droneNavCommand droneNavCommand;
    //geometry_msgs::Twist navCommands;
    //Odometry measures
    //ardrone_autonomy::Navdata navData;
    droneMsgs::droneNavData droneNavData;
    float timeMeasuresOdom=0.0;
    //sensor measures
    droneMsgs::droneSensorData sensorData;
    //Estimated State
    droneMsgs::dronePose DronePose;
    droneMsgs::droneSpeeds DroneSpeeds;

    //Loop
    while (ros::ok())
    {

        //Read ros messages
        ros::spinOnce();



        //Set inputs
        //Read-> Comprobar!!!!
        droneNavCommand=MyDrone.getDroneNavCommand();
        //navCommands=MyDrone.getNavCommand();
        //inputData.pitch=-navCommands.linear.x;
        //inputData.roll=-navCommands.linear.y;
        //inputData.dyaw=navCommands.angular.z; //Derivada de Yaw
        //inputData.dz=-navCommands.linear.z; //derivada de z

        //Set
        MyStateEstimator.setDroneInputs(droneNavCommand);


        //Set measures
        //Odometry
        droneNavData=MyDrone.getDroneNavData();
        //navData=MyDrone.getNavData();
        if(droneNavData.time>timeMeasuresOdom)
        {
            //AJUSTAMOS inputs
            //feedbackData.altitude=-1.0*navData.altd/100.0;
            //feedbackData.pitch=navData.rotY;
            //feedbackData.roll=navData.rotX;
            //feedbackData.yaw=navData.rotZ;
            //feedbackData.speedX=navData.vx/1000.0;
            //feedbackData.speedY=navData.vy/1000.0;
            //timeMeasuresOdom=navData.tm;

            MyStateEstimator.setDroneMeasuresOdometry(droneNavData);
        }


        //Sensors



        //State estimation
        int ok=MyStateEstimator.stateEstimation(-10.0);



        //Get output
        MyStateEstimator.getEstimatedPose(&DronePose);
        MyStateEstimator.getEstimatedSpeeds(&DroneSpeeds);

        //Display
        std::cout<<"ES "<<ok<<": x="<<DronePose.x<<"; y="<<DronePose.y<<"; z="<<DronePose.z
                <<"; Yaw="<<180.0/PI*DronePose.yaw<<"; Pitch="<<180.0/PI*DronePose.pitch<<"; Roll="<<180.0/PI*DronePose.roll<<std::endl;

        // DroneLogger - MyStateEstimator - Retrieving and logging stacked log string
        MyStateEstimator.getStateEstimatorLogMsgStr( ekfLogMsgStr );
        if (ekfLogMsgStr.length() > 0)
            drone_logger.publishEventString( ekfLogMsgStr );
//        // DroneLogger - MyStateEstimator - Same with event logging
//        MyDrone.getDroneDriverModeEventLogMsgStr( droneModeEventLogMsgStr );
//        if (droneModeEventLogMsgStr.length() > 0)
//            while (!drone_logger.logThisStringService( droneModeEventLogMsgStr )) {
//                ros::Duration(0.005).sleep();
//        }

        //Sleep
        MyStateEstimator.sleep();
    }




    return 1;

}

