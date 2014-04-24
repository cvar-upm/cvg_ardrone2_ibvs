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
#include "ekf_lmrt_pose_publisher.h"
#include "communication_definition.h"

#define PI 3.1415





int main(int argc, char **argv)
{

    ros::init(argc, argv, "parrotStateEstimator");
  	ros::NodeHandle n;


    //Drone
    ARDrone MyDrone(parrotDriver::monitor);
    MyDrone.open(n);
    
    
    //State Estimator
    DroneStateEstimator MyStateEstimator(droneModule::active);
    MyStateEstimator.open(n,MODULE_NAME_ODOMETRY_STATE_ESTIMATOR);

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

    EKF_LMrT_pose_publisher ekf_LMrt_pose_publisher;
    std::string stackPath; ros::param::get("~stackPath",stackPath); stackPath+="/";
    if(!ekf_LMrt_pose_publisher.initComponents(stackPath+"IMAV13_configuration_files/drone"+MyStateEstimator.getStringId()+".xml")) {
        std::cout<<"[WARNING] ekf_LMrt_pose_publisher: error opening components"<<std::endl;
    }
    ekf_LMrt_pose_publisher.open(n,MODULE_NAME_ODOMETRY_STATE_ESTIMATOR);
    
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

        // publish pose Frame_{droneLMrT} with respect to Frame_{EKF_LMrT}
        float yaw_aux, pitch_aux, roll_aux;
        yaw_aux   = DronePose.yaw;
        pitch_aux = droneNavData.pitch*M_PI/180.0; // pitch_aux = DronePose.pitch;
        roll_aux  = droneNavData.roll*M_PI/180.0; // roll_aux = DronePose.roll;
        ekf_LMrt_pose_publisher.setHomogTransform_rad_wYvPuR( DronePose.x, DronePose.y, DronePose.z, yaw_aux, pitch_aux, roll_aux);
        ekf_LMrt_pose_publisher.publishHomogTransform_rad_wYvPuR();

        //Sleep
        MyStateEstimator.sleep();



	}
    
    
    
    
    return 1;
    
}

