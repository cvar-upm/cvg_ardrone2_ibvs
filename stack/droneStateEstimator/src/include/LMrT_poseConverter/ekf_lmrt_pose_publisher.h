#ifndef EKF_LMrT_POSE_PUBLISHER_H
#define EKF_LMrT_POSE_PUBLISHER_H

#include "ros/ros.h"
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "pugixml.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include "referenceFrames.h"
#include "droneMsgs/dronePose.h"
#include "droneStateEstimator/setInitDroneYaw_srv_type.h"
#include "communication_definition.h"

class EKF_LMrT_pose_publisher
{
private:
    ros::Publisher droneEstimatedPosePubl;
    ros::ServiceServer   setDroneYawInitSrv;
    bool setInitDroneYaw(droneStateEstimator::setInitDroneYaw_srv_type::Request& request, droneStateEstimator::setInitDroneYaw_srv_type::Response& response);

    // Homogeneous tranform data Frame_{droneLMrT} with respect to Frame_{EKF_LMrT}
    float x, y, z;          // m
    float yaw, pitch, roll; // rad, wYvPuR
    // Homogeneous tranforms
    cv::Mat matHomog_drone_GMR_t0_wrt_GFF;          // from droneXX.xml configFile
    cv::Mat matHomog_drone_LMrT_wrt_drone_GMR;      // constant, see reference frames' definition (see pdf)
    cv::Mat matHomog_drone_GMR_wrt_drone_LMrT;      // constant, see reference frames' definition (see pdf)
    cv::Mat matHomog_EKF_LMrT_wrt_drone_LMrT_t0;    // from initial yaw
    cv::Mat matHomog_EKF_LMrT_wrt_GFF;              // to obtain SLAMs required transform from EKF data
    cv::Mat matHomog_GFF_wrt_EKF_LMrT;              // to obtain droneController required transform from TrajectoryPlanner data
    cv::Mat matHomog_droneLMrT_wrt_EKF_LMrT;        // from EKF

    cv::Mat matHomog_drone_GMR_wrt_GFF;                     // for SLAM
    cv::Mat matHomog_droneLMrT_wrt_EKF_LMrT_4controller;    // for controller

    bool pose_publisher;

public:
    EKF_LMrT_pose_publisher( bool pose_publisher = true);

    int initComponents(std::string configFile);
    void open(ros::NodeHandle & nIn, std::string moduleName);
    void setHomogTransform_rad_wYvPuR( float x, float y, float z, float yaw, float pitch, float roll);
    void publishHomogTransform_rad_wYvPuR();        // publishes matHomog_drone_GMR_wrt_GFF

    // For ControllerNodeMain
    void passFROM_droneGMRwrtGFF_TO_droneLMrTwrtEKF( double x_in, double y_in, double z_in, double yaw_in, double pitch_in, double roll_in, double *px, double *py, double *pz, double *pyaw, double *ppitch, double *proll);
private:
    cv::Mat matHomog_droneGMRwrtGFF, matHomog_droneLMrTwrtEKF;
};

#endif // EKF_LMrT_POSE_PUBLISHER_H
