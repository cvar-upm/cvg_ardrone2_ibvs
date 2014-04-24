/*
 * referenceFrames.h
 *
 *  Created on: 
 *      Author: jl.sanchez
 */

#ifndef REFERENCE_FRAMES_H
#define REFERENCE_FRAMES_H



////// ROS  ///////
#include "ros/ros.h"




//Cpp
#include <sstream>
#include <string>
#include <iostream>




//Messages
#include "droneMsgs/dronePose.h"

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/opencv.hpp>

namespace referenceFrames
{

//Paloma 2 Sist ref global
int refFrameChangeXYZ2ZYX(droneMsgs::dronePose pose_xYyPzR, droneMsgs::dronePose* pose_zYyPxR); //mobile robots 2 common
int refFrameChangeZYX2XYZ(droneMsgs::dronePose pose_zYyPxR, droneMsgs::dronePose* pose_xYyPzR); //common 2 mobile robots

// xYyPzR 2 wYvPuR 
int xYyPzR2wYvPuR(droneMsgs::dronePose pose_xYyPzR, droneMsgs::dronePose* pose_wYvPuR); // GMR xYyPzR 2 GMR wYvPuR
// wYvPuR 2 xYyPzR
int wYvPuR2xYyPzR(droneMsgs::dronePose pose_wYvPuR, droneMsgs::dronePose* pose_xYyPzR); // GMR wYvPuR 2 GMR xYyPzR (mine_p)



//Otros

// Funciones auxiliares
int createHomogMatrix_wYvPuR(cv::Mat *pHomogMat, double x, double y, double z, double yaw, double pitch, double roll);
inline int createHomogMatrix_xYyPzR(cv::Mat *pHomogMat, double x, double y, double z, double yaw, double pitch, double roll) { return createHomogMatrix_wYvPuR( pHomogMat, x, y, z, roll, pitch, yaw); }
int getxyzYPRfromHomogMatrix_wYvPuR(const cv::Mat &HomogMat, double *px, double *py, double *pz, double *pyaw, double *ppitch, double *proll);
int getxyzYPRfromHomogMatrix_xYyPzR(const cv::Mat &HomogMat, double *px, double *py, double *pz, double *pyaw, double *ppitch, double *proll);

// from arUcoDefinition.cpp
int createMatHomogFromVecs(cv::Mat* MatHomog, const cv::Mat &TransVec, const cv::Mat &RotVec);
}
        
        
 

#endif
