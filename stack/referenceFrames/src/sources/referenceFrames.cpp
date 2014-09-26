/*
 * referenceFrames.cpp
 *
 *  Created on: 
 *      Author: jl.sanchez
 */


#include "referenceFrames.h"
#include "htrans.h"
#include <cmath>
#include <ctgmath>

namespace referenceFrames
{
int refFrameChangeXYZ2ZYX(droneMsgs::dronePose pose_xYyPzR, droneMsgs::dronePose* pose_zYyPxR) //mobile robots 2 common
{  
    float x_input = pose_xYyPzR.x;
    float y_input = pose_xYyPzR.y;
    float z_input = pose_xYyPzR.z;
    float roll_input = pose_xYyPzR.roll;
    float pitch_input = pose_xYyPzR.pitch;
    float yaw_input = pose_xYyPzR.yaw;
    
    HomogTrans h;
    h.DirectTrans(x_input,y_input,z_input,roll_input,pitch_input,yaw_input);
    
    float x_output = x_input;
    float y_output = y_input;
    float z_output = z_input;
    float roll_output = atan2(-h.mat[1][2],h.mat[2][2]);
    float pitch_output = asin(h.mat[0][2]);
    float yaw_output = acos(h.mat[0][0]/cos(pitch_output));
    
    pose_zYyPxR->x = x_output;
    pose_zYyPxR->y = y_output;
    pose_zYyPxR->z = z_output;
    pose_zYyPxR->roll = roll_output;
    pose_zYyPxR->pitch = pitch_output;
    pose_zYyPxR->yaw = yaw_output;
    
    
    return 1;
}

int refFrameChangeZYX2XYZ(droneMsgs::dronePose pose_zYyPxR, droneMsgs::dronePose* pose_xYyPzR) //common 2 mobile robots
{
    
    float x_input = pose_zYyPxR.x;
    float y_input = pose_zYyPxR.y;
    float z_input = pose_zYyPxR.z;
    float roll_input = pose_zYyPxR.roll;
    float pitch_input = pose_zYyPxR.pitch;
    float yaw_input = pose_zYyPxR.yaw;
    
    HomogTrans h;
    h.DirectTrans(x_input,y_input,z_input,roll_input,pitch_input,yaw_input);
    
    float x_output = x_input;
    float y_output = y_input;
    float z_output = z_input;
    float roll_output = atan2(h.mat[1][0],h.mat[0][0]);
    float pitch_output = atan2(-h.mat[2][0],cos(roll_output)*h.mat[0][0]+sin(roll_output)*h.mat[1][0]);
    float yaw_output = atan2(sin(roll_output)*h.mat[0][2]-cos(roll_output)*h.mat[1][2], -sin(roll_output)*h.mat[0][1]+cos(roll_output)*h.mat[1][1]);
    
    pose_xYyPzR->x = x_output;
    pose_xYyPzR->y = y_output;
    pose_xYyPzR->z = z_output;
    pose_xYyPzR->roll = roll_output;
    pose_xYyPzR->pitch = pitch_output;
    pose_xYyPzR->yaw = yaw_output;    
    
    return 1;
}

// xYyPzR 2 wYvPuR 2
int xYyPzR2wYvPuR(droneMsgs::dronePose pose_xYyPzR, droneMsgs::dronePose* pose_wYvPuR) // GMR 2 LMrT
{
	 float x_input = pose_xYyPzR.x;
    float y_input = pose_xYyPzR.y;
    float z_input = pose_xYyPzR.z;
    float roll_input = pose_xYyPzR.roll;
    float pitch_input = pose_xYyPzR.pitch;
    float yaw_input = pose_xYyPzR.yaw;
    
    float x_output = x_input;
    float y_output = y_input;
    float z_output = z_input;
    float roll_output = yaw_input;
    float pitch_output = pitch_input;
    float yaw_output = roll_input;
    
    pose_wYvPuR->x = x_output;
    pose_wYvPuR->y = y_output;
    pose_wYvPuR->z = z_output;
    pose_wYvPuR->roll = roll_output;
    pose_wYvPuR->pitch = pitch_output;
    pose_wYvPuR->yaw = yaw_output;
    

    return 1;



}
// wYvPuR 2 xYyPzR
int wYvPuR2xYyPzR(droneMsgs::dronePose pose_wYvPuR, droneMsgs::dronePose* pose_xYyPzR) // LMrT 2 GMR
{

	 float x_input = pose_wYvPuR.x;
    float y_input = pose_wYvPuR.y;
    float z_input = pose_wYvPuR.z;
    float roll_input = pose_wYvPuR.roll;
    float pitch_input = pose_wYvPuR.pitch;
    float yaw_input = pose_wYvPuR.yaw;
    
    float x_output = x_input;
    float y_output = y_input;
    float z_output = z_input;
    float roll_output = yaw_input;
    float pitch_output = pitch_input;
    float yaw_output = roll_input;
    
    pose_xYyPzR->x = x_output;
    pose_xYyPzR->y = y_output;
    pose_xYyPzR->z = z_output;
    pose_xYyPzR->roll = roll_output;
    pose_xYyPzR->pitch = pitch_output;
    pose_xYyPzR->yaw = yaw_output;

    
    return 1;



}

int createHomogMatrix_wYvPuR(cv::Mat *pHomogMat, double x, double y, double z, double yaw, double pitch, double roll) {
    int error_ocurred = 0;

    if ( (pHomogMat == NULL) || (pHomogMat->empty()) || (pHomogMat->size() != cv::Size(4,4)) ) {
        error_ocurred = 1;
        return error_ocurred;
    }

    pHomogMat->at<float>(0,0) = cos(pitch)*cos(yaw);
    pHomogMat->at<float>(1,0) = cos(pitch)*sin(yaw);
    pHomogMat->at<float>(2,0) = -sin(pitch);
    pHomogMat->at<float>(3,0) = 0.0;

    pHomogMat->at<float>(0,1) = cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw);
    pHomogMat->at<float>(1,1) = cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw);
    pHomogMat->at<float>(2,1) = cos(pitch)*sin(roll);
    pHomogMat->at<float>(3,1) = 0.0;

    pHomogMat->at<float>(0,2) = sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch);
    pHomogMat->at<float>(1,2) = cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll);
    pHomogMat->at<float>(2,2) = cos(pitch)*cos(roll);
    pHomogMat->at<float>(3,2) = 0.0;

    pHomogMat->at<float>(0,3) = x;
    pHomogMat->at<float>(1,3) = y;
    pHomogMat->at<float>(2,3) = z;
    pHomogMat->at<float>(3,3) = 1.0;

    return error_ocurred;
}

int getxyzYPRfromHomogMatrix_wYvPuR(const cv::Mat &HomogMat, double *px, double *py, double *pz, double *pyaw, double *ppitch, double *proll) {
    int error_ocurred = 0;

    if ( (HomogMat.empty()) || (HomogMat.size() != cv::Size(4,4)) ) {
        error_ocurred = 1;
        return error_ocurred;
    }

    (*px) = HomogMat.at<float>(0,3);
    (*py) = HomogMat.at<float>(1,3);
    (*pz) = HomogMat.at<float>(2,3);

    // http://en.wikibooks.org/wiki/Robotics_Kinematics_and_Dynamics/Description_of_Position_and_Orientation#Inverse_Mapping_2
    // http://en.wikipedia.org/wiki/Euler_angles#Conversion_between_intrinsic_and_extrinsic_rotations
    float tol=1e-4;
    if ( (pow(HomogMat.at<float>(1,0),2) + pow(HomogMat.at<float>(0,0),2) <= tol) ||
         (pow(HomogMat.at<float>(2,1),2) + pow(HomogMat.at<float>(2,2),2) <= tol) ) {
        std::cout<<"referenceFrames::getxyzYPRfromHomogMatrix_wYvPuR: ¡Bad conditioning in Pitch! Pitch =+-pi/2" << std::endl;

        if ( HomogMat.at<float>(2,0) > 0 ) {
            (*ppitch) = -M_PI/2;
            (*proll)  =     0.0;
            (*pyaw)    = atan2( -HomogMat.at<float>(2,1), -HomogMat.at<float>(2,0));
        } else {
            (*ppitch) =  M_PI/2;
            (*proll)  =     0.0;
            (*pyaw)    = atan2(  HomogMat.at<float>(2,1),  HomogMat.at<float>(2,0));
        }
        error_ocurred = 1;
        return error_ocurred;
    }

    (*pyaw)   = atan2(  HomogMat.at<float>(1,0), HomogMat.at<float>(0,0));
    (*ppitch) = atan2( -HomogMat.at<float>(2,0), cos(*pyaw)*HomogMat.at<float>(0,0)+sin(*pyaw)*HomogMat.at<float>(1,0) );
    (*proll)  = atan2(  HomogMat.at<float>(2,1), HomogMat.at<float>(2,2));

    return error_ocurred;
}

int getxyzYPRfromHomogMatrix_xYyPzR(const cv::Mat &HomogMat, double *px, double *py, double *pz, double *pyaw, double *ppitch, double *proll) {
    int error_ocurred = 0;

    // http://en.wikipedia.org/wiki/Euler_angles#Conversion_between_intrinsic_and_extrinsic_rotations
    error_ocurred = getxyzYPRfromHomogMatrix_wYvPuR( HomogMat, px, py, pz, pyaw, ppitch, proll);
    double paux = (*pyaw);
    (*pyaw)  = (*proll);
    (*proll) = paux;

    return error_ocurred;
}

int createMatHomogFromVecs(cv::Mat* pHomogMat, const cv::Mat &TransVec, const cv::Mat &RotVec) {

    int error_ocurred = 0;
    if ( (pHomogMat == NULL) || (pHomogMat->empty()) || (pHomogMat->size() != cv::Size(4,4)) ) {
        error_ocurred = 1;
        return error_ocurred;
    }
//    (*pHomogMat) = cv::Mat::eye(4,4,CV_32F);

    // Rotation
    cv::Mat RotMat;
    cv::Rodrigues(RotVec,RotMat);

    for(int i=0;i<3;i++) {
        for(int j=0;j<3;j++) {
            pHomogMat->at<float>(i,j)=RotMat.at<float>(i,j);
        }
    }


    //Translation
    for(int i=0;i<3;i++) {
        pHomogMat->at<float>(i,3)=TransVec.at<float>(i,0);
    }

    //Scale
    pHomogMat->at<float>(3,3)=1.0;

    for(int i=0;i<3;i++) {
        pHomogMat->at<float>(3,i) = 0.0;
    }

    return error_ocurred;
}



}







