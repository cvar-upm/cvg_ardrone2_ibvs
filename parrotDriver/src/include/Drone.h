/******************************
 * Drone class
 *
 *Author: JL
 *Date: 24-04-2013
 *Last update: 24-04-2013
 *Version: 0.1
 *
 *Description: class to manage ardrone_autonomy node
 *View: https://github.com/AutonomyLab/ardrone_autonomy
 *
 *****************************/


#ifndef _DRONE_H
#define _DRONE_H



/////// OpenCV ///////
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur




////// ROS  ///////
#include "ros/ros.h"



///// AR DRONE AUTONOMY /////
//Services
#include "std_srvs/Empty.h"

#include "ardrone_autonomy/CamSelect.h"
#include "ardrone_autonomy/LedAnim.h"
#include "ardrone_autonomy/FlightAnim.h"

//Topics
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

//images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"

#include <sensor_msgs/image_encodings.h>


//Messages
#include "ardrone_autonomy/Navdata.h"

#include <geometry_msgs/Twist.h>



///////// CVG DRONE MSGS
#include "droneMsgs/droneNavCommand.h"
#include "droneMsgs/droneNavData.h"






////// OTHERS ////////

#include <sstream>

#include <stdio.h>



//Define limits commands
#define LIMIT_COMMAND_DYAW       1.0
#define LIMIT_COMMAND_PITCH     1.0
#define LIMIT_COMMAND_ROLL      1.0
#define LIMIT_COMMAND_DHEIGHT    1.0



namespace parrotDriver
{
    enum parrotDriverTypes
    {
        monitor, //Only read
        drive, //only write
        full //read + write
    };
    enum parrotDriverLoggerTypes
    {
        non_logger, // logs data
        logger      // does not log data
    };
}

// website with an example on the usage of tags http://www.somacon.com/p125.php
#define PARROT_DRIVER_LOG_TAG_RESET     0x00
#define PARROT_DRIVER_LOG_NAVDATA       0x01
#define PARROT_DRIVER_LOG_DRONEMODES    0x02


class ARDrone
{

    //Ros node
private:
    ros::NodeHandle n;




    //Parrot Driver type
private:
    parrotDriver::parrotDriverTypes parrotDriverType;




private:

    //Services
    //Empty
    std_srvs::Empty emptySrvMsg;

    //toggle cam
    ros::ServiceClient toggleCamClient;
    //Set cam channel
    ros::ServiceClient setCamChannelClient;
    ardrone_autonomy::CamSelect setCamChannelSrvMsg;
    //Led animation
    ros::ServiceClient ledAnimationClient;
    ardrone_autonomy::LedAnim ledAnimationSrvMsg;
    //Flight animation
    ros::ServiceClient flightAnimationClient;
    ardrone_autonomy::FlightAnim flightAnimationSrvMsg;
    //Imu calibration
    ros::ServiceClient imuCalibrationClient;
    //Flat Trim
    ros::ServiceClient FlatTrimClient;




    ////////Parrot HL commands
private:
    //Messages
    std_msgs::Empty emptyMsg;
    //Publishers
    ros::Publisher takeOffPubl;
    ros::Publisher landPubl;
    ros::Publisher resetPubl;



    /////////// Navdata
private:

    ////// Suscribers
    ros::Subscriber navdataSub;
    ardrone_autonomy::Navdata navData;
    droneMsgs::droneNavData droneNavData;
    //Topics callbacks
    void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);



    ////////////Cameras: http://www.ros.org/wiki/image_transport

    /////// Front Camera
private:
    ros::Subscriber frontCameraSub;
    //Front image
    cv_bridge::CvImagePtr cvFrontImage;
    bool frontImageRead;
    cv::Mat frontImage; //TODO
    ros::Time frontImage_timesamp;
    uint32_t frontImage_seq;
    //Topics callbacks
    void frontCameraCallback(const sensor_msgs::ImageConstPtr& msg);


    ////////Bottom Camera
private:
    ros::Subscriber bottomCameraSub;
    cv_bridge::CvImagePtr cvBottomImage;
    bool bottomImageRead;
    cv::Mat bottomImage; //TODO
    ros::Time bottomImage_timesamp;
    uint32_t bottomImage_seq;
    //Topics callbacks
    void bottomCameraCallback(const sensor_msgs::ImageConstPtr& msg);



    /////////Commands
private:
    geometry_msgs::Twist navCommand;
    droneMsgs::droneNavCommand droneNavCommand; //CVG drone command type
    ////
    //cmdT.angular.z = -cmd.yaw;
    //cmdT.linear.z = cmd.gaz;
    //cmdT.linear.x = -cmd.pitch;
    //cmdT.linear.y = -cmd.roll;
    ros::Subscriber commandsSub;
    ros::Publisher commandsPubl;
    //Topics callbacks
    void commandsCallback(const geometry_msgs::Twist::ConstPtr& msg);









public:
    ARDrone(parrotDriver::parrotDriverTypes typeIn=parrotDriver::full, parrotDriver::parrotDriverLoggerTypes parrotDriverLoggerType = parrotDriver::non_logger, unsigned char log_configuration_tag_in = PARROT_DRIVER_LOG_TAG_RESET );
    ~ARDrone();

    void init();

    void open(ros::NodeHandle & n, bool subscribe_to_rectified_front_cam = false, bool subscribe_to_rectified_bottom_cam = false);
    void close();




    ///////Parrot HL commands
    void takeOff();
    void land();
    void reset();
    void hover();
    void move();


    ///////Commands
    void setNavCommandToZero();
    //ARdrone
    void setNavCommand(geometry_msgs::Twist navCommandIn);
    geometry_msgs::Twist getNavCommand(void);
    //CVG drone
    void setDroneNavCommand(droneMsgs::droneNavCommand navCommandIn);
    droneMsgs::droneNavCommand getDroneNavCommand(void);


    //Toggle camera
    bool toggleCamera(void);

    //Set cam channel
    bool setCamChannel(uint8_t channelIn);

    //Led Animation
    bool setLedAnimation(uint8_t typeIn, float freqIn, uint8_t durationIn);

    //Flight Animation
    bool setFlightAnimation(uint8_t typeIn, uint16_t durationIn);

    //Imu calibration
    bool calibrateImu(void);

    //Flat Trim
    bool flatTrim(void);


    //////Images
    bool isFrontImageRead();
    bool isBottomImageRead();
    bool displayFrontImage(std::string windowName);
    bool displayBottomImage(std::string windowName);

    bool getFrontImage(cv::Mat *frontImageOut, uint32_t *pseq = NULL);
    bool getBottomImage(cv::Mat *bottomImageOut, uint32_t *pseq = NULL);
    cv::Mat getFrontImage();
    cv::Mat getBottomImage();
    ros::Time getFrontImageTimestamp();
    ros::Time getBottomImageTimestamp();


    //////NavData
    ardrone_autonomy::Navdata getNavData();
    droneMsgs::droneNavData getDroneNavData();

    // DroneLogger
private:
    parrotDriver::parrotDriverLoggerTypes parrotDriverLoggerType;
    unsigned char log_configuration_tag;
    ros::Duration run_timestamp, event_timestamp;
    ros::Time     init_timestamp;
    std::ostringstream droneDriverLogMsgStrm;
    std::ostringstream droneDriverModeEventLogMsgStrm;

public:
    void updateDroneDriverLogMsgStr();
    inline void setInitTimestamp( const ros::Time &init_timestamp_in) { init_timestamp = init_timestamp_in; }
    void        getDroneDriverLogMsgStr( std::string &str_in );
    std::string navdataState2String( uint32_t state );
    void        updateDroneDriverModeEventLogMsgStr( const std::string &event_mode_str );
    void        getDroneDriverModeEventLogMsgStr( std::string &str_in );
};





#endif



