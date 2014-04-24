//Drone
#include "Drone.h"


//using namespace ros;

using namespace cv;

/*
ARDrone::ARDrone()
{
    parrotDriverType=full;

    this->init();


    return;
}
*/

ARDrone::ARDrone(parrotDriver::parrotDriverTypes typeIn, parrotDriver::parrotDriverLoggerTypes parrotDriverLoggerTypeIn, unsigned char log_configuration_tag_in) :
    parrotDriverLoggerType(parrotDriverLoggerTypeIn),
    log_configuration_tag(log_configuration_tag_in)
{
    parrotDriverType=typeIn;
    droneDriverLogMsgStrm.str(std::string());
    droneDriverModeEventLogMsgStrm.str(std::string());
    init_timestamp  = ros::Time::now();
    this->init();


    return;
}


ARDrone::~ARDrone()
{
	close();
    return;
}

void ARDrone::init()
{
    //Images
    frontImageRead=false;
    frontImage_timesamp = ros::Time::now();
    //cvFrontImage->image.empty();
    //frontImage=Mat::zeros(480,640,CV_8UC3);
    //frontImage=NULL;
    bottomImageRead=false;
    bottomImage_timesamp = ros::Time::now();
    //cvBottomImage->image.empty();
    //bottomImage=Mat::zeros(480,640,CV_8UC3);
    //bottomImage=NULL;

    //NavDataValues
    navData.altd=0;
    navData.ax=0;
    navData.ay=0;
    navData.az=0;
    navData.batteryPercent=0.0;
    navData.magX=0;
    navData.magY=0;
    navData.magZ=0;
    navData.pressure=0;
    navData.rotX=0;
    navData.rotY=0;
    navData.rotZ=0;
    navData.state=0;
    //TODO: tags
    navData.temp=0;
    navData.tm=0;
    navData.vx=0;
    navData.vy=0;
    navData.vz=0;
    navData.wind_angle=0;
    navData.wind_comp_angle=0;
    navData.wind_speed=0;

    //Commands
    navCommand.linear.x=0.0;
    navCommand.linear.y=0.0;
    navCommand.linear.z=0.0;
    navCommand.angular.x=0.0;
    navCommand.angular.y=0.0;
    navCommand.angular.z=0.0;

    run_timestamp   = ros::Duration(0.0);
    event_timestamp = ros::Duration(0.0);

    frontImage_seq = 0;
    bottomImage_seq = 0;

    return;

}


void ARDrone::open(ros::NodeHandle & nIn, bool subscribe_to_rectified_front_cam, bool subscribe_to_rectified_bottom_cam)
{
    //ROS node Handle
    n=nIn;


    /////////// Services ///////////////

    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        //toggle cam
        toggleCamClient = n.serviceClient<std_srvs::Empty>("ardrone/togglecam");

        //Set cam channel
        setCamChannelClient = n.serviceClient<ardrone_autonomy::CamSelect>("ardrone/setcamchannel");

        //Start led animation client
        ledAnimationClient = n.serviceClient<ardrone_autonomy::LedAnim>("ardrone/setledanimation");

        //Flight Animation client
        flightAnimationClient = n.serviceClient<ardrone_autonomy::FlightAnim>("ardrone/setflightanimation");

        //Imu calibration
        imuCalibrationClient = n.serviceClient<std_srvs::Empty>("ardrone/imu_recalib");

        //Flat Trim
        FlatTrimClient = n.serviceClient<std_srvs::Empty>("ardrone/flattrim");

    }



    //Chart to suscribe

    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::monitor)
    {
        navdataSub = n.subscribe("ardrone/navdata", 1, &ARDrone::navdataCallback, this);

        //http://www.ros.org/wiki/image_transport
        //image_transport::ImageTransport it(n);
        if (subscribe_to_rectified_front_cam)
            frontCameraSub = n.subscribe("ardrone/front/image_rect_color", 1, &ARDrone::frontCameraCallback, this);
        else
            frontCameraSub = n.subscribe("ardrone/front/image_raw", 1, &ARDrone::frontCameraCallback, this);
        if (subscribe_to_rectified_bottom_cam)
            bottomCameraSub = n.subscribe("ardrone/bottom/image_rect_color", 1, &ARDrone::bottomCameraCallback, this);
        else
            bottomCameraSub = n.subscribe("ardrone/bottom/image_raw", 1, &ARDrone::bottomCameraCallback, this);

        commandsSub = n.subscribe("cmd_vel", 1, &ARDrone::commandsCallback, this);

    }


    //Start charts to publish
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        //Basic commands
        takeOffPubl = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
        landPubl = n.advertise<std_msgs::Empty>("ardrone/land", 1);
        resetPubl = n.advertise<std_msgs::Empty>("ardrone/reset", 1);



        //Start chart to publish commands
        commandsPubl = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }



    //Set commands to zero
    //this->setCommandToZero();




    return;
}


void ARDrone::close()
{
	return;
}


void ARDrone::navdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
	navData.header = msg->header;

    navData.altd=msg->altd;
    navData.ax=msg->ax;
    navData.ay=msg->ay;
    navData.az=msg->az;
    navData.batteryPercent=msg->batteryPercent;
    navData.magX=msg->magX;
    navData.magY=msg->magY;
    navData.magZ=msg->magZ;
    navData.pressure=msg->pressure;
    navData.rotX=msg->rotX;
    navData.rotY=msg->rotY;
    navData.rotZ=msg->rotZ;
    navData.state=msg->state;
    //TODO: tags
    navData.temp=msg->temp;
    navData.tm=msg->tm;
    navData.vx=msg->vx;
    navData.vy=msg->vy;
    navData.vz=msg->vz;
    navData.wind_angle=msg->wind_angle;
    navData.wind_comp_angle=msg->wind_comp_angle;
    navData.wind_speed=msg->wind_speed;

    navData.tm=msg->tm;

    run_timestamp = navData.header.stamp - init_timestamp;
    updateDroneDriverLogMsgStr();

    return;
}


void ARDrone::commandsCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    navCommand.angular.x=msg->angular.x;
    navCommand.angular.y=msg->angular.y;
    navCommand.angular.z=msg->angular.z;

    navCommand.linear.x=msg->linear.x;
    navCommand.linear.y=msg->linear.y;
    navCommand.linear.z=msg->linear.z;

    return;
}


void ARDrone::frontCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //printf("front aqui\n");
    //frontImage=Mat::zeros(540,480,CV_8UC3);

    //http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

    //cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cvFrontImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        frontImageRead=true;
    }
    catch (cv_bridge::Exception& e)
    {
        frontImageRead=false;
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    frontImage=cvFrontImage->image;
    frontImage_timesamp = msg->header.stamp;
    frontImage_seq = msg->header.seq;

    //We can only read one image at the same time
    bottomImageRead=false;

    //cv::imshow("Front Image", cvFrontImage->image);
    //cv::waitKey(1);

    return;
}


void ARDrone::bottomCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //printf("bottom aqui\n");
    //frontImage=Mat::zeros(540,480,CV_8UC3);

    //http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

    //cv_bridge::CvImagePtr cv_ptr;
    try
    {
        bottomImageRead=true;
        cvBottomImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        bottomImageRead=false;
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    bottomImage=cvBottomImage->image;
    bottomImage_timesamp = msg->header.stamp;
    bottomImage_seq = msg->header.seq;

    //We can only read one image at the same time
    frontImageRead=false;


    //cv::imshow("WINDOW", cv_ptr->image);
    //cv::waitKey(3);

    return;
}




void ARDrone::takeOff()
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        updateDroneDriverModeEventLogMsgStr("take_off");
        takeOffPubl.publish(emptyMsg);
    }

}


void ARDrone::land()
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        updateDroneDriverModeEventLogMsgStr("land");
        landPubl.publish(emptyMsg);
    }

}

void ARDrone::reset() // AR Drone - emergency stop. It works for both: entering and exiting emergency mode.
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        updateDroneDriverModeEventLogMsgStr("toggle_emergency");
        this->init();
        resetPubl.publish(emptyMsg);
    }

}


void ARDrone::hover()
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        updateDroneDriverModeEventLogMsgStr("hover");
        this->setNavCommandToZero();
    }
}


void ARDrone::move()
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        //geometry_msgs::Twist navCommandIn;

        navCommand.linear.x=0.0;
        navCommand.linear.y=0.0;
        navCommand.linear.z=0.0;
        navCommand.angular.x=1.0;
        navCommand.angular.y=1.0;
        navCommand.angular.z=0.0;

        //Set commands this angular commands to 1
        //navCommand.angular.x=1.0;
        //navCommand.angular.y=1.0;

        //Publish
        updateDroneDriverModeEventLogMsgStr("move");
        commandsPubl.publish(navCommand);
    }
}


void ARDrone::setNavCommandToZero()
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        //Set to zero
        //geometry_msgs::Twist navCommandIn;

        navCommand.linear.x=0.0;
        navCommand.linear.y=0.0;
        navCommand.linear.z=0.0;
        navCommand.angular.x=0.0;
        navCommand.angular.y=0.0;
        navCommand.angular.z=0.0;

        //Publish
        commandsPubl.publish(navCommand);
    }

}

void ARDrone::setNavCommand(geometry_msgs::Twist navCommandIn)
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        //Set
        navCommand.linear.x=navCommandIn.linear.x;
        navCommand.linear.y=navCommandIn.linear.y;
        navCommand.linear.z=navCommandIn.linear.z;
        navCommand.angular.z=navCommandIn.angular.z;

        //Saturation
        //Pitch
        if(navCommand.linear.x>LIMIT_COMMAND_PITCH)
            navCommand.linear.x=LIMIT_COMMAND_PITCH;
        if(navCommand.linear.x<-LIMIT_COMMAND_PITCH)
            navCommand.linear.x=-LIMIT_COMMAND_PITCH;

        //Roll
        if(navCommand.linear.y>LIMIT_COMMAND_ROLL)
            navCommand.linear.y=LIMIT_COMMAND_ROLL;
        if(navCommand.linear.y<-LIMIT_COMMAND_ROLL)
            navCommand.linear.y=-LIMIT_COMMAND_ROLL;

        //Height
        if(navCommand.linear.z>LIMIT_COMMAND_DHEIGHT)
            navCommand.linear.z=LIMIT_COMMAND_DHEIGHT;
        if(navCommand.linear.z<-LIMIT_COMMAND_DHEIGHT)
            navCommand.linear.z=-LIMIT_COMMAND_DHEIGHT;

        //Yaw
        if(navCommand.angular.z>LIMIT_COMMAND_DYAW)
            navCommand.angular.z=LIMIT_COMMAND_DYAW;
        if(navCommand.angular.z<-LIMIT_COMMAND_DYAW)
            navCommand.angular.z=-LIMIT_COMMAND_DYAW;

        //Publish
        commandsPubl.publish(navCommand);
    }

}

geometry_msgs::Twist ARDrone::getNavCommand(void)
{
    //if(parrotDriverType==full || parrotDriverType==monitor)
    //{
        return this->navCommand;
    //}
}



//JL:
//TODO: ajustar!!!!
void ARDrone::setDroneNavCommand(droneMsgs::droneNavCommand navCommandIn)
{
    //Change
    navCommand.linear.x  =-navCommandIn.pitch;
    navCommand.linear.y  =-navCommandIn.roll;
    navCommand.linear.z  = navCommandIn.dz;
    navCommand.angular.z =-navCommandIn.dyaw;
    //Set
    setNavCommand(navCommand);
    return;
}



//JL:
//TODO: ajustar!!!!
droneMsgs::droneNavCommand ARDrone::getDroneNavCommand(void)
{
    //Change of variables
    droneNavCommand.pitch =-navCommand.linear.x;
    droneNavCommand.roll  =-navCommand.linear.y;
    droneNavCommand.dz    = navCommand.linear.z;
    droneNavCommand.dyaw  =-navCommand.angular.z;
    //time
    droneNavCommand.time=ros::Time::now().toSec();

    //
    return this->droneNavCommand;
}




bool ARDrone::toggleCamera(void)
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        //Both false
        frontImageRead=false;
        bottomImageRead=false;
        //Send service
        if (toggleCamClient.call(emptySrvMsg))
        {
            return true;
            //ROS_INFO("Sum: %d", (bool)srv.response.result);
        }
        else
        {
            return false;
            //ROS_ERROR("Failed to call service ledanimation\n");
        }
    }
    else
        return false;
}

bool ARDrone::setCamChannel(uint8_t channelIn)
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        //Set service parameters
        if ( channelIn == 0 )
            updateDroneDriverModeEventLogMsgStr("use_front_camera");
        if ( channelIn == 1 )
            updateDroneDriverModeEventLogMsgStr("use_bottom_camera");
        setCamChannelSrvMsg.request.channel=channelIn;

        //Send service
        if (setCamChannelClient.call(setCamChannelSrvMsg))
        {
            return true;
            //ROS_INFO("Sum: %d", (bool)srv.response.result);
        }
        else
        {
            return false;
            //ROS_ERROR("Failed to call service ledanimation\n");
        }
    }
    else
        return false;
    return false;
}

bool ARDrone::setLedAnimation(uint8_t typeIn, float freqIn, uint8_t durationIn)
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        //Set service parameters
        ledAnimationSrvMsg.request.type=typeIn;
        ledAnimationSrvMsg.request.freq=freqIn;
        ledAnimationSrvMsg.request.duration=durationIn;

        //Send service
        if (ledAnimationClient.call(ledAnimationSrvMsg))
        {
            return true;
            //ROS_INFO("Sum: %d", (bool)srv.response.result);
        }
        else
        {
            return false;
            //ROS_ERROR("Failed to call service ledanimation\n");
        }
    }
    else
        return false;
    return false;
}


bool ARDrone::setFlightAnimation(uint8_t typeIn, uint16_t durationIn)
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        //Set service parameters
        flightAnimationSrvMsg.request.type=typeIn;
        flightAnimationSrvMsg.request.duration=durationIn;

        //Send service
        if (flightAnimationClient.call(flightAnimationSrvMsg))
        {
            return true;
            //ROS_INFO("Sum: %d", (bool)srv.response.result);
        }
        else
        {
            return false;
            //ROS_ERROR("Failed to call service ledanimation\n");
        }
    }
    else
        return false;
    return false;
}


//Imu calibration
bool ARDrone::calibrateImu(void)
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        //Send service
        if (imuCalibrationClient.call(emptySrvMsg))
        {
            return true;
            //ROS_INFO("Sum: %d", (bool)srv.response.result);
        }
        else
        {
            return false;
            //ROS_ERROR("Failed to call service ledanimation\n");
        }
    }
    else
        return false;
    return false;
}

//Flat Trim
bool ARDrone::flatTrim(void)
{
    if(parrotDriverType==parrotDriver::full || parrotDriverType==parrotDriver::drive)
    {
        //Send service
        if (FlatTrimClient.call(emptySrvMsg))
        {
            return true;
            //ROS_INFO("Sum: %d", (bool)srv.response.result);
        }
        else
        {
            return false;
            //ROS_ERROR("Failed to call service ledanimation\n");
        }
    }
    else
        return false;
    return false;
}


bool ARDrone::isFrontImageRead()
{
    return this->frontImageRead;
}


bool ARDrone::isBottomImageRead()
{
    return this->bottomImageRead;
}


bool ARDrone::displayFrontImage(std::string windowName)
{
    if(frontImageRead)
    {
        cv::imshow(windowName,frontImage);
        cv::waitKey(1);
        return true;
    }
    else
        return false;
}


bool ARDrone::displayBottomImage(std::string windowName)
{
    if(bottomImageRead)
    {
        //cout<<
        cv::imshow(windowName,bottomImage);
        cv::waitKey(1);
        return true;
    }
    else
        return false;

}



bool ARDrone::getFrontImage(cv::Mat* frontImageOut, uint32_t *pseq)
{
    if (pseq!=NULL) {
        (*pseq) = frontImage_seq;
    }
    if(frontImage.empty())
    {
        return false;
    }
    else
    {
        frontImage.copyTo(*frontImageOut);
        return true;
    }
}

bool ARDrone::getBottomImage(cv::Mat *bottomImageOut, uint32_t *pseq)
{
    if (pseq!=NULL) {
        (*pseq) = bottomImage_seq;
    }
    if(bottomImage.empty())
    {
        return false;
    }
    else
    {
        bottomImage.copyTo(*bottomImageOut);
        return true;
    }
}


cv::Mat ARDrone::getFrontImage()
{
    return frontImage;
}

cv::Mat ARDrone::getBottomImage()
{
    return bottomImage;
}

ros::Time ARDrone::getFrontImageTimestamp() {
    return frontImage_timesamp;
}

ros::Time ARDrone::getBottomImageTimestamp() {
    return bottomImage_timesamp;
}


ardrone_autonomy::Navdata ARDrone::getNavData()
{
    return navData;
}



droneMsgs::droneNavData ARDrone::getDroneNavData()
{
    //Convert
    // ardrone_autonomy README.md "altd: Estimated altitude (mm)"
    droneNavData.altitude =-navData.altd/1000.0;
    droneNavData.pitch    =-navData.rotY;
    droneNavData.roll     = navData.rotX;
    droneNavData.yaw      =-navData.rotZ;
    droneNavData.speedX   = navData.vx/1000.0;
    droneNavData.speedY   =-navData.vy/1000.0;
    droneNavData.time     = navData.tm;
    //Send
    return droneNavData;
}

void ARDrone::updateDroneDriverModeEventLogMsgStr( const std::string &event_mode_str ) {
    if ( (parrotDriverLoggerType == parrotDriver::logger) && (log_configuration_tag & PARROT_DRIVER_LOG_DRONEMODES) ) {
        event_timestamp = ros::Time::now() - init_timestamp;
        droneDriverModeEventLogMsgStrm
            /* timestamp   */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
            /* tag         */   << " [ardrone;event]"
            /* mode event  */   << " mode:" << event_mode_str
                                << std::endl;
    }
}

void ARDrone::updateDroneDriverLogMsgStr() {
    if ( (parrotDriverLoggerType == parrotDriver::logger) && (log_configuration_tag & PARROT_DRIVER_LOG_NAVDATA) ) {
        droneDriverLogMsgStrm
            /* timestamp   */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
            /* tag         */   << " [ardrone;state]"
            /* drone_mode  */   << " state:" << navdataState2String(navData.state)
            /* navdata     */   << " batt:"  << navData.batteryPercent  // percent
                                << " z:"     << -navData.altd/1000.0    // [m/s], mavwork reference frame
                                << " ax:"    <<  navData.ax             // [g],   mavwork reference frame
                                << " ay:"    << -navData.ay             // [g],   mavwork reference frame
                                << " az:"    << -navData.az             // [g],   mavwork reference frame
                                << " magX:"  <<  navData.magX           // [TBA?],mavwork reference frame
                                << " magY:"  << -navData.magY           // [TBA?],mavwork reference frame
                                << " magZ:"  << -navData.magZ           // [TBA?],mavwork reference frame
                                << " press:" <<  navData.pressure       // [TBA?]
                                << " pitch:" << -navData.rotY           // deg,   mavwork reference frame
                                << " roll:"  <<  navData.rotX           // deg,   mavwork reference frame
                                << " yaw:"   << -navData.rotZ           // deg,   mavwork reference frame
                                << " temp:"  << navData.temp            // [TBA?]
                                << " tm:"    << navData.tm              // us,    since ardrone boot
                                << " vx:"    <<  navData.vx/1000.0      // [m/s], mavwork reference frame
                                << " vy:"    << -navData.vy/1000.0      // [m/s], mavwork reference frame
                                << " vz:"    << -navData.vz/1000.0      // [m/s], mavwork reference frame
                                << " wind_angle:"       << navData.wind_angle       // [TBA?], reference frame???
                                << " wind_comp_angle:"  << navData.wind_comp_angle  // [TBA?], reference frame???
                                << " wind_speed:"       << navData.wind_speed;      // [TBA?], reference frame???

        droneMsgs::droneNavCommand droneNavCommand = getDroneNavCommand();
        droneDriverLogMsgStrm
            /* commands     */  // << " tc:"    << droneNavCommand.time
              // 1/1 values depend on configuration on ardrone_configured_mc4mavwork.launch
              // euler_angle_max 0.21 corresponds in reality to about 1 [1/1] > 24 [deg]
                                << " pitchc:"<< droneNavCommand.pitch   // 1/1
                                << " rollc:" << droneNavCommand.roll    // 1/1
                                << " dyawc:" << droneNavCommand.dyaw    // 1/1
                                << " daltdc:"<< droneNavCommand.dz;     // 1/1
        droneDriverLogMsgStrm
                                << std::endl;
    }
}

void ARDrone::getDroneDriverLogMsgStr( std::string &str_in ) {

    if (parrotDriverLoggerType == parrotDriver::logger) {
        str_in = droneDriverLogMsgStrm.str();
        droneDriverLogMsgStrm.str(std::string());
    } else {
        str_in = "";
    }
    return;
}

void ARDrone::getDroneDriverModeEventLogMsgStr( std::string &str_in ) {

    if (parrotDriverLoggerType == parrotDriver::logger) {
        str_in = droneDriverModeEventLogMsgStrm.str();
        droneDriverModeEventLogMsgStrm.str(std::string());
    } else {
        str_in = "";
    }
    return;
}

std::string ARDrone::navdataState2String( uint32_t state ) {

    switch(state) {
    case 0:
        return ("Unknown");
        break;
    case 1:
        return ("Init");
        break;
    case 2:
        return ("Landed");
        break;
    case 3:
        return ("Flying");
        break;
    case 4:
        return ("Hovering");
        break;
    case 5:
        return ("Test");
        break;
    case 6:
        return ("Taking_off");
        break;
    case 7:
        return ("Goto_Fix_Point");
        break;
    case 8:
        return ("Landing");
        break;
    case 9:
        return ("Looping");
        break;
    default:
        return ("???");
        break;
    }
}
