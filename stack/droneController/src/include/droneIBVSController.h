/*
 * drobeIBVSController.h
 *
 *  Created on: Jun 20, 2013
 *      Author: jespestana
 */

#ifndef DRONE_IBVS_CONTROLLER_H
#define DRONE_IBVS_CONTROLLER_H

// General includes
#include <math.h>

////// ROS  ///////
#include "ros/ros.h"
//Drone module
#include "droneModule.h"
//Messages
#include "droneMsgs/droneNavCommand.h"
#include "droneMsgs/dronePose.h"
#include "droneMsgs/droneSpeeds.h"
#include "droneMsgs/imageFeaturesIBVS.h"
#include "std_msgs/Int16.h" //Control mode
//Services
#include "droneController/setControlMode.h"

// Other resources
#include <atlante.h>
#include "controller/other/PID.h"
#include "controller/other/LowPassFilter.h"
#include "Other/jesus_library.h"

// OpenTLD
//#include "tld_msgs/BoundingBox.h"
//#include "std_msgs/Float32.h"
#include "droneopentldinterface.h"

#define MULTIROTOR_IBVSCONTROLLER_TS                    (1.0/15.0)
#define MULTIROTOR_IBVSCONTROLLER_INIT_CONTROLMODE   CTRL_NOT_STARTED

// Max values for action commands in "tanto por 1"
#define MULTIROTOR_IBVSCONTROLLER_DYAWMAX 		0.5
#define MULTIROTOR_IBVSCONTROLLER_DZMAX 		0.5
#define MULTIROTOR_IBVSCONTROLLER_MAX_PITCH 	0.5
#define MULTIROTOR_IBVSCONTROLLER_MAX_ROLL  	0.5

// Controller - tilt reference low pass filter configuration
#define MULTIROTOR_TILT_REFERENCE_CUTOFF_TR 		0.30	// seg
#define MULTIROTOR_DYAW_REFERENCE_CUTOFF_TR 		-0.30	// seg
#define MULTIROTOR_DALT_REFERENCE_CUTOFF_TR 		-0.30	// seg

// Camera related constans - for parrot AR Drone 2.0
#define MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH    640.0
#define MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT   360.0
#define MULTIROTOR_FRONTCAM_HORIZONTAL_ANGLE_OF_VIEW    70.0   // deg
#define MULTIROTOR_FRONTCAM_VERTICAL_ANGLE_OF_VIEW      38.0   // deg
#define MULTIROTOR_FRONTCAM_ALPHAX              460.0
#define MULTIROTOR_FRONTCAM_ALPHAY              530.0
#define MULTIROTOR_IBVSCONTROLLER_INIT_DEPTH        3.0         // m
#define MULTIROTOR_IBVSCONTROLLER_TARGET_INIT_SIZE  (0.4*0.3)   // m
#define MULTIROTOR_FAERO_DCGAIN_SPEED2TILT      (1.0/7.0)       // [1/1]/[m/s] when tilt = 1 [1/1] := 24 [deg]

// Distance to tarte estimation constants
#define MULTIROTOR_FRONTCAM_C_fx2Dy     ( MULTIROTOR_IBVSCONTROLLER_INIT_DEPTH*MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH/(MULTIROTOR_FRONTCAM_ALPHAX) )
#define MULTIROTOR_FRONTCAM_C_fx2DY     ( MULTIROTOR_FRONTCAM_HORIZONTAL_ANGLE_OF_VIEW * (M_PI/180.0) )
#define MULTIROTOR_FRONTCAM_C_fy2Dz     ( MULTIROTOR_IBVSCONTROLLER_INIT_DEPTH*MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT/(MULTIROTOR_FRONTCAM_ALPHAY) )
#define MULTIROTOR_FRONTCAM_C_fD2Dx     ( sqrt( (MULTIROTOR_FRONTCAM_ALPHAX*MULTIROTOR_FRONTCAM_ALPHAY*MULTIROTOR_IBVSCONTROLLER_TARGET_INIT_SIZE)/(MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH*MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT) ) )
#define MULTIROTOR_FRONTCAM_C_DY2Dfx    ( 1/(MULTIROTOR_FRONTCAM_C_fx2DY) )
#define MULTIROTOR_FRONTCAM_C_DP2Dfy    ( 1/( MULTIROTOR_FRONTCAM_VERTICAL_ANGLE_OF_VIEW * (M_PI/180.0) ) )

// PID Gains
#define MULTIROTOR_IBVSCONTROLLER_FX2R_DELTA_KP     1.0
#define MULTIROTOR_IBVSCONTROLLER_FX2R_KP  (((-0.5) * MULTIROTOR_FRONTCAM_C_fx2Dy  * MULTIROTOR_IBVSCONTROLLER_FX2R_DELTA_KP ) * MULTIROTOR_FAERO_DCGAIN_SPEED2TILT )
#define MULTIROTOR_IBVSCONTROLLER_FX2R_KI   (( 0.0) * MULTIROTOR_IBVSCONTROLLER_FX2R_KP )
#define MULTIROTOR_IBVSCONTROLLER_FX2R_KD   ((1.62*0.3) * MULTIROTOR_IBVSCONTROLLER_FX2R_KP )   // *0.3, Added 19 Aug 2013

#define MULTIROTOR_IBVSCONTROLLER_FD2P_DELTA_KP     1.0
#define MULTIROTOR_IBVSCONTROLLER_FD2P_KP  (((+0.5) * MULTIROTOR_FRONTCAM_C_fD2Dx * MULTIROTOR_IBVSCONTROLLER_FD2P_DELTA_KP ) * MULTIROTOR_FAERO_DCGAIN_SPEED2TILT )
#define MULTIROTOR_IBVSCONTROLLER_FD2P_KI   (( 0.0) * MULTIROTOR_IBVSCONTROLLER_FD2P_KP )
#define MULTIROTOR_IBVSCONTROLLER_FD2P_KD   ((1.62*0.3) * MULTIROTOR_IBVSCONTROLLER_FD2P_KP )   // *0.3, Added 19 Aug 2013

#define MULTIROTOR_IBVSCONTROLLER_FX2DY_DELTA_KP    (0.9)
#define MULTIROTOR_IBVSCONTROLLER_FX2DY_KP ((-0.9) * MULTIROTOR_FRONTCAM_C_fx2DY * MULTIROTOR_IBVSCONTROLLER_FX2DY_DELTA_KP )
#define MULTIROTOR_IBVSCONTROLLER_FX2DY_KI (( 0.0) * MULTIROTOR_IBVSCONTROLLER_FX2DY_KP )
#define MULTIROTOR_IBVSCONTROLLER_FX2DY_KD ((0.12) * MULTIROTOR_IBVSCONTROLLER_FX2DY_KP )

#define MULTIROTOR_IBVSCONTROLLER_FY2DZ_DELTA_KP    1.0
#define MULTIROTOR_IBVSCONTROLLER_FY2DZ_KP ((+0.7) * MULTIROTOR_FRONTCAM_C_fy2Dz * MULTIROTOR_IBVSCONTROLLER_FY2DZ_DELTA_KP )
#define MULTIROTOR_IBVSCONTROLLER_FY2DZ_KI (( 0.0) * MULTIROTOR_IBVSCONTROLLER_FY2DZ_KP )
#define MULTIROTOR_IBVSCONTROLLER_FY2DZ_KD ((0.26) * MULTIROTOR_IBVSCONTROLLER_FY2DZ_KP )

// The gains for the Fs2P controller are neither well calculated nor tested
#define MULTIROTOR_IBVSCONTROLLER_FS2P_DELTA_KP     0.0
#define MULTIROTOR_IBVSCONTROLLER_FS2P_KP  ((-0.5) * MULTIROTOR_IBVSCONTROLLER_FS2P_DELTA_KP)
#define MULTIROTOR_IBVSCONTROLLER_FS2P_KI  (( 0.0) * MULTIROTOR_IBVSCONTROLLER_FS2P_KP )
#define MULTIROTOR_IBVSCONTROLLER_FS2P_KD  ((1.62) * MULTIROTOR_IBVSCONTROLLER_FS2P_KP )

#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FX    0.5
#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FY    0.5
//#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FS    0.019 // follow ardrone box
#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FS    0.025 // AVG
//#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FS    0.0073  // IROS13 test1 and 2
//#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FS    0.012
//#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FS    0.006 // Demos instituto y SSRR13 0.009
#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FD    ( 1.0/sqrt(MULTIROTOR_IBVSCONTROLLER_INITVAL_FS) )
#define MULTIROTOR_IBVSCONTROLLER_SAFETY_DISTANCE_TO_IMAGE_BORDER 0.25
#define MULTIROTOR_IBVSCONTROLLER_SAFETY_TARGET_IMAGE_AREA pow((1-2*MULTIROTOR_IBVSCONTROLLER_SAFETY_DISTANCE_TO_IMAGE_BORDER),2)

//#define MULTIROTOR_IBVSCONTROLLER_DEBUG

class DroneIBVSController : public DroneModule
{	
public:
//    // adds funtionality to bool DroneModule::stop(), by reseting the controller to its internal state
//    bool stop();
//    bool start();

    ////// Control mode
public: // control mode definition
    enum controlMode
    {
        CTRL_NOT_STARTED = 1
        , PERSON_FOLLOWING = 2
        , PERSON_NOT_ON_FRAME = 3
//		, MIDDLE_SIZE_OBJECT_HOVERING
//      , OTHER_MODES
    };
private:
    //Active control mode
    controlMode control_mode;
    // Services
    ros::ServiceServer setControlModeServerSrv;
    ros::ServiceClient setControlModeClientSrv;
    droneController::setControlMode setControlModeSrv;
    //Set control mode
public:
    bool setControlMode(controlMode mode);
private:
    bool setControlModeServCall(droneController::setControlMode::Request& request, droneController::setControlMode::Response& response);
protected:
    bool setControlModeVal(controlMode mode);
private:
//    void setPositionControl();
//    void setSpeedControl();
    //Get control mode
public:
    controlMode getControlMode();
private:
    ros::Publisher controlModePub;
    ros::Subscriber controlModeSub;
    std_msgs::Int16 controlModeMsg;
    void controlModeSubCallback(const std_msgs::Int16::ConstPtr &msg);
    void controlModePublish();
protected:
    controlMode controlModeFromInt( int controlMode_int);
    int         controlModeToInt( controlMode controlMode_enum);





    /////// Estimated position and speed of the UAV (from IBVS)
    // Drone Pose
private:
    droneMsgs::dronePose dronePose;
public:
//    void setDronePose(droneMsgs::dronePose dronePoseIn);
    // Drone speeds
private:
    droneMsgs::droneSpeeds droneSpeeds;
public:
//    void setDroneSpeeds(droneMsgs::droneSpeeds droneSpeedsIn);
	







    ///// droneNavCommand: controller commands
private:
    ros::Publisher droneNavCommandPubl;
    ros::Subscriber droneNavCommandSub;
    droneMsgs::droneNavCommand droneNavCommandMsg;
private:
    void publishDroneNavCommand(void);
    void droneNavCommandSubCallback(const droneMsgs::droneNavCommand::ConstPtr &msg);

protected:
    void setNavCommand(float roll, float pitch, float dyaw, float dz, double time=-1.0);
    void setNavCommandToZero();
public:
    droneMsgs::droneNavCommand getNavCommand();
    //JL: sustituida por getNavCommand
    //void getLastOutput( double *pitchco_out, double *rollco_out, double *dyawco_out, double *dzco_out);





    //////// drone References: referencias del controlador

    ////Position refs
private:
    droneMsgs::dronePose targetRelPositionRefs;
    ros::Publisher targetRelPositionRefsPub;
    ros::Subscriber targetRelPositionRefsSub;
protected:
    void targetRelPositionRefsSubCallback(const droneMsgs::dronePose::ConstPtr &msg);
    void setTargetRelPositionRefs(droneMsgs::dronePose targetRelPositionRefsIn); //Publish function
public:
    //JL: get Position references. changed
    void getTargetRelPositionRefs( double &xci_out, double &yci_out, double &yawci_out, double &zci_out);
    droneMsgs::dronePose getTargetRelPositionRefs();
public:
    void setTargetRelPositionRefs(float &Dxci_in, float &Dyci_in, float &Dyawci_in, float &Dzci_in, bool hover_stop_or_reset_references = false, bool set_yaw_to_zero=false);
//    void setXReference( double xci_in);
//    void setYReference( double yci_in);
//    void setYawReference( double yawci_in);
//    void setZReference( double zci_in);
//    void setPitchRollReference( double pitchfi_in, double rollfi_in);



    ////Speed refs
private:
    droneMsgs::droneSpeeds targetRelSpeedRefs;
    ros::Publisher targetRelSpeedRefsPub;
    ros::Subscriber targetRelSpeedRefsSub;
protected:
    void targetRelSpeedRefsSubCallback(const droneMsgs::droneSpeeds::ConstPtr &msg);
    void setTargetRelSpeedRefs(droneMsgs::droneSpeeds speedsRefsIn); //Publish function
public:
    void getTargetRelSpeedRefs( double &vxfi_out, double &vyfi_out );
    droneMsgs::droneSpeeds getTargetRelSpeedRefs();
public:
    void setVFIReference( double vxfi_in, double vyfi_in);
    void setVXFIReference( double vxfi_in);
    void setVYFIReference( double vyfi_in);
    void setDYawFIReference( double dyawfi_in);
    void setDZFIReference( double dzfi_in);


    ////Position and speed refs
protected:
    void setAllReferences( double xci_in, double yci_in, double yawci_in, double zci_in,
            double vxfi_t = 0.0, double vyfi_t = 0.0, double dyawfi_t = 0.0, double dzfi_t = 0.0,
            double pitchfi_t = 0.0, double rollfi_t = 0.0);
public:
//    void setAllReferences2ActualValues();


    // ***** BEGIN: Internal variables in the controller block diagram *****
private:
    // Controller command inputs
    // ci ~ command inputs
    double xci, yci, yawci, zci;
    // fi ~ feedforward inputs
    double pitchfi, rollfi;             //Position
    double vxfi, vyfi, dyawfi, dzfi;    //=Speed ci

    // co ~ command outputs (to pelican proxy)
    double pitchco_hf, rollco_hf, dyawco_hf, dzco_hf; // hf ~ "high frequency", before (low pass) filtering
    double pitchco, rollco, dyawco, dzco;             // co ~ controller output, sent to the AR Drone

    CVG_BlockDiagram::LowPassFilter pitch_lowpassfilter, roll_lowpassfilter, dyaw_lowpassfilter, dz_lowpassfilter;
    CVG_BlockDiagram::PID   pid_fx2R, pid_fx2DY, pid_fy, pid_fs, pid_fD;
    double C_fx2Dy, C_fx2DY, C_fy2Dz, C_fD2Dx;  // Constant to calculate distance2target from fxs, fys, fDs
    double C_DY2Dfx, C_DP2Dfy;
    double yaw_t, pitch_t, roll_t;      // telemetry
    double yaw_cr, pitch_cr, roll_cr;   // centroid reference
    double C_ObjVisArea;                        // m^2
    double Dxs, Dys, Dzs, DYs; // estimated distance to target
    // ***** END: Internal variables in the controller block diagram *****


public:
    DroneIBVSController(droneModule::droneModuleTypes droneModuleTypeIn=droneModule::monitor, droneModule::droneModuleLoggerTypes droneModuleLoggerTypesIn = droneModule::non_logger);
    ~DroneIBVSController();

    void open(ros::NodeHandle & nIn, std::string moduleName);
    void init();
    void close();


    //Reset
protected:
    bool resetValues();

    //Start
protected:
    bool startVal();

    //Stop
protected:
    bool stopVal();


public:
    bool run();


    // IBVS Controller - using OpenTLD
public:
    // [fx]horizontal and [fy]vertical position of the centroid on the image
    // [fs]size and [fD]inverse sqrt of size of object in the image
    bool boundingBox2ImageFeatures( const int bb_x, const int bb_y, const int bb_width, const int bb_height,
                                    float &fx, float &fy, float &fs, float &fD, const bool target_is_on_frame_in );

private:
    // input layers
    float  fxs,  fys,  fss,   fDs;   // image feature measurements
    float fxci, fyci, fsci,  fDci;   // image feature references
    bool   target_is_on_frame;
    // internals
    float Dfx, Dfy, Dfs, DfD;
    // Values after correction
    float Dfx_4DYC, Dfx_4DyC, Dfy_4DzC, DfD_4DxC;
    float fxs_4DyC, fxs_4DYC, fys_4DzC, fDs_4DxC;

    // output layers
    // defined above on Internal variables in the controller block diagram "block of the code"
private:
    droneMsgs::imageFeaturesIBVS imFeaturesRefs;
    ros::Publisher  imFeaturesRefsPub;
    ros::Subscriber imFeaturesRefsSub;
protected:
    void imFeaturesRefsSubCallback(const droneMsgs::imageFeaturesIBVS ::ConstPtr &msg);
    void setImFeaturesRefs(droneMsgs::imageFeaturesIBVS  imFeaturesRefsIn); //Publish function
public:
    droneMsgs::imageFeaturesIBVS getImFeaturesRefs();
    void getImFeatReferences(         float &fxci_out,     float &fyci_out,    float  &fsci_out,     float &fDci_out);
    void getImFeatFeedback2PIDs( float &fxs_4DyC_out, float &fxs_4DYC_out, float &fys_4DzC_out, float &fDs_4DxC_out);
    bool setImFeatReferences(   const float fxci_in, const float fyci_in, const float fsci_in, const float fDci_in);
    bool setImFeatMeasurements( const float fxs_in,  const float fys_in,  const float fss_in,  const float fDs_in, const int bb_width, const int bb_heigth);
    void setTargetIsOnFrame( const bool target_is_on_frame_in);
    float distanceToTarget( const float fD);
    void setTelemetryAttitude_rad( double yaw_t_in, double pitch_t_in, double roll_t_in);
private:
    int bb_width, bb_heigth;
    bool setImFeatReferencesFromDPos( float Dxc, float Dyc, float Dzc, float Dyawc);

    // DroneLogger - Module required definitions/declarations
private:
    ros::Duration run_timestamp;
    ros::Time     init_timestamp;
    std::ostringstream ibvsControllerLogMsgStrm;
    void updateIBVSControllerLogMsgStr();
    void updateIBVSControllerLogMsgStrWControllerGains();
    void updateDistanceToTarget();
public:
    inline void setInitTimestamp( const ros::Time &init_timestamp_in) { init_timestamp = init_timestamp_in; }
    void        getIBVSControllerLogMsgStr( std::string &str_in );
    std::string controlMode2String();
};
        
        
        
 

#endif /* DRONE_IBVS_CONTROLLER_H_ */
