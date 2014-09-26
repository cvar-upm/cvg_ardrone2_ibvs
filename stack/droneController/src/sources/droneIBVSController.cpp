/*
 * droneIBVSController.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */


#include "droneIBVSController.h"

DroneIBVSController::DroneIBVSController(droneModule::droneModuleTypes droneModuleTypeIn, droneModule::droneModuleLoggerTypes droneModuleLoggerTypesIn) : DroneModule(droneModuleTypeIn,1.0/MULTIROTOR_IBVSCONTROLLER_TS)
{
    // logger => active
    if ( droneModuleType == droneModule::monitor )
        droneModuleLoggerType = droneModule::non_logger;
    else
        droneModuleLoggerType = droneModuleLoggerTypesIn;


    init();
    return;
}


DroneIBVSController::~DroneIBVSController()
{
    close();
    return;
}

void DroneIBVSController::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);



    if(droneModuleType==droneModule::active)
    {
        //Services
        setControlModeServerSrv=n.advertiseService(moduleName+"/setControlMode",&DroneIBVSController::setControlModeServCall,this);
        //Topics. Publishers
        droneNavCommandPubl = n.advertise<droneMsgs::droneNavCommand>(moduleName+"/droneNavCommand", 1);
        controlModePub=n.advertise<std_msgs::Int16>(moduleName+"/controlMode", 1);
        //Subscribers
        targetRelPositionRefsSub=n.subscribe(moduleName+"/dronePositionRefs", 1, &DroneIBVSController::targetRelPositionRefsSubCallback, this);
        targetRelSpeedRefsSub=n.subscribe(moduleName+"/droneSpeedsRefs", 1, &DroneIBVSController::targetRelSpeedRefsSubCallback, this);
        imFeaturesRefsSub=n.subscribe(moduleName+"/imFeaturesRefs", 1, &DroneIBVSController::imFeaturesRefsSubCallback, this);
    }

    if(droneModuleType==droneModule::monitor)
    {
        //Services
        setControlModeClientSrv=n.serviceClient<droneController::setControlMode>(moduleName+"/setControlMode");
        //Topics. Subscriber
        droneNavCommandSub=n.subscribe(moduleName+"/droneNavCommand", 1, &DroneIBVSController::droneNavCommandSubCallback, this);
        controlModeSub=n.subscribe(moduleName+"/controlMode", 1, &DroneIBVSController::controlModeSubCallback, this);
        //Publishers
        targetRelPositionRefsPub=n.advertise<droneMsgs::dronePose>(moduleName+"/dronePositionRefs", 1);
        targetRelSpeedRefsPub=n.advertise<droneMsgs::droneSpeeds>(moduleName+"/droneSpeedsRefs", 1);
        imFeaturesRefsPub=n.advertise<droneMsgs::imageFeaturesIBVS>(moduleName+"/imFeaturesRefs", 1);
    }



    //Flag of module opened
    droneModuleOpened=true;


    //End
    return;
}



void DroneIBVSController::init()
{
    //Control mode
    control_mode = MULTIROTOR_IBVSCONTROLLER_INIT_CONTROLMODE;

    //Filters
    pitch_lowpassfilter.setResponseTime(MULTIROTOR_TILT_REFERENCE_CUTOFF_TR);
    pitch_lowpassfilter.enableSaturation( true, -MULTIROTOR_IBVSCONTROLLER_MAX_PITCH, +MULTIROTOR_IBVSCONTROLLER_MAX_PITCH);
    pitch_lowpassfilter.reset();
    roll_lowpassfilter.setResponseTime(MULTIROTOR_TILT_REFERENCE_CUTOFF_TR);
    roll_lowpassfilter.enableSaturation( true, -MULTIROTOR_IBVSCONTROLLER_MAX_ROLL, +MULTIROTOR_IBVSCONTROLLER_MAX_ROLL);
    roll_lowpassfilter.reset();
    dyaw_lowpassfilter.setResponseTime(MULTIROTOR_DYAW_REFERENCE_CUTOFF_TR);
    dyaw_lowpassfilter.enableSaturation( true, -MULTIROTOR_IBVSCONTROLLER_DYAWMAX, +MULTIROTOR_IBVSCONTROLLER_DYAWMAX);
    dyaw_lowpassfilter.reset();
    dz_lowpassfilter.setResponseTime(MULTIROTOR_DALT_REFERENCE_CUTOFF_TR);
    dz_lowpassfilter.enableSaturation( true, -MULTIROTOR_IBVSCONTROLLER_DZMAX, +MULTIROTOR_IBVSCONTROLLER_DZMAX);
    dz_lowpassfilter.reset();

    pid_fx2R.setGains( 	MULTIROTOR_IBVSCONTROLLER_FX2R_KP,
                        MULTIROTOR_IBVSCONTROLLER_FX2R_KI,
                        MULTIROTOR_IBVSCONTROLLER_FX2R_KD);
    pid_fx2R.enableMaxOutput(true,  MULTIROTOR_IBVSCONTROLLER_MAX_ROLL);
    pid_fx2R.enableAntiWindup(true, MULTIROTOR_IBVSCONTROLLER_MAX_ROLL);

    pid_fx2DY.setGains( MULTIROTOR_IBVSCONTROLLER_FX2DY_KP,
                        MULTIROTOR_IBVSCONTROLLER_FX2DY_KI,
                        MULTIROTOR_IBVSCONTROLLER_FX2DY_KD);
    pid_fx2DY.enableMaxOutput(true,  MULTIROTOR_IBVSCONTROLLER_DYAWMAX);
    pid_fx2DY.enableAntiWindup(true, MULTIROTOR_IBVSCONTROLLER_DYAWMAX);

    pid_fy.setGains( 	MULTIROTOR_IBVSCONTROLLER_FY2DZ_KP,
                        MULTIROTOR_IBVSCONTROLLER_FY2DZ_KI,
                        MULTIROTOR_IBVSCONTROLLER_FY2DZ_KD);
    pid_fy.enableMaxOutput(true,  MULTIROTOR_IBVSCONTROLLER_DZMAX);
    pid_fy.enableAntiWindup(true, MULTIROTOR_IBVSCONTROLLER_DZMAX);

    pid_fs.setGains( 	MULTIROTOR_IBVSCONTROLLER_FS2P_KP,
                        MULTIROTOR_IBVSCONTROLLER_FS2P_KI,
                        MULTIROTOR_IBVSCONTROLLER_FS2P_KD);
    pid_fs.enableMaxOutput(true,  MULTIROTOR_IBVSCONTROLLER_MAX_PITCH);
    pid_fs.enableAntiWindup(true, MULTIROTOR_IBVSCONTROLLER_MAX_PITCH);

    pid_fD.setGains( 	MULTIROTOR_IBVSCONTROLLER_FD2P_KP,
                        MULTIROTOR_IBVSCONTROLLER_FD2P_KI,
                        MULTIROTOR_IBVSCONTROLLER_FD2P_KD);
    pid_fD.enableMaxOutput(true,  MULTIROTOR_IBVSCONTROLLER_MAX_PITCH);
    pid_fD.enableAntiWindup(true, MULTIROTOR_IBVSCONTROLLER_MAX_PITCH);

    //Controller
    //Position refs
    targetRelPositionRefs.time=ros::Time::now().toSec();
    targetRelPositionRefs.x=0.0;
    targetRelPositionRefs.y=0.0;
    targetRelPositionRefs.z=0.0;
    targetRelPositionRefs.yaw=0.0;
    targetRelPositionRefs.pitch=0.0;
    targetRelPositionRefs.roll=0.0;
    setTargetRelPositionRefs(targetRelPositionRefs);

    //Speeds refs
    targetRelSpeedRefs.time=ros::Time::now().toSec();
    targetRelSpeedRefs.dx=0.0;
    targetRelSpeedRefs.dy=0.0;
    targetRelSpeedRefs.dz=0.0;
    targetRelSpeedRefs.dyaw=0.0;
    targetRelSpeedRefs.dpitch=0.0;
    targetRelSpeedRefs.droll=0.0;
    setTargetRelSpeedRefs(targetRelSpeedRefs);

    //Init
    resetValues();

    run_timestamp  = ros::Duration(0.0);
    init_timestamp = ros::Time::now();

    // Constant to calculate distance2target from fxs, fys, fDs
    C_fx2Dy  = MULTIROTOR_FRONTCAM_C_fx2Dy;
    C_fx2DY  = MULTIROTOR_FRONTCAM_C_fx2DY;
    C_fy2Dz  = MULTIROTOR_FRONTCAM_C_fy2Dz;
    C_fD2Dx  = MULTIROTOR_FRONTCAM_C_fD2Dx;
    C_DY2Dfx = MULTIROTOR_FRONTCAM_C_DY2Dfx;
    C_DP2Dfy = MULTIROTOR_FRONTCAM_C_DP2Dfy;
    C_ObjVisArea = MULTIROTOR_IBVSCONTROLLER_TARGET_INIT_SIZE;
    // estimated distance to target
    Dxs = 0.0;
    Dys = 0.0;
    Dzs = 0.0;
    DYs = 0.0;

    // telemetry
    yaw_t   = 0.0;
    pitch_t = 0.0;
    roll_t  = 0.0;
    // centroid reference
    yaw_cr = 0.0;
    pitch_cr = 0.0;
    roll_cr = 0.0;

    ibvsControllerLogMsgStrm.str(std::string());
    updateIBVSControllerLogMsgStrWControllerGains();

    //End
    return;
}

void DroneIBVSController::close()
{

    DroneModule::close();
    return;
}


bool DroneIBVSController::resetValues()
{
    control_mode = MULTIROTOR_IBVSCONTROLLER_INIT_CONTROLMODE; //Speed control mode as default!!

    pitch_lowpassfilter.reset();
    roll_lowpassfilter.reset();
    dyaw_lowpassfilter.reset();
    dz_lowpassfilter.reset();

    pid_fx2R.reset();
    pid_fx2DY.reset();
    pid_fy.reset();
    pid_fs.reset();
    pid_fD.reset();

    // co ~ command outputs
    pitchco_hf = 0.0, rollco_hf = 0.0, dyawco_hf = 0.0, dzco_hf = 0.0;
    pitchco = 0.0, rollco = 0.0, dyawco = 0.0, dzco = 0.0;
    setNavCommandToZero();

    // image feature measurements and references
    fxci = MULTIROTOR_IBVSCONTROLLER_INITVAL_FX;
    fyci = MULTIROTOR_IBVSCONTROLLER_INITVAL_FY;
    fsci = MULTIROTOR_IBVSCONTROLLER_INITVAL_FS;
    fDci = MULTIROTOR_IBVSCONTROLLER_INITVAL_FD;
    fxs = fxci, fys = fyci, fss = fsci,  fDs = fDci;
    Dfx = 0.0, Dfy = 0.0, Dfs = 0.0, DfD = 0.0;
    Dfx_4DYC = 0.0, Dfx_4DyC = 0.0, Dfy_4DzC = 0.0, DfD_4DxC = 0.0;

    yaw_cr   = yaw_t;

    return true;

}



bool DroneIBVSController::startVal() {

//    yaw_cr   = yaw_t;
//    pitch_cr = 0.0;
//    roll_cr  = 0.0;

    if (target_is_on_frame)
        setControlModeVal( PERSON_FOLLOWING );
    else
        setControlModeVal( PERSON_NOT_ON_FRAME );

    return DroneModule::startVal();
}




bool DroneIBVSController::stopVal()
{
    //JL: publish null command!!
    setNavCommandToZero();
    setControlModeVal( CTRL_NOT_STARTED );
    //pmy_drone->setControlData( 0.0, 0.0, 0.0, 0.0);
    return DroneModule::stopVal();
}



void DroneIBVSController::setNavCommand(float roll, float pitch, float dyaw, float dz, double time)
{
    //Set time
    if(time<0.0)
    {
        double actualTime = ros::Time::now().toSec();
        droneNavCommandMsg.time=actualTime;
    }
    else
    {
        droneNavCommandMsg.time=time;
    }
    //Set others
    droneNavCommandMsg.dyaw=dyaw;
    droneNavCommandMsg.dz=dz;
    droneNavCommandMsg.pitch=pitch;
    droneNavCommandMsg.roll=roll;
    //Internal if active

    if(droneModuleType==droneModule::active)
    {
        pitchco=droneNavCommandMsg.pitch;
        rollco=droneNavCommandMsg.roll;
        dyawco=droneNavCommandMsg.dyaw;
        dzco=droneNavCommandMsg.dz;
    }

    //publish
    publishDroneNavCommand();
    //end
    return;
}

void DroneIBVSController::setNavCommandToZero(void)
{
    //set
    setNavCommand(0.0,0.0,0.0,0.0,-1.0);
    //End
    return;
}



droneMsgs::droneNavCommand DroneIBVSController::getNavCommand()
{
    return droneNavCommandMsg;
}


void DroneIBVSController::publishDroneNavCommand()
{
    if ((droneModuleType==droneModule::active) && (droneModuleOpened) )
    {
        //Publish
        droneNavCommandPubl.publish(droneNavCommandMsg);
    }
    //End
    return;
}


void DroneIBVSController::droneNavCommandSubCallback(const droneMsgs::droneNavCommand::ConstPtr &msg)
{
    droneNavCommandMsg.time=msg->time;
    droneNavCommandMsg.pitch=msg->pitch;
    droneNavCommandMsg.roll=msg->roll;
    droneNavCommandMsg.dyaw=msg->dyaw;
    droneNavCommandMsg.dz=msg->dz;
    return;
}


//void DroneIBVSController::setDronePose(droneMsgs::dronePose dronePoseIn)
//{
//    dronePose.x=dronePoseIn.x;
//    dronePose.y=dronePoseIn.y;
//    dronePose.z=dronePoseIn.z;
//    dronePose.yaw=dronePoseIn.yaw;
//    dronePose.pitch=dronePoseIn.pitch;
//    dronePose.roll=dronePoseIn.roll;

//    //internal value
//    if(droneModuleType==droneModule::active)
//    {
//        yaws = dronePose.yaw;
//        // Corregir uno de los valores de yaw para controlar correctamente
//        yaws = jesus_library::mapAnglesToBeNear_PIrads( yaws, yawci);

//        zs   = dronePose.z;
//        xs  = dronePose.x;
//        ys  = dronePose.y;
//    }

//    return;
//}


//void DroneIBVSController::setDroneSpeeds(droneMsgs::droneSpeeds droneSpeedsIn)
//{
//    droneSpeeds.dx=droneSpeedsIn.dx;
//    droneSpeeds.dy=droneSpeedsIn.dy;
//    droneSpeeds.dz=droneSpeedsIn.dz;
//    droneSpeeds.dyaw=droneSpeedsIn.dyaw;
//    droneSpeeds.dpitch=droneSpeedsIn.dpitch;
//    droneSpeeds.droll=droneSpeedsIn.droll;

//    //internal value
//    if(droneModuleType==droneModule::active)
//    {
//        vzs = droneSpeeds.dz;
//        vxs  = droneSpeeds.dx;
//        vys  = droneSpeeds.dy;
//        dyaws = droneSpeeds.dyaw;
//    }
//    return;
//}



void DroneIBVSController::targetRelPositionRefsSubCallback(const droneMsgs::dronePose::ConstPtr &msg)
{
    //General value
    targetRelPositionRefs.time=msg->time;
    targetRelPositionRefs.x=msg->x;
    targetRelPositionRefs.y=msg->y;
    targetRelPositionRefs.z=msg->z;
    targetRelPositionRefs.yaw=msg->yaw;
    targetRelPositionRefs.pitch=msg->pitch;
    targetRelPositionRefs.roll=msg->roll;
    //internal value
    if(droneModuleType==droneModule::active)
    {
//        xci = targetRelPositionRefs.x;
//        yci = targetRelPositionRefs.y;
//        yawci = atan2( sin(targetRelPositionRefs.yaw), cos(targetRelPositionRefs.yaw));
//        zci  = targetRelPositionRefs.z;
        if (targetRelPositionRefs.pitch>0 && targetRelPositionRefs.roll>0)  {
            // hover_stop_or_reset_references == false, set_yaw_to_zero == false
            float Dxc, Dyc, Dzc, Dyawc;
            Dxc = targetRelPositionRefs.x;
            Dyc = targetRelPositionRefs.y;
            Dyawc = atan2( sin(targetRelPositionRefs.yaw), cos(targetRelPositionRefs.yaw));
            Dzc  = targetRelPositionRefs.z;
            setImFeatReferencesFromDPos( Dxc, Dyc, Dzc, Dyawc);
        } else {
            if (targetRelPositionRefs.pitch<0 && targetRelPositionRefs.roll<0)  {
                // hover_stop_or_reset_references == false, set_yaw_to_zero == true
                fxci = 0.5;
            } else {
                // hover_stop_or_reset_references == true, set_yaw_to_zero == false
                if ( isStarted() ) {
                    fxci = MULTIROTOR_IBVSCONTROLLER_INITVAL_FX;
                    fyci = MULTIROTOR_IBVSCONTROLLER_INITVAL_FY;
                    fsci = MULTIROTOR_IBVSCONTROLLER_INITVAL_FS;
                    fDci = MULTIROTOR_IBVSCONTROLLER_INITVAL_FD;
                } else {
                    setNavCommandToZero();
                }
            }
        }
    }

    return;
}


void DroneIBVSController::getTargetRelPositionRefs( double &xci_out, double &yci_out, double &yawci_out, double &zci_out) {
    xci_out = targetRelPositionRefs.x;
    yci_out = targetRelPositionRefs.y;
    yawci_out=targetRelPositionRefs.yaw;
    zci_out = targetRelPositionRefs.z;
    return;
}

droneMsgs::dronePose DroneIBVSController::getTargetRelPositionRefs()
{
    //End
    return targetRelPositionRefs;
}


void DroneIBVSController::setTargetRelPositionRefs(float &Dxci_in, float &Dyci_in, float &Dyawci_in, float &Dzci_in, bool hover_stop_or_reset_references, bool set_yaw_to_zero) {

    //Update value
    targetRelPositionRefs.time = ros::Time::now().toSec();
    targetRelPositionRefs.x    = Dxci_in;
    targetRelPositionRefs.y    = Dyci_in;
    targetRelPositionRefs.z    = Dzci_in;
    targetRelPositionRefs.yaw  = Dyawci_in;
    targetRelPositionRefs.pitch = 0.0;
    targetRelPositionRefs.roll  = 0.0;
    if (!hover_stop_or_reset_references) {
        targetRelPositionRefs.pitch= 0.1;
        targetRelPositionRefs.roll = 0.1;
    } else {
        if (set_yaw_to_zero) {
            targetRelPositionRefs.pitch= -0.1;
            targetRelPositionRefs.roll = -0.1;
        }
    }

    setTargetRelPositionRefs(targetRelPositionRefs);
}

void DroneIBVSController::setTargetRelPositionRefs(droneMsgs::dronePose targetRelPositionRefsIn)
{
    //Update value
    targetRelPositionRefs.time=ros::Time::now().toSec();
    targetRelPositionRefs.x=targetRelPositionRefsIn.x;
    targetRelPositionRefs.y=targetRelPositionRefsIn.y;
    targetRelPositionRefs.z=targetRelPositionRefsIn.z;
    targetRelPositionRefs.yaw=targetRelPositionRefsIn.yaw;
    targetRelPositionRefs.pitch=targetRelPositionRefsIn.pitch;
    targetRelPositionRefs.roll=targetRelPositionRefsIn.roll;

    if(droneModuleType==droneModule::active)
    {
        // TODO_JP Maybe the changes were not be done here
        //Update internal value
        //Controller commands
        float Dxc, Dyc, Dzc, Dyawc;
        Dxc = targetRelPositionRefs.x;
        Dyc = targetRelPositionRefs.y;
        Dyawc = atan2( sin(targetRelPositionRefs.yaw), cos(targetRelPositionRefs.yaw));
        Dzc  = targetRelPositionRefs.z;
        setImFeatReferencesFromDPos( Dxc, Dyc, Dzc, Dyawc);
        //Feedforward commands
        pitchfi = targetRelPositionRefs.pitch;
        rollfi = targetRelPositionRefs.roll;
    }
    else
    {
        //Publish
        if (droneModuleOpened)
            targetRelPositionRefsPub.publish(targetRelPositionRefs);
    }

    //end
    return;
}


//void DroneIBVSController::setXReference( double xci_in)
//{

//    //saveCntRefData2Disk();

//    //if ( controllerMutex.lock() )
//    //{
//        targetRelPositionRefs.x=xci_in;

//        setTargetRelPositionRefs(targetRelPositionRefs);

//    //    saveCntRefData2Disk();
//    //	controllerMutex.unlock();
//    //}
//}

//void DroneIBVSController::setYReference( double yci_in)
//{

//    //saveCntRefData2Disk();

//    //if ( controllerMutex.lock() )
//    //{
//        targetRelPositionRefs.y=yci_in;

//        setTargetRelPositionRefs(targetRelPositionRefs);

//    //    saveCntRefData2Disk();
//    //	controllerMutex.unlock();
//    //}
//}

//void DroneIBVSController::setYawReference( double yawci_in)
//{

//    //saveCntRefData2Disk();

//    //if ( controllerMutex.lock() )
//    //{
//        targetRelPositionRefs.yaw=atan2( sin(yawci_in), cos(yawci_in));


//        setTargetRelPositionRefs(targetRelPositionRefs);

//    //    saveCntRefData2Disk();
//    //	controllerMutex.unlock();
//    //}
//}


//void DroneIBVSController::setZReference( double zci_in)
//{
//    //saveCntRefData2Disk();

//    //if ( controllerMutex.lock() )
//    //{
//        targetRelPositionRefs.z = zci_in;

//        setTargetRelPositionRefs(targetRelPositionRefs);

//    //    saveCntRefData2Disk();
//    //	controllerMutex.unlock();
//    //}
//}


//void DroneIBVSController::setPitchRollReference( double pitchfi_in, double rollfi_in)
//{
//    //saveCntRefData2Disk();

//    //if ( controllerMutex.lock() )
//    //{
//        //pitchfi = pitchfi_in;
//        //rollfi  =  rollfi_in;

//        targetRelPositionRefs.pitch=pitchfi_in;
//        targetRelPositionRefs.roll=rollfi_in;


//        setTargetRelPositionRefs(targetRelPositionRefs);

//    //    saveCntRefData2Disk();
//    //	controllerMutex.unlock();
//    //}
//}





void DroneIBVSController::targetRelSpeedRefsSubCallback(const droneMsgs::droneSpeeds::ConstPtr &msg)
{
    //Update value
    targetRelSpeedRefs.time=msg->time;
    targetRelSpeedRefs.dx=msg->dx;
    targetRelSpeedRefs.dy=msg->dy;
    targetRelSpeedRefs.dz=msg->dz;
    targetRelSpeedRefs.dyaw=msg->dyaw;
    targetRelSpeedRefs.dpitch=msg->dpitch;
    targetRelSpeedRefs.droll=msg->droll;
    if(droneModuleType==droneModule::active)
    {
        //Feedforward speeds
        vxfi = targetRelSpeedRefs.dx;
        vyfi = targetRelSpeedRefs.dy;
        dyawfi = targetRelSpeedRefs.dyaw;
        dzfi   = targetRelSpeedRefs.dz;
    }

    return;
}

void DroneIBVSController::getTargetRelSpeedRefs( double &vxfi_out, double &vyfi_out ) {
    vxfi_out = targetRelSpeedRefs.dx;
    vyfi_out = targetRelSpeedRefs.dy;
    return;
}

droneMsgs::droneSpeeds DroneIBVSController::getTargetRelSpeedRefs()
{
    return targetRelSpeedRefs;
}


void DroneIBVSController::setTargetRelSpeedRefs(droneMsgs::droneSpeeds targetRelSpeedRefsIn)
{
    //Update value
    targetRelSpeedRefs.time  = ros::Time::now().toSec();
    targetRelSpeedRefs.dx    = targetRelSpeedRefsIn.dx;
    targetRelSpeedRefs.dy    = targetRelSpeedRefsIn.dy;
    targetRelSpeedRefs.dz    = targetRelSpeedRefsIn.dz;
    targetRelSpeedRefs.dyaw  = targetRelSpeedRefsIn.dyaw;
    targetRelSpeedRefs.dpitch= targetRelSpeedRefsIn.dpitch;
    targetRelSpeedRefs.droll = targetRelSpeedRefsIn.droll;

    if(droneModuleType==droneModule::active)
    {
        //Update internal value
        vxfi = targetRelSpeedRefs.dx;
        vyfi = targetRelSpeedRefs.dy;
        dyawfi = targetRelSpeedRefs.dyaw;
        dzfi   = targetRelSpeedRefs.dz;
    }
    else
    {
        //Publish
        if (droneModuleOpened)
            targetRelSpeedRefsPub.publish(targetRelSpeedRefs);
    }
    //end
    return;
}


void DroneIBVSController::setVFIReference( double vxfi_in, double vyfi_in)
{
    //saveCntRefData2Disk();

    //if ( controllerMutex.lock() )
    //{
        targetRelSpeedRefs.dx = vxfi_in;
        targetRelSpeedRefs.dy = vyfi_in;

        setTargetRelSpeedRefs(targetRelSpeedRefs);

    //   saveCntRefData2Disk();
    //	controllerMutex.unlock();
    //}
}

void DroneIBVSController::setVXFIReference( double vxfi_in)
{
    //saveCntRefData2Disk();

    //if ( controllerMutex.lock() )
    //{
        targetRelSpeedRefs.dx = vxfi_in;

        setTargetRelSpeedRefs(targetRelSpeedRefs);

    //    saveCntRefData2Disk();
    //	controllerMutex.unlock();
    //}
}

void DroneIBVSController::setVYFIReference( double vyfi_in)
{
    //saveCntRefData2Disk();

    //if ( controllerMutex.lock() )
    //{
        targetRelSpeedRefs.dy = vyfi_in;
        setTargetRelSpeedRefs(targetRelSpeedRefs);

    //    saveCntRefData2Disk();
    //	controllerMutex.unlock();
    //}
}



void DroneIBVSController::setDYawFIReference( double dyawfi_in)
{
    //saveCntRefData2Disk();

    //if ( controllerMutex.lock() )
    //{
        targetRelSpeedRefs.dyaw = dyawfi_in;
        setTargetRelSpeedRefs(targetRelSpeedRefs);

    //    saveCntRefData2Disk();
    //	controllerMutex.unlock();
    //}
}

void DroneIBVSController::setDZFIReference( double dzfi_in)
{
    //saveCntRefData2Disk();

    //if ( controllerMutex.lock() )
    //{
        targetRelSpeedRefs.dz= dzfi_in;
        setTargetRelSpeedRefs(targetRelSpeedRefs);

    //   saveCntRefData2Disk();
    //	controllerMutex.unlock();
    //}
}



// ***** Set controller reference functions *****
void DroneIBVSController::setAllReferences( double xci_in, double yci_in, double yawci_in, double zci_in,
        double vxfi_t , double vyfi_t , double dyawfi_t , double dzfi_t ,
        double pitchfi_t , double rollfi_t )
{

    //saveCntRefData2Disk();

    //if ( controllerMutex.lock() )
    //{
        targetRelPositionRefs.x = xci_in;
        targetRelPositionRefs.y = yci_in;
        targetRelPositionRefs.z  = zci_in;
        targetRelPositionRefs.yaw = atan2( sin(yawci_in), cos(yawci_in));

        targetRelPositionRefs.pitch = pitchfi_t;
        targetRelPositionRefs.roll = rollfi_t;

        setTargetRelPositionRefs(targetRelPositionRefs);


        targetRelSpeedRefs.dx = vxfi_t;
        targetRelSpeedRefs.dy = vyfi_t;
        targetRelSpeedRefs.dz   = dzfi_t;
        targetRelSpeedRefs.dyaw = dyawfi_t;


        setTargetRelSpeedRefs(targetRelSpeedRefs);

    //    saveCntRefData2Disk();
    //	controllerMutex.unlock();
    //}
}

// TODO_JL: this function has an ambiguous meaning, as the contents of the variable
// TODO_JL: dronePose are different is the droneController object is a "monitor" or an "active".
// TODO_JL:     "active" : dronePose contains the controller feedback
// TODO_JL:     "monitor": dronePose contains the controller references
//void DroneIBVSController::setAllReferences2ActualValues() {
//    setAllReferences( dronePose.x, dronePose.y, dronePose.yaw, dronePose.z, 0.0, 0.0, 0.0, 0.0);
//}




// ***** Get/Accessors Controller functions *****

//// adds funtionality to bool DroneModule::stop(), by reseting the controller to its internal state
//bool DroneController::stop() {
//    resetValues();
//    return DroneModule::stop();
//}
// adds funtionality to bool DroneModule::stop(), by reseting the controller to its internal state
//bool DroneIBVSController::start() {
//    if ( !isStarted() ) {
//        resetValues();
//    }
//    return DroneModule::start();
//}


bool DroneIBVSController::setControlMode(controlMode mode) {
    //printf("aqui\n");
    if(droneModuleType==droneModule::active)
    {
        return setControlModeVal(mode);
    }
    else
    {
        //Prepare msg
        setControlModeSrv.request.controlMode = controlModeToInt( mode );

        //use service
        if(setControlModeClientSrv.call(setControlModeSrv))
        {
            return setControlModeSrv.response.ack;
        }
        else
        {
            return false;
        }
    }
    return false;
}


bool DroneIBVSController::setControlModeServCall(droneController::setControlMode::Request& request, droneController::setControlMode::Response& response)
{
    DroneIBVSController::controlMode control_mode_enum = MULTIROTOR_IBVSCONTROLLER_INIT_CONTROLMODE;

    control_mode_enum = controlModeFromInt( request.controlMode );
    response.ack=setControlModeVal(control_mode_enum);

    return response.ack;
}

bool DroneIBVSController::setControlModeVal(controlMode mode)
{
    if (control_mode != mode) {

        switch (mode) {
        case CTRL_NOT_STARTED:
            break;
        case PERSON_FOLLOWING:
            // independetly of the prior control_mode do...
            resetValues();
            break;
        case PERSON_NOT_ON_FRAME:
            break;
        default:
            return false;
            break;
        }

        control_mode = mode;
    }
    return true;
}




void DroneIBVSController::controlModeSubCallback(const std_msgs::Int16::ConstPtr &msg)
{
    control_mode = controlModeFromInt(msg->data);

    return;
}


void DroneIBVSController::controlModePublish()
{
    if(droneModuleType==droneModule::active)
    {

        controlModeMsg.data = controlModeToInt( control_mode );

        if (droneModuleOpened)
            controlModePub.publish(controlModeMsg);
    }
    return;
}

DroneIBVSController::controlMode DroneIBVSController::controlModeFromInt( int controlMode_int) {
    DroneIBVSController::controlMode control_mode_enum = MULTIROTOR_IBVSCONTROLLER_INIT_CONTROLMODE;

    switch (controlMode_int) {
    case 0: //Default
        control_mode_enum= MULTIROTOR_IBVSCONTROLLER_INIT_CONTROLMODE;
        break;
    case 1:
        control_mode_enum= CTRL_NOT_STARTED;
        break;
    case 2:
        control_mode_enum= PERSON_FOLLOWING;
        break;
    case 3:
        control_mode_enum= PERSON_NOT_ON_FRAME;
        break;
    default:
        control_mode_enum= MULTIROTOR_IBVSCONTROLLER_INIT_CONTROLMODE;
        break;
    }

    return control_mode_enum;
}

int DroneIBVSController::controlModeToInt( controlMode controlMode_enum) {
    int controlMode_int = 1;

    switch (controlMode_enum) {
    case CTRL_NOT_STARTED:
        controlMode_int = 1;
        break;
    case PERSON_FOLLOWING:
        controlMode_int = 2;
        break;
    case PERSON_NOT_ON_FRAME:
        controlMode_int = 3;
        break;
    default:
        controlMode_int = 1;
        break;
    }

    return controlMode_int;
}



DroneIBVSController::controlMode DroneIBVSController::getControlMode()
{

    controlMode current_control_mode;
    //if ( controllerMutex.lock() ) {
        current_control_mode = control_mode;
    //	controllerMutex.unlock();
    //}
    return current_control_mode;
}
// END: ***** Get/Accessors Controller functions *****

bool DroneIBVSController::run()
{
    isStartedPublish();
    controlModePublish();

    bool run_successful = true;
    run_timestamp = ros::Time::now() - init_timestamp;
    updateDistanceToTarget();

    float diff_yaw = 0.0;
    int   sign_diff_yaw = 0;
    switch (control_mode) {
    case CTRL_NOT_STARTED:
        pitchco = 0.0;
        rollco  = 0.0;
        dyawco  = 0.0;
        dzco    = 0.0;
        run_successful = false;
        break;
    case PERSON_FOLLOWING:
        // update yaw_cr, diff_yaw = yaw_t-yaw_cr;
        sign_diff_yaw = (cos(yaw_t)*sin(yaw_cr) - cos(yaw_cr)*sin(yaw_t))>0 ? +1 : -1;
        diff_yaw = sign_diff_yaw*acos(cos(yaw_t)*cos(yaw_cr) + sin(yaw_t)*sin(yaw_cr));
        if ( fabs(diff_yaw)>(25.0*M_PI/180.0)) {
            yaw_cr   = yaw_t;
            diff_yaw = 0.0;
        }

        // Calculate image feature tracking error
        Dfx = fxci - fxs;
        Dfy = fyci - fys;
        Dfs = fsci - fss;
        DfD = fDci - fDs;
        Dfx_4DYC = Dfx;
        Dfx_4DyC = Dfx + C_DY2Dfx*diff_yaw;
        Dfy_4DzC = Dfy + C_DP2Dfy*(pitch_t - pitch_cr);
        DfD_4DxC = DfD;
        fxs_4DyC = fxci - Dfx_4DyC;
        fxs_4DYC = fxci - Dfx_4DYC;
        fys_4DzC = fyci - Dfy_4DzC;
        fDs_4DxC = fDci - DfD_4DxC;

        // set PID input, and calculate ouput
        pid_fx2R.setReference(fxci);
        pid_fx2R.setFeedback( fxs_4DyC );
        rollco_hf = pid_fx2R.getOutput();

        pid_fx2DY.setReference(fxci);
        pid_fx2DY.setFeedback( fxs_4DYC );
        dyawco_hf = pid_fx2DY.getOutput();

        pid_fy.setReference(fyci);
        pid_fy.setFeedback( fys_4DzC );
        dzco_hf = pid_fy.getOutput();

        pid_fs.setReference(fsci);          // out-dated
        pid_fs.setFeedback( fss );          // out-dated
//        pitchco_hf = pid_fs.getOutput();  // out-dated

        pid_fD.setReference(fDci);
        pid_fD.setFeedback( fDs_4DxC );
        pitchco_hf = pid_fD.getOutput();

        // (low pass) filter the command outputs
        pitch_lowpassfilter.setInput(pitchco_hf);
        pitchco = pitch_lowpassfilter.getOutput();
        roll_lowpassfilter.setInput(rollco_hf);
        rollco = roll_lowpassfilter.getOutput();
        dyaw_lowpassfilter.setInput(dyawco_hf);
        dyawco = dyaw_lowpassfilter.getOutput();
        // Uncomment to switch off yaw control
//        dyawco = 0.0;
        dz_lowpassfilter.setInput(dzco_hf);
        dzco = dz_lowpassfilter.getOutput();

        // Send commands to the multirotor
        setNavCommand(rollco,pitchco,dyawco,dzco,-1.0);
        run_successful = true;
        break;
    case PERSON_NOT_ON_FRAME:
        // HYP_JP: I think that (HOVER == setNavCommandToZero());
        pitchco = 0.0;
        rollco  = 0.0;
        dyawco  = 0.0;
        dzco    = 0.0;
        setNavCommandToZero();
        run_successful = true;
        break;
    default:
        run_successful = false;
        break;
    }

    updateIBVSControllerLogMsgStr();

#ifdef MULTIROTOR_IBVSCONTROLLER_DEBUG
    std::cout
        /* timestamp   */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
        /* tag         */   << " [ctrlr;state]"
        /* isStarted   */   << " started:" << isStarted()
        /* controlMode */   << " mode:"    << controlMode2String()
        /* tof         */   << " tof:"     << target_is_on_frame
        /* references  */   << " fxci:" << fxci
                            << " fyci:" << fyci
                            << " fsci:" << fsci
                            << " fDci:" << fDci
        /* dstnce2trget*/   << " Dxs:" << Dxs
                            << " Dys:" << Dys
                            << " Dzs:" << Dzs
                            << " DYs:" << DYs
        /* commands    */   << " Pco:"  << pitchco
                            << " Rco:"  << rollco
                            << " Dzco:" << dzco
                            << " DYco:" << dyawco
//          /* feedback    */   << " fxs:" << fxs
//                              << " fys:" << fys
//                              << " fss:" << fss
//                              << " fDs:" << fDs
                            << std::endl;
#endif // MULTIROTOR_IBVSCONTROLLER_DEBUG

    return run_successful;
}

bool DroneIBVSController::boundingBox2ImageFeatures( const int bb_x, const int bb_y, const int bb_width, const int bb_height,
                                float &fx, float &fy, float &fs, float &fD, const bool target_is_on_frame_in ) {

    bool error_ocurred = false;
    if ( (bb_width == 0) || (bb_height == 0) || (!target_is_on_frame_in) ) {
        error_ocurred = true;   // degenerate bounding box
        return error_ocurred;
    }

    // [fx]horizontal and [fy]vertical position of the centroid on the image
    fx = ( (float) bb_x + ((float) bb_width)/2.0 )/MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH;
    fy = ( (float) bb_y + ((float)bb_height)/2.0 )/MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT;

    // [fs]size and [fD]inverse sqrt of size of object in the image
    fs = (((float)bb_width)/MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH)*(((float)bb_height)/MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT);
    fD = 1.0/sqrt( (float)fs );

    return error_ocurred;
}

void DroneIBVSController::imFeaturesRefsSubCallback(const droneMsgs::imageFeaturesIBVS ::ConstPtr &msg) {

    ros::Time timestamp = ros::Time::now();

    //Update value
    imFeaturesRefs.time=msg->time;
    imFeaturesRefs.fx=msg->fx;
    imFeaturesRefs.fy=msg->fy;
    imFeaturesRefs.fs=msg->fs;
    imFeaturesRefs.fD=msg->fD;
    if(droneModuleType==droneModule::active)
    {
        //Feedforward speeds
        fxci = imFeaturesRefs.fx;
        fyci = imFeaturesRefs.fy;
        fsci = imFeaturesRefs.fs;
        fDci = imFeaturesRefs.fD;
    }

    return;
}

void DroneIBVSController::setImFeaturesRefs(droneMsgs::imageFeaturesIBVS  imFeaturesRefsIn) { //Publish function

//    setImFeatReferences(  imFeaturesRefsIn.fx, imFeaturesRefsIn.fy, imFeaturesRefsIn.fs, imFeaturesRefsIn.fD);

    ros::Time timestamp = ros::Time::now();

    //Update value
    imFeaturesRefs.time  = timestamp.toSec();
    imFeaturesRefs.fx    = imFeaturesRefsIn.fx;
    imFeaturesRefs.fy    = imFeaturesRefsIn.fy;
    imFeaturesRefs.fs    = imFeaturesRefsIn.fs;
    imFeaturesRefs.fD    = imFeaturesRefsIn.fD;

    if(droneModuleType==droneModule::active)
    {
        //Update internal value
        fxci = imFeaturesRefs.fx;
        fyci = imFeaturesRefs.fy;
        fsci = imFeaturesRefs.fs;
        fDci = imFeaturesRefs.fD;

    }
    else
    {
        //Publish
        if (droneModuleOpened)
            imFeaturesRefsPub.publish(imFeaturesRefs);
    }
    //end

    return;
}

droneMsgs::imageFeaturesIBVS DroneIBVSController::getImFeaturesRefs() {
    return imFeaturesRefs;
}

void DroneIBVSController::getImFeatReferences(         float &fxci_out,     float &fyci_out,    float  &fsci_out,     float &fDci_out) {

    fxci_out = fxci;
    fyci_out = fyci;
    fsci_out = fsci;
    fDci_out = fDci;

    return;
}

 void DroneIBVSController::getImFeatFeedback2PIDs( float &fxs_4DyC_out, float &fxs_4DYC_out, float &fys_4DzC_out, float &fDs_4DxC_out) {

     if ( isStarted() ) {
         fxs_4DyC_out = fxs_4DyC;
         fxs_4DYC_out = fxs_4DYC;
         fys_4DzC_out = fys_4DzC;
         fDs_4DxC_out = fDs_4DxC;
     } else {
         fxs_4DyC_out = fxs;
         fxs_4DYC_out = fxs;
         fys_4DzC_out = fys;
         fDs_4DxC_out = fDs;
     }

     return;
 }

bool DroneIBVSController::setImFeatReferences(   const float fxci_in, const float fyci_in, const float fsci_in, const float fDci_in) {

    bool error_ocurred = false;

    //    if ( (0.0 < fxci_in) && (fxci_in < 1.0) ||
    //         (0.0 < fyci_in) && (fyci_in < 1.0) ||
    //         (0.0 < fsci_in) && (fsci_in < 1.0) ||
    //         (0.0 < fDci_in)                      )  { // Note: fD can be >1.0 since: fD = 1.0/sqrt(fs)
    //        error_ocurred = true;
    //        return error_ocurred;
    //    }

    ros::Time timestamp = ros::Time::now();

    //Update value
    imFeaturesRefs.time  = timestamp.toSec();
    imFeaturesRefs.fx    = fxci_in;
    imFeaturesRefs.fy    = fyci_in;
    imFeaturesRefs.fs    = fsci_in;
    imFeaturesRefs.fD    = fDci_in;

    setImFeaturesRefs( imFeaturesRefs );

//    if(droneModuleType==droneModule::active)
//    {
//        //Update internal value
//        fxci = imFeaturesRefs.fx;
//        fyci = imFeaturesRefs.fy;
//        fsci = imFeaturesRefs.fs;
//        fDci = imFeaturesRefs.fD;

//        if (droneModuleLoggerType == droneModule::logger) {
//            imFeaturesMsgStrm << timestamp.sec << "." << timestamp.nsec << " [ctr][ref]"
//                    << " fx:" << imFeaturesRefs.fx
//                    << " fy:" << imFeaturesRefs.fy
//                    << " fs:" << imFeaturesRefs.fs
//                    << " fD:" << imFeaturesRefs.fD << std::endl;
//        }
//    }
//    else
//    {
//        //Publish
//        if (droneModuleOpened)
//            imFeaturesRefsPub.publish(imFeaturesRefs);
//    }

    return error_ocurred;
}

bool DroneIBVSController::setImFeatMeasurements( const float fxs_in,  const float fys_in,  const float fss_in,  const float fDs_in, const int bb_width_in, const int bb_heigth_in) {

    bool error_ocurred = false;
    // podría comprobar que todos los valores estan entre 0 y 1 (fD solo >0.0), de momento no lo voy a hacer

    //internal value
    if(droneModuleType==droneModule::active){
        fxs = fxs_in;
        fys = fys_in;
        fss = fss_in;
        fDs = fDs_in;
        bb_width  = bb_width_in;
        bb_heigth = bb_heigth_in;
    }

    return error_ocurred;
}

void DroneIBVSController::setTargetIsOnFrame( const bool target_is_on_frame_in) {

    if (droneModuleType==droneModule::active) {
        target_is_on_frame = target_is_on_frame_in;
        if ( isStarted() ) {
            if ( target_is_on_frame ) {
                setControlModeVal( PERSON_FOLLOWING );
            } else {
                setControlModeVal( PERSON_NOT_ON_FRAME );
            }
        }
    }
}

float DroneIBVSController::distanceToTarget( const float fD) {

    float depth;
    depth = sqrt(MULTIROTOR_FRONTCAM_ALPHAX*MULTIROTOR_FRONTCAM_ALPHAY*MULTIROTOR_IBVSCONTROLLER_TARGET_INIT_SIZE/(MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH*MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT)) * fD;

    return depth;

}

void DroneIBVSController::setTelemetryAttitude_rad( double yaw_t_in, double pitch_t_in, double roll_t_in) {
    yaw_t = yaw_t_in;
    pitch_t = pitch_t_in;
    roll_t = roll_t_in;
}

bool DroneIBVSController::setImFeatReferencesFromDPos( float Dxc, float Dyc, float Dzc, float DYc) {

    bool forced_safety_image_area = false;

    float Dfxci, Dfyci, DfDci; // Dfsci
    Dfxci = Dyc/C_fx2Dy + DYc/C_fx2DY;
    Dfyci = Dzc/C_fy2Dz;
    DfDci = Dxc/C_fD2Dx;

    // MULTIROTOR_IBVSCONTROLLER_SAFETY_DISTANCE_TO_IMAGE_BORDER
    // MULTIROTOR_IBVSCONTROLLER_SAFETY_TARGET_IMAGE_AREA
    float fxci_aux = fxci + Dfxci;
    float safety_distance_from_center = (0.5-MULTIROTOR_IBVSCONTROLLER_SAFETY_DISTANCE_TO_IMAGE_BORDER-bb_width/MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH);
    float sign = (fxci_aux-0.5) > 0? 1.0 : -1.0;
    if ( safety_distance_from_center > 0 ) {
        if ( sign*(fxci_aux-0.5) > safety_distance_from_center ) {
            // saturate
            fxci = 0.5 + sign*safety_distance_from_center;
            forced_safety_image_area = true;
        } else {
            // don't saturate
            fxci = fxci_aux;
        }
    } else {
        fxci = 0.5;
        forced_safety_image_area = true;
    }

    float fyci_aux = fyci + Dfyci;
    safety_distance_from_center = (0.5-MULTIROTOR_IBVSCONTROLLER_SAFETY_DISTANCE_TO_IMAGE_BORDER-bb_heigth/MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT);
    sign = (fyci_aux-0.5) > 0? 1.0 : -1.0;
    if ( safety_distance_from_center > 0 ) {
        if ( sign*(fyci_aux-0.5) > safety_distance_from_center ) {
            // saturate
            fyci = 0.5 + sign*safety_distance_from_center;
            forced_safety_image_area = true;
        } else {
            // don't saturate
            fyci = fyci_aux;
        }
    } else {
        fyci = 0.5;
        forced_safety_image_area = true;
    }

    float fDci_aux = fDci + DfDci;
    float fsci_aux = 1/(fDci_aux*fDci_aux);
    if ( fsci_aux > MULTIROTOR_IBVSCONTROLLER_SAFETY_TARGET_IMAGE_AREA ) {
        // saturate
        fDci_aux = 1/sqrt(MULTIROTOR_IBVSCONTROLLER_SAFETY_TARGET_IMAGE_AREA);
        fDci = fDci_aux;
        forced_safety_image_area = true;
    } else {
        // don't saturate
        fDci = fDci_aux;
    }

    return forced_safety_image_area;
}


void DroneIBVSController::updateIBVSControllerLogMsgStr() {

    if ( droneModuleLoggerType == droneModule::logger ) {
        ibvsControllerLogMsgStrm
            /* timestamp   */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
            /* tag         */   << " [ctrlr;state]"
            /* isStarted   */   << " started:" << isStarted()
            /* controlMode */   << " mode:"    << controlMode2String()
            /* tof         */   << " tof:"     << target_is_on_frame
            /* references  */   << " fxci:" << fxci
                                << " fyci:" << fyci
                                << " fsci:" << fsci
                                << " fDci:" << fDci
            /* dstnce2trget*/   << " Dxs:" << Dxs
                                << " Dys:" << Dys
                                << " Dzs:" << Dzs
                                << " DYs:" << DYs
            /* commands    */   << " Pco:"  << pitchco
                                << " Rco:"  << rollco
                                << " dzco:" << dzco
                                << " dYco:" << dyawco
            /* feedback    */   << " fxs:" << fxs
                                << " fys:" << fys
                                << " fss:" << fss
                                << " fDs:" << fDs
                                << " fxs_4DyC:" << fxs_4DyC
                                << " fxs_4DYC:" << fxs_4DYC
                                << " fys_4DzC:" << fys_4DzC
                                << " fDs_4DxC:" << fDs_4DxC
                                << " yaw_t:"    << yaw_t
//                                << " pitch_t:"  << pitch_t
//                                << " roll_t:"   << roll_t
                                << " yaw_cr:"    << yaw_cr
//                                << " pitch_cr:" << pitch_cr
//                                << " roll_cr:"  << roll_cr
                                << std::endl;
    }
}

void DroneIBVSController::updateIBVSControllerLogMsgStrWControllerGains() {

    if ( droneModuleLoggerType == droneModule::logger ) {
        ibvsControllerLogMsgStrm
                /* timestamp   */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
                /* tag         */   << " [ctrlr;gains]";
        double Kp, Ki, Kd;
        pid_fx2R.getGains( Kp, Ki, Kd);
        ibvsControllerLogMsgStrm
                /* ctrlr fx2R  */   << " Kp_fx2R:" << Kp
                                    << " Ki_fx2R:" << Ki
                                    << " Kd_fx2R:" << Kd;
        pid_fx2DY.getGains( Kp, Ki, Kd);
        ibvsControllerLogMsgStrm
                /* ctrlr fx2DY */   << " Kp_fx2DY:" << Kp
                                    << " Ki_fx2DY:" << Ki
                                    << " Kd_fx2DY:" << Kd;
        pid_fy.getGains( Kp, Ki, Kd);
        ibvsControllerLogMsgStrm
                /* ctrlr fy2dz */   << " Kp_fy:" << Kp
                                    << " Ki_fy:" << Ki
                                    << " Kd_fy:" << Kd;
        pid_fD.getGains( Kp, Ki, Kd);
        ibvsControllerLogMsgStrm
                /* ctrlr fD2P  */   << " Kp_fD:" << Kp
                                    << " Ki_fD:" << Ki
                                    << " Kd_fD:" << Kd;
        ibvsControllerLogMsgStrm
                /*dstnce2trget cts*/<< " C_fx2Dy:" << C_fx2Dy
                                    << " C_fx2DY:" << C_fx2DY
                                    << " C_fy2Dz:" << C_fy2Dz
                                    << " C_fD2Dx:" << C_fD2Dx
                                    << " C_ObjVisArea:" << C_ObjVisArea
                                    << std::endl;
    }
}

void DroneIBVSController::updateDistanceToTarget() {

    // TODO_JP: What happens if fDs == 0 (event which I believe impossible??
    // TODO_JP: Are C_[...] constants always >0 ?
    Dxs = C_fD2Dx*fDs;
    // TODO_JP: I am not yet sure if we want to make this correction on real time
    C_fx2Dy = MULTIROTOR_FRONTCAM_C_fx2Dy * (Dxs/MULTIROTOR_IBVSCONTROLLER_INIT_DEPTH);
    C_fy2Dz = MULTIROTOR_FRONTCAM_C_fy2Dz * (Dxs/MULTIROTOR_IBVSCONTROLLER_INIT_DEPTH);
    Dys = C_fx2Dy*(fxs - 0.5);
    Dzs = C_fy2Dz*(fys - 0.5);
    DYs = C_fx2DY*(fxs - 0.5);
}

void DroneIBVSController::getIBVSControllerLogMsgStr( std::string &str_in ) {

    if (droneModuleLoggerType == droneModule::logger) {
        str_in = ibvsControllerLogMsgStrm.str();
        ibvsControllerLogMsgStrm.str(std::string());
    } else {
        str_in = "";
    }
    return;
}

std::string DroneIBVSController::controlMode2String() {

    switch( control_mode ) {
    case DroneIBVSController::CTRL_NOT_STARTED:
        return ("stopped");
        break;
    case DroneIBVSController::PERSON_FOLLOWING:
        return ("TOF_IBVS");
        break;
    case DroneIBVSController::PERSON_NOT_ON_FRAME:
        return ("TNOF_IBVS");
        break;
    default:
        return ("unknown_IBVS_control_mode");
        break;
    }
}
