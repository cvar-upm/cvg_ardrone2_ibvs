/*
*
*
*
*
*/





//
#include "droneIBVSBrain.h"




DroneIBVSBrain::DroneIBVSBrain(droneModule::droneModuleTypes droneModuleTypeIn) : DroneModule(droneModuleTypeIn,FREQ_BRAIN),
    droneIBVSController(droneModule::monitor),
    openTLDInterface(),
    TheDroneStateEstimator(droneModule::monitor)
{
    init();
    return;
}


DroneIBVSBrain::~DroneIBVSBrain()
{
    close();
    return;
}


void DroneIBVSBrain::init()
{
    return;
}


void DroneIBVSBrain::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);


    //Open sub-cores
    droneIBVSController.open(n,MODULE_NAME_TRAJECTORY_CONTROLLER);
    openTLDInterface.open(n,"ardrone");
    TheDroneStateEstimator.open(n,MODULE_NAME_ODOMETRY_STATE_ESTIMATOR);

    //Return
    return;

}


void DroneIBVSBrain::close()
{

    droneIBVSController.close();
//    openTLDInterface.close();
    TheDroneStateEstimator.close();

    DroneModule::close();

    return;

}




bool DroneIBVSBrain::run() {
    if(droneModuleType==droneModule::active) {
        DroneModule::run();
    }
    return true;
}
