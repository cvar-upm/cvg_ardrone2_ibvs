/*
*
*
*
*
*/


#ifndef DRONE_IBVS_BRAIN_H
#define DRONE_IBVS_BRAIN_H


/// ROS
#include "ros/ros.h"


#include <stdio.h>
#include <iostream>



//module
#include "droneModule.h"


//controller
#include "droneIBVSController.h"

// OpenTLD
#include "droneopentldinterface.h"

//State estimator
#include "droneStateEstimator.h"
#include "communication_definition.h"

#define FREQ_BRAIN  5.0


class DroneIBVSBrain : public DroneModule {

public:
    DroneIBVSController   droneIBVSController;
    DroneOpenTLDInterface openTLDInterface;
    DroneStateEstimator TheDroneStateEstimator;

public:
    DroneIBVSBrain(droneModule::droneModuleTypes droneModuleTypeIn=droneModule::monitor);
    ~DroneIBVSBrain();

    void init();

    void open(ros::NodeHandle & nIn, std::string moduleName);
    void close();

public:
    bool run();

};



#endif // DRONE_IBVS_BRAIN_H
