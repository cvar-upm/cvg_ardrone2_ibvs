/*
 * droneModule.h
 *
 *  Created on: 
 *      Author: jl.sanchez
 */

#ifndef DRONE_MODULE_H
#define DRONE_MODULE_H



////// ROS  ///////
#include "ros/ros.h"




//Cpp
#include <sstream>
#include <string>
#include <iostream>






//Services
#include "std_srvs/Empty.h"
//#include "droneModule/isStarted.h"


//Topics
#include "std_msgs/Bool.h"


namespace droneModule
{
    enum droneModuleTypes
    {
        active, //Writes
        monitor //Reads
    };
    enum droneModuleLoggerTypes
    {
        non_logger, // logs data
        logger      // does not log data
    };
}



class Module
{
    //Ros node
protected:
    ros::NodeHandle n;

protected:
    std::string stackPath;



    //Module vars
protected:
    //module started
    bool moduleStarted;
    //module opened
    bool droneModuleOpened;


protected:
    //Type of module
    droneModule::droneModuleTypes droneModuleType;
    //Rate
    ros::Rate moduleRate;




    //Reset
public:
    bool reset();
private:
    std_srvs::Empty emptySrv;
    ros::ServiceServer resetServerSrv;
    ros::ServiceClient resetClientSrv;
    bool resetServCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
protected:
    virtual bool resetValues();


    //Start
public:
    bool start();
private:
    //std_srvs::Empty emptySrv; //Compartido. Ya puesto antes
    bool startServCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    ros::ServiceServer startServerSrv;
    ros::ServiceClient startClientSrv;
protected:
    virtual bool startVal();


    //Stop
public:
    bool stop();
private:
    //std_srvs::Empty emptySrv; //Compartido. Ya puesto antes
    bool stopServCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    ros::ServiceServer stopServerSrv;
    ros::ServiceClient stopClientSrv;
protected:
    virtual bool stopVal();


    //is Started
public:
    bool isStarted();
private:
    //bool isStartedServCall(droneModule::isStarted::Request& request, droneModule::isStarted::Response& response);
    void isStartedSubCallback(const std_msgs::Bool::ConstPtr &msg);
protected:
    void isStartedPublish();
    //Topics
private:
    std_msgs::Bool isStartedMsg;
    ros::Publisher isStartedPub;
    ros::Subscriber isStartedSub;



public:
    Module(droneModule::droneModuleTypes droneModuleTypeIn=droneModule::monitor, double moduleRateIn=0.0, droneModule::droneModuleLoggerTypes droneModuleLoggerTypesIn = droneModule::non_logger);
    ~Module();

    virtual void open(ros::NodeHandle & nIn, std::string moduleName);

protected:
    void init();
    void close();


    //Run
public:
    virtual bool run();

    //Sleep
public:
    void sleep();



    // droneLogger
protected:
    // pointer to logging function
    droneModule::droneModuleLoggerTypes droneModuleLoggerType;
//    void (DroneLogger::*ploggingfunction)( const std::string &);
//    DroneLogger *pdronelogger;
public:
//    inline void setLoggingFunction( void (DroneLogger::*ploggingfunctionIn)( const std::string &) ) { ploggingfunction = ploggingfunctionIn; }
//    inline void setDroneLogger( DroneLogger *pdroneloggerIn ) { pdronelogger = pdroneloggerIn; }



};




class DroneModule : public Module
{	


protected:
    int idDrone;
    std::string stringId;






public:

    DroneModule(droneModule::droneModuleTypes droneModuleTypeIn=droneModule::monitor, double moduleRateIn=0.0, droneModule::droneModuleLoggerTypes droneModuleLoggerTypesIn = droneModule::non_logger);
    ~DroneModule();


    void open(ros::NodeHandle & nIn, std::string moduleName);
    std::string getStringId() {return stringId;}
    int getId() {return idDrone;}

};
        
        
        
 

#endif /* DRONE_MODULE_H_ */
