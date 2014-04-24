/*
 * droneModule.cpp
 *
 *  Created on: 
 *      Author: jl.sanchez
 */


#include "droneModule.h"



Module::Module(droneModule::droneModuleTypes droneModuleTypeIn, double moduleRateIn, droneModule::droneModuleLoggerTypes droneModuleLoggerTypesIn) :
    //n("~")
    moduleRate(moduleRateIn)
//  , ploggingfunction(NULL)
//  , pdronelogger(NULL)
  , droneModuleLoggerType(droneModuleLoggerTypesIn)
{
    droneModuleType=droneModuleTypeIn;
    init();
    return;
}


Module::~Module()
{
    close();
    return;
}

void Module::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    n=nIn;







    ros::param::get("~stackPath",stackPath);
    stackPath+="/";


    //std::cout<<stackPath<<std::endl;



    if(droneModuleType==droneModule::active)
    {
        //Services
        startServerSrv=n.advertiseService(moduleName+"/start",&DroneModule::startServCall,this);
        stopServerSrv=n.advertiseService(moduleName+"/stop",&DroneModule::stopServCall,this);
        resetServerSrv=n.advertiseService(moduleName+"/reset",&DroneModule::resetServCall,this);
        //isStartedServerSrv=n.advertiseService(moduleName+"/isStarted",&DroneModule::isStartedServCall,this);

        //Topics
        isStartedPub=n.advertise<std_msgs::Bool>(moduleName+"/isStarted", 1);

    }

    if(droneModuleType==droneModule::monitor)
    {
        //Services
        startClientSrv=n.serviceClient<std_srvs::Empty>(moduleName+"/start");
        stopClientSrv=n.serviceClient<std_srvs::Empty>(moduleName+"/stop");
        resetClientSrv=n.serviceClient<std_srvs::Empty>(moduleName+"/reset");
        //isStartedClientSrv=n.serviceClient<droneModule::isStarted>(moduleName+"/isStarted");

        //Topics
        isStartedSub=n.subscribe(moduleName+"/isStarted", 1, &DroneModule::isStartedSubCallback, this);

    }



    //End
    return;
}



void Module::init()
{
    moduleStarted=false;
    droneModuleOpened=false;

    //End
    return;
}

void Module::close()
{
    //desactivar
    moduleStarted=false;


    //publicar
    isStartedPublish();

    return;
}





bool Module::reset()
{
    if(droneModuleType==droneModule::active)
    {
        return resetValues();
    }
    else
    {
        //use service
        if(resetClientSrv.call(emptySrv))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}


bool Module::resetServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    return resetValues();
}


bool Module::resetValues()
{
    return true;
}



bool Module::start()
{
    if(droneModuleType==droneModule::active)
    {
        return startVal();
    }
    else
    {
        //use service
        if(startClientSrv.call(emptySrv))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool Module::startServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    return startVal();
}


bool Module::startVal()
{
    if (!moduleStarted)
    {
        moduleStarted = true;
    }
    return true;
}





bool Module::stop()
{
    if(droneModuleType==droneModule::active)
    {
        return stopVal();
    }
    else
    {
        //use service
        if(stopClientSrv.call(emptySrv))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool Module::stopServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    return stopVal();
}


bool Module::stopVal()
{
    if (moduleStarted)
    {
        moduleStarted = false;
    }
    return true;
}




bool Module::isStarted()
{
    //if(droneModuleType==droneModule::active)
    //{
        return moduleStarted;
    //}
    //else
    //{
        //use service
        //if(isStartedClientSrv.call(isStartedSrv))
        //{
        //    return isStartedSrv.response.result;
        //}
        //else
        //{
        //    return false;
        //}
    //}
    //return false;
}

/*
bool DroneModule::isStartedServCall(droneModule::isStarted::Request& request, droneModule::isStarted::Response& response)
{
    response.result=moduleStarted;
    return true;
}
*/


void Module::isStartedSubCallback(const std_msgs::Bool::ConstPtr &msg)
{
    moduleStarted=(bool)msg->data;
    return;
}


void Module::isStartedPublish()
{
    if ( (droneModuleType==droneModule::active) && (droneModuleOpened) )
    {
        isStartedMsg.data=moduleStarted;
        isStartedPub.publish(isStartedMsg);
    }
    return;
}





bool Module::run()
{
    //Publish status
    isStartedPublish();
    //Do other staff
    if (!moduleStarted)
    {
        return false;
	}
    else
        return true;
}



void Module::sleep()
{
    moduleRate.sleep();
    return;
}










DroneModule::DroneModule(droneModule::droneModuleTypes droneModuleTypeIn, double moduleRateIn, droneModule::droneModuleLoggerTypes droneModuleLoggerTypesIn)
    : Module(droneModuleTypeIn,moduleRateIn,droneModuleLoggerTypesIn)
{

    return;
}


DroneModule::~DroneModule()
{

    return;
}


void DroneModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    Module::open(nIn,moduleName);


    //Drone id
    idDrone=0;
    ros::param::get("~droneId",idDrone);

    std::ostringstream convert;
    convert<<idDrone;
//    convert << std::setfill('0') << std::setw(2) << idDrone; // maybe we should consider doing this
    stringId=convert.str();


    return;
}
