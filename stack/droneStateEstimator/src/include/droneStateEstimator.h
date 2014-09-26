/*
 * DroneStateEstimator.h
 *
 *  Created on: May 10, 2012
 *      Author: jespestana // joselusl
 */

#ifndef DRONE_STATE_ESTIMATOR_H
#define DRONE_STATE_ESTIMATOR_H



////// ROS  ///////
#include "ros/ros.h"




/////// EKF //////////
#include "extendedKalmanFilter.h"
#include "matrixLib.h"


///// Model definition //////
// Which multirotor is going to be controlled?
//#include "config_Mydrone.h"


#define _MULTIROTOR_IS_PARROT_
//#define _MULTIROTOR_IS_PELICAN_
//#define _MULTIROTOR_IS_LINKQUAD_


#ifdef _MULTIROTOR_IS_PARROT_
#include "stateObserver/models/EKF_config_Parrot.h"
#include "stateObserver/models/EKF_model_Parrot.h"
typedef ParrotModel1 Multirotor_Model;
#endif

#ifdef _MULTIROTOR_IS_PELICAN_
#include "stateObserver/models/EKF_config_Pelican.h"
#include "stateObserver/models/EKF_model_Pelican.h"
typedef PelicanModel1 Multirotor_Model;
#endif

#ifdef _MULTIROTOR_IS_LINKQUAD_
#include "stateObserver/models/EKF_config_LinkQuad.h"
#include "stateObserver/models/EKF_model_LinkQuad.h"
typedef LinkQuadModel1 Multirotor_Model;
#endif




//Mutex for data syncronization!
//#include <pthread.h>


//Math
#include <math.h>


//Timer. No usado nunca!
//#include "Other/Timer.h"

//Time. Se usa la libreria de ros para el tiempo
//#include <time.h>


//Other tools
#include "Other/jesus_library.h"



//#include "datalogger.h"

// Jesus' to-do list:
// TODO: Cosas que me gustaría añadir al EKF,
//			- TODO: Hacer pruebas con diferentes constantes para las varianzas de medida, de ruido a la entrada, etc
//			- TODO: habilitar/permitir cambios de configuracion del EKF online


//Drone module
#include "droneModule.h"


//Drone msgs
#include "droneMsgs/dronePose.h"
#include "droneMsgs/droneSpeeds.h"
#include "droneMsgs/droneNavCommand.h" //Input to the process model for prediction
#include "droneMsgs/droneNavData.h"
#include "droneMsgs/droneSensorData.h"



//Change on reference frame
#include "referenceFrames.h"

#include "communication_definition.h"

//Services
#define FREQ_STATE_ESTIMATOR    30.0

class DroneStateEstimator : public DroneModule
{	

        
        
    //Model definition
private:
    Multirotor_Model multirotorModel;

	// ***** Internal objects and variables *****
	ContinuousExtendedKalmanFilter 	EKF;
    //volatile bool EKF_started;
    //bool EKF_started;
	//DroneProxy::Threading::Mutex EKF_Mutex;  //JL: TBR
    //Vector RealState;
	Vector EstimatedState;
	Vector FlagObservers;   			// Flags to be activated when a measurement is obtained
	Vector RealObservation;
	// This two vector may be used for logging
	Vector RealActuation; 	 			// Without noise
    //Vector RealActuation_km1;


    //Vector OdometryMeasurementsIndex; 	// Index of odometry measurements
    //Vector PositionMeasurementsIndex; 	// Index of odometry measurements
    //Vector OtherMeasurementsIndex; 		// Index of other UNUSED measurements

	Vector VarObservation_EKF;
	Vector VarActuation_EKF;
	Vector varStateModel_EKF;
	Matrix MatPInit;

    //Matrices de varianzas y covarianzas en la estimacion
    Matrix MatVarEstimatedState;
    Matrix MatVarOutput;
	// END: ***** Internal objects and variables *****

	Vector EstimatedObservation;



	//Mutexs for data sincronization
    //pthread_mutex_t EKF_Mutex;
	//static pthread_mutex_t* mutexMeasOdometry;
	//static pthread_mutex_t* mutexMeasPose;
	//static pthread_mutex_t* mutexInputs;
	//static pthread_mutex_t* mutexOutputEstimatedState;


    ////Publishers
    //estimated Pose
private:
    ros::Publisher droneEstimatedPosePubl;
    droneMsgs::dronePose droneEstimatedPoseMsg;
    //estimated speeds
private:
    ros::Publisher droneEstimatedSpeedsPubl;
    droneMsgs::droneSpeeds droneEstimatedSpeedsMsg;

    //TODO JL: desired pose and speeds to be set!
private:
    droneMsgs::dronePose dronePoseMsg;
    droneMsgs::droneSpeeds droneSpeedsMsg;



    /////Subscribers
private:
    //estimated pose
    ros::Subscriber droneEstimatedPoseSubs;
    //droneStateEstimator::dronePose droneEstimatedPoseMsg;
    void droneEstimatedPoseCallback(const droneMsgs::dronePose::ConstPtr& msg);
    //estimated speeds
    ros::Subscriber droneEstimatedSpeedsSubs;
    //droneStateEstimator::droneSpeeds droneEstimatedSpeedsMsg;
    void droneEstimatedSpeedsCallback(const droneMsgs::droneSpeeds::ConstPtr& msg);



public:

    //JL no usada nunca!!!
    //Gains ???
    inline void setModelIntputGains(double gain_pitch, double gain_roll, double gain_dyaw, double gain_dz);



    //State estimation
    int stateEstimation(float mahalanobisDistance);


    //Set measures
    int setDroneMeasuresOdometry(droneMsgs::droneNavData feedbackData);
    int setDroneMeasuresSensor(droneMsgs::droneSensorData sensorData);


    //Set drone inputs
public:
    int setDroneInputs(droneMsgs::droneNavCommand inputData);
private:
    void setDroneInputPitch(float pitch);
    void setDroneInputRoll(float roll);
    void setDroneInputdYaw(float dyaw);
    void setDroneInputGaz(float gaz);

    //Get drone estimated state
public:
    //void getDroneEstimatedPosition(double *x, double *y, double *z, double *yaw);
    //void getEstimatedObservation(Vector *estObserv);
    bool getEstimatedPose(droneMsgs::dronePose* droneEstimatedPoseOut);
    bool getEstimatedSpeeds(droneMsgs::droneSpeeds* droneEstimatedSpeedsOut);
    droneMsgs::dronePose getEstimatedPose();
    droneMsgs::droneSpeeds getEstimatedSpeeds();


    //JL TODO
    //Get variances of the pose and speeds estimation
public:




public:
    DroneStateEstimator(droneModule::droneModuleTypes droneModuleTypeIn=droneModule::monitor, droneModule::droneModuleLoggerTypes droneModuleLoggerTypesIn = droneModule::non_logger);
    ~DroneStateEstimator();
	
    void open(ros::NodeHandle & nIn, std::string moduleName);
	void close();

protected:
	void init();


    //Reset
protected:
    bool resetValues();



    //Start
protected:
    bool startVal();


    //Stop
protected:
    bool stopVal();


    //TODO JL
public:
    //To set the values of the state estimator
    void setDronePose(droneMsgs::dronePose dronePoseIn);
    void setDroneSpeeds(droneMsgs::droneSpeeds droneSpeedsIn);


private:
    int publishEstimatedPose(void);
    int publishEstimatedSpeeds(void);

    // DroneLogger - Module required definitions/declarations
private:
    ros::Duration run_timestamp;
    ros::Time     init_timestamp;
    std::ostringstream stateEstimatorLogMsgStrm;
    void updateStateEstimatorLogMsgStr();
public:
    inline void setInitTimestamp( const ros::Time &init_timestamp_in) { init_timestamp = init_timestamp_in; }
    void getStateEstimatorLogMsgStr( std::string &str_in );
};



#endif /* MULTIROTOR_EKF_H_ */
