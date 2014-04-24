/*
 * PelicanEKF.cpp
 *
 *  Created on: May 10, 2012
 *      Author: jespestana
 */


#include "droneStateEstimator.h"



#define EKF_MAXIMUM_ADMISIBLE_SPEED 2.0 // m/s

#define PI 3.14159265
#define DEG2RAD PI/180.0





DroneStateEstimator::DroneStateEstimator(droneModule::droneModuleTypes droneModuleTypeIn, droneModule::droneModuleLoggerTypes droneModuleLoggerTypesIn) : DroneModule(droneModuleTypeIn,FREQ_STATE_ESTIMATOR)
{
    // logger => active
    if ( droneModuleType == droneModule::monitor )
        droneModuleLoggerType = droneModule::non_logger;
    else
        droneModuleLoggerType = droneModuleLoggerTypesIn;

    init();
    return;
}



DroneStateEstimator::~DroneStateEstimator()
{
	close();
	return;
}

void DroneStateEstimator::open(ros::NodeHandle & nIn, std::string moduleName)
{
	//Node
    DroneModule::open(nIn,moduleName);


	
    //// Topics ///
    //Publisers
    if(droneModuleType==droneModule::active)
    {
        droneEstimatedPosePubl = n.advertise<droneMsgs::dronePose>(DRONE_STATE_ESTIMATOR_POSE_PUBLICATION_LMrT, 1);
        droneEstimatedSpeedsPubl = n.advertise<droneMsgs::droneSpeeds>(DRONE_STATE_ESTIMATOR_SPEEDS_PUBLICATION_LMrT, 1);
    }
    //subscribers
    if(droneModuleType==droneModule::monitor)
    {
        droneEstimatedPoseSubs = n.subscribe(DRONE_STATE_ESTIMATOR_POSE_SUBSCRIPTION_LMrT, 1, &DroneStateEstimator::droneEstimatedPoseCallback, this);
        droneEstimatedSpeedsSubs = n.subscribe(DRONE_STATE_ESTIMATOR_SPEEDS_SUBSCRIPTION_LMrT, 1, &DroneStateEstimator::droneEstimatedSpeedsCallback, this);
    }


    //Flag of module opened
    droneModuleOpened=true;
	
	//End
	return;
}


void DroneStateEstimator::droneEstimatedPoseCallback(const droneMsgs::dronePose::ConstPtr& msg)
{
    droneEstimatedPoseMsg.time=msg->time;

    droneEstimatedPoseMsg.x=msg->x;
    droneEstimatedPoseMsg.y=msg->y;
    droneEstimatedPoseMsg.z=msg->z;
    droneEstimatedPoseMsg.yaw=msg->yaw;
    droneEstimatedPoseMsg.pitch=msg->pitch;
    droneEstimatedPoseMsg.roll=msg->roll;

    return;
}


void DroneStateEstimator::droneEstimatedSpeedsCallback(const droneMsgs::droneSpeeds::ConstPtr& msg)
{
    droneEstimatedSpeedsMsg.time=msg->time;

    droneEstimatedSpeedsMsg.dx=msg->dx;
    droneEstimatedSpeedsMsg.dy=msg->dy;
    droneEstimatedSpeedsMsg.dz=msg->dz;
    droneEstimatedSpeedsMsg.dyaw=msg->dyaw;
    droneEstimatedSpeedsMsg.dpitch=msg->dpitch;
    droneEstimatedSpeedsMsg.droll=msg->droll;

    return;
}


void DroneStateEstimator::init()
{

    //EKF_started=false;
    EKF.creation( MULTIROTOR_MODEL_NUMSTATES, MULTIROTOR_MODEL_NUMINPUTS, MULTIROTOR_MODEL_NUMMEASURES);

	//Init State Estimator
	int numStates   = MULTIROTOR_MODEL_NUMSTATES;
	int numInputs   = MULTIROTOR_MODEL_NUMINPUTS;
	int numMeasures = MULTIROTOR_MODEL_NUMMEASURES;
	//pmy_drone = NULL; //JL: removed

    multirotorModel.setMaxIntegrationTime(0.001);
	EKF.initModel(&multirotorModel);

	//Inicializamos otras variables
    //RealState.creation(numStates);
	EstimatedState.creation(numStates);
	RealActuation.creation(numInputs);
    //RealActuation_km1.creation(numInputs);
	FlagObservers.creation(numMeasures);   			// Flags to be activated when a measurement is obtained
	RealObservation.creation(numMeasures);
	EstimatedObservation.creation(numMeasures);

    //JL
    //OdometryMeasurementsIndex.creation(6); 			// Index of odometry measurements
    //OdometryMeasurementsIndex.setValueData( 1  , 1 );
    //OdometryMeasurementsIndex.setValueData( 2  , 2 );
    //OdometryMeasurementsIndex.setValueData( 4  , 3 );
    //OdometryMeasurementsIndex.setValueData( 6  , 4 );
    //OdometryMeasurementsIndex.setValueData( 11 , 5 );
    //OdometryMeasurementsIndex.setValueData( 12 , 6 );

    //JL
    //PositionMeasurementsIndex.creation(2); 			// Index of odometry measurements
    //PositionMeasurementsIndex.setValueData( 7  , 1 );
    //PositionMeasurementsIndex.setValueData( 8  , 2 );

    //JL
    //OtherMeasurementsIndex.creation(4); 			// Index of other UNUSED measurements
    //OtherMeasurementsIndex.setValueData( 3  , 1 );
    //OtherMeasurementsIndex.setValueData( 5  , 2 );
    //OtherMeasurementsIndex.setValueData( 9  , 3 );
    //OtherMeasurementsIndex.setValueData( 10 , 4 );

	/////// Observation variances
	VarObservation_EKF.creation(numMeasures);
	// Odometry measurements
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_PITCH_DEG 	*PI/180.0)      , 2 ) ,1); // Pitch
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_ROLL_DEG 	*PI/180.0)      , 2 ) ,2); // Roll
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_YAW_ODOMETRY_DEG*PI/180.0)  , 2 ) ,4); // Yaw
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_Z_M					 )      , 2 ) ,6); // Z
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_VXM_MPS				 )      , 2 ) ,11);// Vxm
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_VYM_MPS 			 )      , 2 ) ,12);// Vym
	// Position measurements
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_X_M					 )      , 2 ) ,7); // X position (PX)
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_Y_M					 )      , 2 ) ,8); // Y position (PY)
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_YAW_VICON_DEG*PI/180.0)     , 2 ) ,13); // Yaw
	// Other measurements
	VarObservation_EKF.setValueData( STD_OBS_EKF_UNUSED ,3); // dYaw/dt
	VarObservation_EKF.setValueData( STD_OBS_EKF_UNUSED ,5); // dZ/dt
	VarObservation_EKF.setValueData( STD_OBS_EKF_UNUSED ,9); // Vx
	VarObservation_EKF.setValueData( STD_OBS_EKF_UNUSED ,10);// Vy

	/////// Command variances
	VarActuation_EKF.creation(numInputs);
	VarActuation_EKF.setValueData( pow( ( STD_ACT_EKF_PITCH_DEG		/24)   , 2 ) ,1);  // Pitch
	VarActuation_EKF.setValueData( pow( ( STD_ACT_EKF_ROLL_DEG 		/24)   , 2 ) ,2);  // Roll
	VarActuation_EKF.setValueData( pow( ( STD_ACT_EKF_DYAW_DEGPS	/100)  , 2 ) ,3);  // dYaw/dt
	VarActuation_EKF.setValueData( pow( ( STD_ACT_EKF_DZ_MPS		/1)    , 2 ) ,4);  // dZ/dt

	/////// Initial state estimation variances
	varStateModel_EKF.creation(numStates);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_YAW_DEG*PI/180.0 ,2) , 5);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_Z_M	 ,2) , 8);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_X_M	 ,2) , 9);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_Y_M	 ,2) ,10);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_VX_MPS	 ,2) ,11);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_VY_MPS	 ,2) ,12);

	MatPInit.creation(numStates,numStates);
	MatPInit.setValueData( pow( (( STD_IST_EKF_PITCH_DEG  *PI/180)/58.227) , 2 ) ,1 ,1 ); //  X1 : proportional to pitch
	MatPInit.setValueData( pow( (( STD_IST_EKF_ROLL_DEG   *PI/180)/38.575) , 2 ) ,2 ,2 ); //  X2 : proportional to roll
	MatPInit.setValueData( pow( (( STD_IST_EKF_DYAW1_DEGPS*PI/180)/67.213) , 2 ) ,3 ,3 ); //  X3 : internal variable related to d(yaw)/dt
	MatPInit.setValueData( pow( (( STD_IST_EKF_DYAW2_DEGPS*PI/180)/67.213) , 2 ) ,4 ,4 ); //  X4 : proportional to d(yaw)/dt
	MatPInit.setValueData( pow( (( STD_IST_EKF_YAW_DEG    *PI/180)       ) , 2 ) ,5 ,5 ); //  X5 : yaw
	MatPInit.setValueData( pow( (  STD_IST_EKF_DZ1_MPS   /14.189)          , 2 ) ,6 ,6 ); //  X6 : internal variable related to d(Z)/dt
	MatPInit.setValueData( pow( (  STD_IST_EKF_DZ2_MPS   /14.189)          , 2 ) ,7 ,7 ); //  X7 : proportional to d(Z)/dt
	MatPInit.setValueData( pow( (  STD_IST_EKF_Z_M       )                 , 2 ) ,8 ,8 ); //  X8 : Z
	MatPInit.setValueData( pow( (  STD_IST_EKF_X_M       )                 , 2 ) ,9 ,9 );	//  X9 : X
	MatPInit.setValueData( pow( (  STD_IST_EKF_Y_M       )                 , 2 ) ,10,10);	//  X10: Y
	MatPInit.setValueData( pow( (  STD_IST_EKF_VX_MPS    )                 , 2 ) ,11,11);	//  X11: VX
	MatPInit.setValueData( pow( (  STD_IST_EKF_VY_MPS    )                 , 2 ) ,12,12);	//  X12: VY


    MatVarEstimatedState.creation(numStates,numStates);
    MatVarOutput.creation(numMeasures,numMeasures);

    //EKF.init(&EstimatedState,&VarActuation_EKF,&VarObservation_EKF,&MatPInit);
    EKF.setVarActuation(&VarActuation_EKF);
    EKF.setVarObservation(&VarObservation_EKF);
	EKF.setVarState(&varStateModel_EKF);


    //initial state + time
    resetValues();

    run_timestamp  = ros::Duration(0.0);
    init_timestamp = ros::Time::now();

    stateEstimatorLogMsgStrm.str(std::string());

	return;
}



void DroneStateEstimator::close()
{

    DroneModule::close();
    return;
}



bool DroneStateEstimator::resetValues()
{

    //Init State
    EstimatedState.setZeros();
    EKF.setInitialState(&EstimatedState);
    EKF.setVarInitState(&MatPInit);

    //time
    double initTime = ros::Time::now().toSec();
    EKF.setInitTime(initTime);


    //JL: It should reset everything to zero, because EKF is reset to zero!
    droneEstimatedPoseMsg.time=initTime;
    droneEstimatedPoseMsg.x=0.0;
    droneEstimatedPoseMsg.y=0.0;
    droneEstimatedPoseMsg.z=0.0;
    droneEstimatedPoseMsg.yaw=0.0;
    droneEstimatedPoseMsg.pitch=0.0;
    droneEstimatedPoseMsg.roll=0.0;

    droneEstimatedSpeedsMsg.time=initTime;
    droneEstimatedSpeedsMsg.dx=0.0;
    droneEstimatedSpeedsMsg.dy=0.0;
    droneEstimatedSpeedsMsg.dz=0.0;
    droneEstimatedSpeedsMsg.dyaw=0.0;
    droneEstimatedSpeedsMsg.dpitch=0.0;
    droneEstimatedSpeedsMsg.droll=0.0;


    //Publicamos
    publishEstimatedPose();
    publishEstimatedSpeeds();
    run_timestamp = ros::Time::now() - init_timestamp;
    updateStateEstimatorLogMsgStr();

    return true;

}



bool DroneStateEstimator::startVal()
{
    //Reset time

    //time
    double initTime = ros::Time::now().toSec();
    EKF.setInitTime(initTime);


    //
    droneEstimatedPoseMsg.time=initTime;
    droneEstimatedSpeedsMsg.time=initTime;


    //Publicamos
    publishEstimatedPose();
    publishEstimatedSpeeds();
    run_timestamp = ros::Time::now() - init_timestamp;
    updateStateEstimatorLogMsgStr();

    //End
    return DroneModule::startVal();
}



bool DroneStateEstimator::stopVal()
{
    return DroneModule::stopVal();
}





void DroneStateEstimator::setModelIntputGains(double gain_pitch, double gain_roll, double gain_dyaw, double gain_dz)
{
    multirotorModel.setIntputGains(gain_pitch, gain_roll, gain_dyaw, gain_dz);
}



int DroneStateEstimator::setDroneMeasuresOdometry(droneMsgs::droneNavData feedbackData)
{


    //	struct feedbackData__ {
    //			unsigned char	grantedAccessMode;
    //			unsigned int	timeCodeH;
    //			unsigned int	timeCodeL;
    //			char			commWithDrone;
    //			char		 	droneMode;
    //			int				nativeDroneState;
    //			float			batteryLevel;
    //			float			roll;
    //			float			pitch;
    //			float			yaw;
    //			float			altitude;
    //			float			speedX;
    //			float			speedY;
    //			float			speedYaw;
    //		} __PACKED feedbackData;

    //  Medidas:
    //  01 pitch: C(1,1)*X1
    //  02 roll : C(2,2)*X2
    //  03 dYaw : C(3,4)*X4
    //  04 Yaw  : X5
    //  05 dZ   : C(5,7)*X7
    //  06 Z    : X8
    //  07 X    : X9					"VISION"
    //  08 Y    : X10					"VISION"
    //  09 Vx   : X11
    //  10 Vy   : X12
    //  11 [Vxm;: [R_Y R_Y]' * [X11;
    //  12 Vym] : [R_Y R_Y]     X12]
    //  13 Yaw  : X5					"VISION"




    if(!moduleStarted ||  droneModuleType==droneModule::monitor)
        return 0;



    //Read measures
    double pitch_rad = feedbackData.pitch*DEG2RAD;
    double roll_rad  = feedbackData.roll*DEG2RAD;


    //Yaw
    double yaw_rad   = feedbackData.yaw*DEG2RAD;
    //To fix angle problems with yaw ->JL I have to fix this!!!
    //EKF.getEstimatedState(&EstimatedState);
    //multirotorModel.observationModel(&EstimatedObservation, &EstimatedState);
    EKF.getEstimatedOutput(&EstimatedObservation);
    double yaw_EKF_rad = EstimatedObservation.getValueData( 4 );
    yaw_EKF_rad = jesus_library::mapAnglesToBeNear_PIrads( yaw_EKF_rad, yaw_rad);
    EstimatedState.setValueData(yaw_EKF_rad, 5);
    EKF.setInitialState(&EstimatedState);


    //Speed limit-> to avoid bad measures in the speed
    float vx_fb = feedbackData.speedX, vy_fb = feedbackData.speedY;
    float v = sqrt(vx_fb*vx_fb + vy_fb*vy_fb);
    if ( v <= EKF_MAXIMUM_ADMISIBLE_SPEED )
    {
        RealObservation.setValueData( feedbackData.speedX, 11);
        FlagObservers.setValueData( 1.0, 11 );
        RealObservation.setValueData( feedbackData.speedY, 12);
        FlagObservers.setValueData( 1.0, 12 );
        RealObservation.setValueData( yaw_rad    , 4);  // Hay que arreglar yaw_rad
        FlagObservers.setValueData( 1.0, 4 );
        RealObservation.setValueData( pitch_rad  , 1);
        FlagObservers.setValueData( 1.0, 1 );
        RealObservation.setValueData( roll_rad   , 2);
        FlagObservers.setValueData( 1.0, 2 );
        RealObservation.setValueData( feedbackData.altitude,6);
        FlagObservers.setValueData( 1.0, 6 );
        EKF.setSystemMeasures(&RealObservation);
    }
    else //we dont use speed values
    {
        FlagObservers.setValueData( 0.0, 11 );
        FlagObservers.setValueData( 0.0, 12 );
        //JL change
        //FlagObservers.setValueData( 0.0, 4 );
        //FlagObservers.setValueData( 0.0, 1 );
        //FlagObservers.setValueData( 0.0, 2 );
        //FlagObservers.setValueData( 0.0, 6 );
        RealObservation.setValueData( yaw_rad    , 4);  // Hay que arreglar yaw_rad
        FlagObservers.setValueData( 1.0, 4 );
        RealObservation.setValueData( pitch_rad  , 1);
        FlagObservers.setValueData( 1.0, 1 );
        RealObservation.setValueData( roll_rad   , 2);
        FlagObservers.setValueData( 1.0, 2 );
        RealObservation.setValueData( feedbackData.altitude,6);
        FlagObservers.setValueData( 1.0, 6 );
        EKF.setSystemMeasures(&RealObservation);
    }




    /*
    if (pthread_mutex_lock(&EKF_Mutex))
    {


        // make the estimation advance until the actualTime
        for ( int i = 1; i <= MULTIROTOR_MODEL_NUMMEASURES; i ++)
        {
            EKF.deactivateSystemMeasure(i);
            FlagObservers.setValueData( 0.0, i);
        }
        stateEstimation(PARROTEKF_MAHALANOBIS_DISTANCE);


        double pitch_rad = feedbackData->pitch*DEG2RAD;
        double roll_rad  = feedbackData->roll*DEG2RAD;
        double yaw_rad   = feedbackData->yaw*DEG2RAD;

        EKF.getEstimatedState(&EstimatedState);

        multirotorModel.observationModel(&EstimatedObservation, &EstimatedState);

        double yaw_EKF_rad = EstimatedObservation.getValueData( 4 );
        yaw_EKF_rad = jesus_library::mapAnglesToBeNear_PIrads( yaw_EKF_rad, yaw_rad);

        EstimatedState.setValueData(yaw_EKF_rad, 5);


        EKF.setInitialState(&EstimatedState);



        //Execute EKF
        float vx_fb = feedbackData->speedX, vy_fb = feedbackData->speedY;
        float v = sqrt(vx_fb*vx_fb + vy_fb*vy_fb);
        // feedbackData->commWithDrone == true -> Communicaciones ok
        // feedbackData->commWithDrone == false -> perdida de comunicacion, medidas potencialmente erroneas
        if ( v <= EKF_MAXIMUM_ADMISIBLE_SPEED )
        {
            RealObservation.setValueData( feedbackData->speedX, 11);
            FlagObservers.setValueData( 1.0, 11 );
            RealObservation.setValueData( feedbackData->speedY, 12);
            FlagObservers.setValueData( 1.0, 12 );
            RealObservation.setValueData( yaw_rad    , 4);  // Hay que arreglar yaw_rad
            FlagObservers.setValueData( 1.0, 4 );
            RealObservation.setValueData( pitch_rad  , 1);
            FlagObservers.setValueData( 1.0, 1 );
            RealObservation.setValueData( roll_rad   , 2);
            FlagObservers.setValueData( 1.0, 2 );
            RealObservation.setValueData( feedbackData->altitude,6);
            FlagObservers.setValueData( 1.0, 6 );
            EKF.setSystemMeasures(&RealObservation);
        }
        else
        {
            FlagObservers.setValueData( 0.0, 11 );
            FlagObservers.setValueData( 0.0, 12 );
            FlagObservers.setValueData( 0.0, 4 );
            FlagObservers.setValueData( 0.0, 1 );
            FlagObservers.setValueData( 0.0, 2 );
            FlagObservers.setValueData( 0.0, 6 );
        }

        stateEstimation(PARROTEKF_MAHALANOBIS_DISTANCE);


        pthread_mutex_unlock(&EKF_Mutex);
    }


    */

    return 1;

}


int DroneStateEstimator::setDroneMeasuresSensor(droneMsgs::droneSensorData sensorData)
{
    if(!moduleStarted ||  droneModuleType==droneModule::monitor)
        return 0;

    //JL: TODO Terminar!!
    RealObservation.setValueData( sensorData.x, 11);
    FlagObservers.setValueData( 1.0, 11 );

    RealObservation.setValueData( sensorData.y, 12);
    FlagObservers.setValueData( 1.0, 12 );

    RealObservation.setValueData( sensorData.z   , 4);
    FlagObservers.setValueData( 1.0, 4 );

    RealObservation.setValueData( sensorData.yaw  , 1);
    FlagObservers.setValueData( 1.0, 1 );

    RealObservation.setValueData( sensorData.pitch   , 2);
    FlagObservers.setValueData( 1.0, 2 );

    RealObservation.setValueData( sensorData.roll, 11);
    FlagObservers.setValueData( 1.0, 11 );



    return 1;
}

int DroneStateEstimator::stateEstimation(float mahalanobisDistance)
{

    DroneModule::run();


    //int output = 0;
    //JL: just for log
    //cvg_ulong timeCode = cvgTimer::getSystemSeconds()*1e6;
    //double actualTime = pmy_drone->getTime(); // timer.getElapsedSeconds();



    //TEST
    //std::cout<<"Actual Time="<<actualTime<<std::endl;
    //std::cout<<"Elapsed Time="<<actualTime-initTime<<std::endl;
    //return 1;

//	timer.restart(started);

    if (!moduleStarted || droneModuleType==droneModule::monitor)
    {
        /*
        for (int i = 1; i <= EstimatedState.length(); i++ )
        {
            EstimatedState.setValueData( 0.0, i);
        }

        EKF.init(&EstimatedState,&VarActuation_EKF,&VarObservation_EKF,&MatPInit);
        EKF.setVarState(&varStateModel_EKF);

        EKF_started = true;
        */
        return 0; // MIRAR ESTO!!!
    }



    //double last_actualTime =  EKF.getActualTime();
    //if (last_actualTime < actualTime)
    //{
    double actualTime = ros::Time::now().toSec();
    //std::cout<<"time:"<<actualTime<<std::endl;
        EKF.setActualTime(actualTime);
    //}

    //Activa o desactiva observacion
    for( int i=1; i<=MULTIROTOR_MODEL_NUMMEASURES; i++)
    {
        if ( FlagObservers.getValueData(i) == 1.0 )
        {
            EKF.activateSystemMeasure(i);
            //Bajamos la bandera al haber leido ya las entradas
            FlagObservers.setValueData(0.0,i);
        }
        else
        {
            EKF.deactivateSystemMeasure(i);
        }
    }

    int output = EKF.stateEstimation(mahalanobisDistance);



    if(output)
    {
        //std::cout<<"aqui\n";
        //Get estimated state
        EKF.getEstimatedState(&EstimatedState);
        EKF.getEstimatedStateVariances(&MatVarEstimatedState);
        //Vector estObserv;
        //multirotorModel.observationModel( &EstimatedObservation, &EstimatedState);
        EKF.getEstimatedOutput(&EstimatedObservation);
        EKF.getEstimatedOutputVariances(&MatVarOutput);

        //Set values
        droneEstimatedPoseMsg.time=actualTime;
        droneEstimatedPoseMsg.x=EstimatedObservation.getValueData(7);
        droneEstimatedPoseMsg.y=EstimatedObservation.getValueData(8);
        droneEstimatedPoseMsg.z=EstimatedObservation.getValueData(6);
        droneEstimatedPoseMsg.yaw=EstimatedObservation.getValueData(4);
        droneEstimatedPoseMsg.pitch=EstimatedObservation.getValueData(1);
        droneEstimatedPoseMsg.roll=EstimatedObservation.getValueData(2);

        droneEstimatedSpeedsMsg.time=actualTime;
        droneEstimatedSpeedsMsg.dx=EstimatedObservation.getValueData(9);
        droneEstimatedSpeedsMsg.dy=EstimatedObservation.getValueData(10);
        droneEstimatedSpeedsMsg.dz=EstimatedObservation.getValueData(5);
        droneEstimatedSpeedsMsg.dyaw=EstimatedObservation.getValueData(3); //Never used!!
        droneEstimatedSpeedsMsg.dpitch=0.0; //Not defined!!!. Never used!!
        droneEstimatedSpeedsMsg.droll=0.0; //Not defined!!. Never used!!

        //Publish
        publishEstimatedPose();
        publishEstimatedSpeeds();
        run_timestamp = ros::Time::now() - init_timestamp;
        updateStateEstimatorLogMsgStr();

    }



    return output;
}




bool DroneStateEstimator::getEstimatedPose(droneMsgs::dronePose* droneEstimatedPoseOut)
{
    //getEstimatedObservation(&EstimatedObservation);

    droneEstimatedPoseOut->time=droneEstimatedPoseMsg.time;
    droneEstimatedPoseOut->x=droneEstimatedPoseMsg.x;
    droneEstimatedPoseOut->y=droneEstimatedPoseMsg.y;
    droneEstimatedPoseOut->z=droneEstimatedPoseMsg.z;
    droneEstimatedPoseOut->yaw=droneEstimatedPoseMsg.yaw;
    droneEstimatedPoseOut->pitch=droneEstimatedPoseMsg.pitch;
    droneEstimatedPoseOut->roll=droneEstimatedPoseMsg.roll;

    return true;
}



bool DroneStateEstimator::getEstimatedSpeeds(droneMsgs::droneSpeeds* droneEstimatedSpeedsOut)
{
    //getEstimatedObservation(&EstimatedObservation);
    droneEstimatedSpeedsOut->time=droneEstimatedSpeedsMsg.time;
    droneEstimatedSpeedsOut->dx=droneEstimatedSpeedsMsg.dx;
    droneEstimatedSpeedsOut->dy=droneEstimatedSpeedsMsg.dy;
    droneEstimatedSpeedsOut->dz=droneEstimatedSpeedsMsg.dz;
    droneEstimatedSpeedsOut->dyaw=droneEstimatedSpeedsMsg.dyaw;
    droneEstimatedSpeedsOut->dpitch=droneEstimatedSpeedsMsg.dpitch;
    droneEstimatedSpeedsOut->droll=droneEstimatedSpeedsMsg.droll;
    return true;
}


droneMsgs::dronePose DroneStateEstimator::getEstimatedPose()
{
    return droneEstimatedPoseMsg;
}



droneMsgs::droneSpeeds DroneStateEstimator::getEstimatedSpeeds()
{
    return droneEstimatedSpeedsMsg;
}






// ***** Functions that act on the input to the system *****
int DroneStateEstimator::setDroneInputs(droneMsgs::droneNavCommand inputData)
{

    if(!moduleStarted ||  droneModuleType==droneModule::monitor)
        return 0;

    //Set inputs
    setDroneInputPitch(inputData.pitch);
    setDroneInputRoll(inputData.roll);
    //float dyaw=inputData.dyaw;
    setDroneInputdYaw(inputData.dyaw);
    //float gaz=inputData.gaz;
    setDroneInputGaz(inputData.dz);

    //set
    EKF.setSystemInputs(&RealActuation);



    //	// Assign inputs
    //	for (int i = 1; i <= inputsIn->length(); i++) {
    //		RealActuation.setValueData( inputsIn->getValueData(i), i);
    //	}

    //double actualTime = pmy_drone->getTime(); // timer.getElapsedSeconds();
/*
    if ( pthread_mutex_lock(&EKF_Mutex) )
    {
        stateEstimation(PARROTEKF_MAHALANOBIS_DISTANCE);
        EKF.getSystemInputs(&RealActuation_km1);
//		logCommandData(actualTime, &RealActuation_km1);
        EKF.setSystemInputs(&RealActuation);
//		logCommandData(actualTime, &RealActuation);
        pthread_mutex_unlock(&EKF_Mutex);
    }
    */

    return 1;
}


void DroneStateEstimator::setDroneInputPitch(float pitch)
{
    //if ( pthread_mutex_lock(&EKF_Mutex) )
    //{
        RealActuation.setValueData( pitch, 1);  // 1
      //  pthread_mutex_unlock(&EKF_Mutex);
    //}
}


void DroneStateEstimator::setDroneInputRoll(float roll)
{
    //if ( pthread_mutex_lock(&EKF_Mutex) )
    //{
        RealActuation.setValueData( roll,  2);  // 2
    //    pthread_mutex_unlock(&EKF_Mutex);
    //}
}


void DroneStateEstimator::setDroneInputdYaw(float dyaw)
{
    //if ( pthread_mutex_lock(&EKF_Mutex) )
    //{
        RealActuation.setValueData( dyaw,  3);  // 3
    //    pthread_mutex_unlock(&EKF_Mutex);
    //}
}


void DroneStateEstimator::setDroneInputGaz(float gaz)
{
    //if ( pthread_mutex_lock(&EKF_Mutex) )
    //{
        RealActuation.setValueData( gaz,   4); 	// 4
    //    pthread_mutex_unlock(&EKF_Mutex);
    //}
}
// END: ***** Functions that act on the input to the system *****






int DroneStateEstimator::publishEstimatedPose(void)
{
    if(droneModuleOpened==false || droneModuleType==droneModule::monitor)
        return 0;

    droneEstimatedPosePubl.publish(droneEstimatedPoseMsg);
    return 1;
}

int DroneStateEstimator::publishEstimatedSpeeds(void)
{
    if(droneModuleOpened==false || droneModuleType==droneModule::monitor)
        return 0;

    droneEstimatedSpeedsPubl.publish(droneEstimatedSpeedsMsg);
    return 1;
}



//JL TODO
void DroneStateEstimator::setDronePose(droneMsgs::dronePose dronePoseIn)
{
    //Set value in message
    dronePoseMsg.time=ros::Time::now().toSec();
    dronePoseMsg.x=dronePoseIn.x;
    dronePoseMsg.y=dronePoseIn.y;
    dronePoseMsg.z=dronePoseIn.z;
    dronePoseMsg.yaw=dronePoseIn.yaw;
    dronePoseMsg.pitch=dronePoseIn.pitch;
    dronePoseMsg.roll=dronePoseIn.roll;

    return;
}


//JL TODO
void DroneStateEstimator::setDroneSpeeds(droneMsgs::droneSpeeds droneSpeedsIn)
{

    return;
}

void DroneStateEstimator::updateStateEstimatorLogMsgStr() {
    if ( droneModuleLoggerType == droneModule::logger ) {
        stateEstimatorLogMsgStrm
            /* timestamp   */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
            /* tag         */   << " [ekf;state]"
            /* isStarted   */   << " started:" << isStarted()
            /* xs, ys, zs               */  << " xs:"     << droneEstimatedPoseMsg.x
            /* yaws, pitchs, rolls      */  << " ys:"     << droneEstimatedPoseMsg.y
                                            << " zs:"     << droneEstimatedPoseMsg.z
                                            << " yaws:"   << droneEstimatedPoseMsg.yaw
                                            << " pitchs:" << droneEstimatedPoseMsg.pitch
                                            << " rolls:"  << droneEstimatedPoseMsg.roll
            /* vxs, vys, vzs            */  << " vxs:"    << droneEstimatedSpeedsMsg.dx
            /* dyaws                    */  << " vys:"    << droneEstimatedSpeedsMsg.dy
                                            << " vzs:"    << droneEstimatedSpeedsMsg.dz
                                            << " dyaws:"  << droneEstimatedSpeedsMsg.dyaw
                                            << std::endl;
    }
}

void DroneStateEstimator::getStateEstimatorLogMsgStr( std::string &str_in ) {

    if (droneModuleLoggerType == droneModule::logger) {
        str_in = stateEstimatorLogMsgStrm.str();
        stateEstimatorLogMsgStrm.str(std::string());
    } else {
        str_in = "";
    }
    return;
}
