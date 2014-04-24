Introduction:
-------------

 * OpenTLD, repositories:
	[Matlab] https://github.com/zk00006/OpenTLD
		 http://gnebehay.github.com/OpenTLD/
	[C++] 	 https://github.com/gnebehay/OpenTLD/
	[ROS]	 https://github.com/Ronan0912/ros_opentld
	
	En la versión de C++ se pueden importar y exportar modelos a través de la linea de comandos.
	En ROS, importar y exportar modelos parece que no funciona.

 * When you run the code sometimes the tld_gui shows an empty screen. To refresh the screen to the latest image just press F5. I found this checking the function: "void BaseFrame::keyPressEvent(QKeyEvent * event)" of the BaseFrame class from the "tld_tracker" package:
~/Documents/workspace/ROS_OpenTLD/tld_tracker/src/base_frame.hpp
~/Documents/workspace/ROS_OpenTLD/tld_tracker/src/base_frame.cpp

ROS_OpenTLD, nodes and parameters:
----------------------------------

 * Information about the ROS_Opentld tracker nodes: published topics, subscriptions and similarly with services.

	ROS_Opentld nodes:
	 Node [/OpenTLD/ros_tld_gui_node]
	  Publications: 
	   * /OpenTLD/tld_gui_bb [tld_msgs/Target]
	   * /rosout [rosgraph_msgs/Log]
	   * /OpenTLD/tld_gui_cmds [std_msgs/Char]
	  Subscriptions: 
	   * /OpenTLD/tld_tracked_object [tld_msgs/BoundingBox]
	   * /OpenTLD/tld_fps [std_msgs/Float32]
	   * /ardrone/front/image_rect_color [sensor_msgs/Image] 
	  Services: 
	   * /OpenTLD/ros_tld_gui_node/set_logger_level
	   * /OpenTLD/ros_tld_gui_node/get_loggers

	 Node [/OpenTLD/ros_tld_tracker_node]
	  Publications: 
	   * /OpenTLD/tld_fps [std_msgs/Float32]
	   * /OpenTLD/tld_tracked_object [tld_msgs/BoundingBox]
	   * /rosout [rosgraph_msgs/Log]
	  Subscriptions: 
	   * /ardrone/front/image_rect_color [sensor_msgs/Image]
	   * /OpenTLD/tld_gui_cmds [std_msgs/Char]
	   * /OpenTLD/tld_gui_bb [tld_msgs/Target]
	  Services: 
	   * /OpenTLD/ros_tld_tracker_node/get_loggers
	   * /OpenTLD/ros_tld_tracker_node/set_logger_level

	 Message [tld_msgs/BoundingBox]
	  std_msgs/Header header
	    uint32 seq
	    time stamp
	    string frame_id
	  int32 x
	  int32 y
	  int32 width
	  int32 height
	  float32 confidence

	 Message [tld_msgs/Target]
	  tld_msgs/BoundingBox bb
	    std_msgs/Header header
	      uint32 seq
	      time stamp
	      string frame_id
	    int32 x
	    int32 y
	    int32 width
	    int32 height
	    float32 confidence
	  sensor_msgs/Image img
	    std_msgs/Header header
	      uint32 seq
	      time stamp
	      string frame_id
	    uint32 height
	    uint32 width
	    string encoding
	    uint8 is_bigendian
	    uint32 step
	    uint8[] data

 * I have been checking the parameters of these nodes to see if I could export and import models to and from the harddrive. I have not been succesful. The worst thing is that this option works for the OpenTLD that can be installed in Ubuntu from the software center (note that I think that is the C++ version, so the ROS version should be able to do it too if it where well programmed):
	PARAMETERS
	 * /ardrone/OpenTLD/ros_tld_tracker_node/autoFaceDetection
	 * /ardrone/OpenTLD/ros_tld_tracker_node/cascadePath
	 * /ardrone/OpenTLD/ros_tld_tracker_node/correctBB
	 * /ardrone/OpenTLD/ros_tld_tracker_node/exportModelAfterRun
	 * /ardrone/OpenTLD/ros_tld_tracker_node/height
	 * /ardrone/OpenTLD/ros_tld_tracker_node/loadModel
	 * /ardrone/OpenTLD/ros_tld_tracker_node/modelExportFile
	 * /ardrone/OpenTLD/ros_tld_tracker_node/modelImportFile
	 * /ardrone/OpenTLD/ros_tld_tracker_node/showOutput
	 * /ardrone/OpenTLD/ros_tld_tracker_node/width
	 * /ardrone/OpenTLD/ros_tld_tracker_node/x
	 * /ardrone/OpenTLD/ros_tld_tracker_node/y
	 * /rosdistro
	 * /rosversion

	NODES
	  /ardrone/OpenTLD/
	    ros_tld_gui_node (tld_tracker/ros_tld_gui_node)
	    ros_tld_tracker_node (tld_tracker/ros_tld_tracker_node)

[IBVS_Controller] Datalogging architecture:
-------------------------------------------

 * Datalogging during tests. I am going to program a droneLogger node that is to be started before all the other nodes, so that it can log all the information about the test. This is going to be a comperhensive list of which node is logging what:
    
	+ [x][ALL] All data should have a timestamp attached to it
	+ [x][IBVS-Controller] timestamp tag:[ardrone;state] drone mode (hovering, move, landed), navdata (telemetry), commands
	+ [x][IBVS-Controller] timestamp tag:[ctrlr;state] isStarted(), controlMode, target_is_on_frame, References, distance_to_target, commands, //feedback
	+ [x][IBVS-Controller] timestamp tag:[ctrlr;gains] Kp_fx2R, Ki_fx2R, Kd__fx2R, Kp_fx2DY (...), Kp_fy (...), Kp_fs (...), Kp_fD (...), C_fx2Dy, C_fx2DY, C_fy2Dz, C_fD2Dx
	+ [x][OpenTLDInterface][NOTA: implementado en IBVScontrollerNodeMain] timestamp tag:[opentld] isTrackingObject(), isObjectOnFrame(), bb_x, bb_y, bb_width, bb_height, bb_confidence, bb_fps
	+ [x][UI][IBVS-Controller] timestamp tag:[ardrone;event] takeoff, landing, hover, move, emergency, use_front_camera, use_bottom_camera
	+ [x][IBVS-Controller] timestamp tag:[ardrone;image] ¿images? basically image names when logged
	+ [NO;DoneByIBVS-Controller][OpenTLDInterface] Relative position to the target as obtained from basic CV calculation on the Bounding Box data
	+ [NOT_LOGGED] selection of new model in OpenTLD
	+ Videos from experiments and other information. Every succesful experiment should have a log attached from the flight we did.

 * The actual logging of the data is performed by the parrotLogger node, which contains an "active" instance of the DroneLogger module. The rest of the nodes can send information to the parrotLogger node though the topic "CVG/droneLogger/eventString" and the service "CVG/droneLogger/logThisString":

	+ "CVG/droneLogger/eventString" topic: this intended for repetitive events such as [ardrone;state], [ctrlr;state], [opentld], etc. To publish data on his topic use the function "void DroneLogger::publishEventString( const std::string &event_string);".
	+ "CVG/droneLogger/logThisString" service: it is intended for puntual events such as take_off, land, ... ; and puntual events such as controller configuration (gains) logging, etc. In this case, a service is used so that the caller can be sure that data is logged.

 * To avoid sending too many messages to the logger, each node piles up all the messages/data to be sent on a string. Then only one string with multiple messages in it is sent using the previous functions. Also, droneLogger has a 10 element queue for sending operations and a 30 element queue for receiving operations (on the "[...]/eventString" topic).
 
 * The messages are sent with the timestamp already written on them. For that reason each module creating messages must have a copy of init_timestamp inside. init_timestamp is determined by the droneLogger[active] module on the parrotLogger node. The timestamp can be requested by other nodes using the "bool DroneLogger::getCurrentLogPathInitTimeStamp( ros::Time &init_timestamp, std::string &currentlog_path )" function. Then the time_stamp (and logpath) can be given to the modules using the "module_name..setInitTimestamp( init_logger_timestamp );" functions.

 * The messages are automatically piled by each module internally. But the node must include code such as the following to send the information to the parrotLogger node:
	
	ParrotIBVSController.getIBVSControllerLogMsgStr( ibvsCtrlrLogMsgStr );
        if (ibvsCtrlrLogMsgStr.length() > 0)
            drone_logger.publishEventString( ibvsCtrlrLogMsgStr );

        if (!bb_object_is_on_frame)
            openTLDInterface.updateOpenTLDLogMsgStr();

        openTLDInterface.getOpenTLDLogMsgStr( opentldLogMsgStr );
        if (opentldLogMsgStr.length() > 0)
            drone_logger.publishEventString( opentldLogMsgStr );

        MyDrone.getDroneDriverLogMsgStr( droneLogMsgStr );
        if (droneLogMsgStr.length() > 0)
            drone_logger.publishEventString( droneLogMsgStr );

        MyDrone.getDroneDriverModeEventLogMsgStr( droneModeEventLogMsgStr );
        if (droneModeEventLogMsgStr.length() > 0)
            while (!drone_logger.logThisStringService( droneModeEventLogMsgStr )) {
                ros::Duration(0.005).sleep();
            }

 * To avoid piling messages in all the modules in the code (note that several nodes have instances of the same module internally, so only one of these copies will be in charge of sending the information to parrotLogger), the modules can be initialized as "logger" or "non_logger". In the case of parrot driver a configuration_tag has been added, so that one instance of the module logs navdata, and the other logs the eventmodes (take_off, landing, etc).

 * This said, the logdata is currently generated in the following nodes (some of this might change):
	+ [IBVScontrollerNodeMain] logs: [ardrone;state], [ctrlr;state], [ctrlr;gains] and [opentld]
	+ [IBVS_interface] logs: [ardrone;event] and [ardrone;image]
	+ The information [ardrone;image] is susceptible to be logged by [IBVScontrollerNodeMain] node in the future, as it has all the information necessary to generate the hud image appropiately (the controller referenes are required, and the only place where they are correct for sure are in the "active" controller module).

 * Very important!! For the log to be written complete correctly it is necessary to close the node ROS processes using control+c before using control+shift+w or control+shift+q to close the corresponding terminal tab. I have asked Mariusz about it, and even using his "screen" application it is necessary to close each tab one by one. Maybe there is a way in ROS to close all the nodes at onces (killing the ros::spins) sending a service or somethig like that. NOTE: the most comfortable way to do this is:
	+ First, make control+c on all tabs: do this using [control+c] and [control+PgUp/PgDn]
	+ [control+PgUp/PgDn] in the terminal let you navigate the terminal tabs using only the keyboard shortcut
	+ Then, close all tabs using [control+shift+w] or [control+shift+q]

* As an alternative DroneLogger logging architecture we could do the following:
	+ Mariusz suggested that to read the logs in Matlab it is easier to write logfiles where the first line specified the names of the variables in Matlab. The rest of the lines are the values of the time and the variables (in a comma/space separated values) for each logged set of values.
	+ Each node can now the init_time and localization of the logfile by querying "dronelogger".
	+ Then each node can open as many logfiles as necessary (one for each kind of logging), giving the timestamps referenced to the dronelogger initrostime value. 
	+ This alternative would decrease ros messages overhead, but then the log could not be read easily by a person. As Mariusz pointed out the logs have to be processed anyway and graphed to be usable (I share this opinion... Maybe I will move to this kind of logging in the future, we'll see). As a disadvantage the current kind of logging is more open (which is not really useful anyway): example, you can log cvgException messages (I believe this is one of the reasons why this kind of event logging was implemented that way in mavwork).
 

Cancelled tasks:
----------------

 * CANCELLED_TASK_BECAUSE(los enumerados para control modes son complicados de ) drone[IBVS]Controller y drone[IMAV2012]Controller tienen muchas cosas en comun, por tanto conviene hacer una clase padre de la que hereden ambos (para no tener que gestionar dos códigos muy similares), esta clase la voy a llamar drone[Base]Controller (Base/Bare de Empty, Vacío).

droneIBVSController:
	Publications: 
	 * [droneBaseController:isa:droneModule::active]/CVG/droneController/isStarted [std_msgs/Bool]
	 * [droneBaseController::active]/CVG/droneController/controlMode [std_msgs/Int16]
	 * [droneBaseController::active]/CVG/droneController/droneNavCommand [droneMsgs/droneNavCommand]
	 * [droneBaseController::monitor]/CVG/droneController/dronePositionRefs [droneMsgs/dronePose]
	 * [droneBaseController::monitor]/CVG/droneController/droneSpeedsRefs [droneMsgs/droneSpeeds]

	Subscriptions: 
	 * [droneBaseController:isa:droneModule::monitor]/CVG/droneController/isStarted [std_msgs/Bool]
	 * [droneBaseController::monitor]/CVG/droneController/controlMode [std_msgs/Int16]
	 * [droneBaseController::monitor]/CVG/droneController/droneNavCommand [droneMsgs/droneNavCommand]
	 * [droneBaseController::active]/CVG/droneController/dronePositionRefs [droneMsgs/dronePose]
	 * [droneBaseController::active]/CVG/droneController/droneSpeedsRefs [droneMsgs/droneSpeeds]

	Services: 
	 * [droneBaseController::active][pure_virtual_override_function]/CVG/droneController/setControlMode
	 * [droneBaseController:isa:droneModule::active]/CVG/droneController/start
	 * [droneBaseController:isa:droneModule::active]/CVG/droneController/stop
	 * [droneBaseController:isa:droneModule::active][virtual_override_function]/CVG/droneController/reset
	 > setControlMode, tiene que estar definido en cada controlador, por ejemplo, para poder rechazar modos invalidos... Que esta funcion sea pure virtual sirve para que que no se puedan instanciar objetos tipo droneBaseController. pure_virtual_function nombrePorDeterminar()
	 > reset, hay que resetear las variables del objeto child. virtual_function resetValues() de Jose...

	Cuando esto este hecho, quizas deberia hacer un refactor a varias variables para incluir el actual droneController dentro de la jerarquía.
