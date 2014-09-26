/// ROS
#include "ros/ros.h"


#include <sstream>
#include "atlante.h"


#include <stdio.h>
#include <math.h>



//Console
#include <curses.h>


//Drone
#include "Drone.h"

//brain
#include "droneIBVSBrain.h"

//Msgs

// droneLogger
#include "DroneLogger.h"

//http://www.asciitable.com/
#define ASCII_KEY_UP      65
#define ASCII_KEY_DOWN    66

#define ASCII_KEY_RIGHT   67
#define ASCII_KEY_LEFT    68

#define ASCII_KEY_DEL      127
#define ASCII_KEY_ESC      27

//Define step commands
#define CTE_COMMAND_YAW    0.70
#define CTE_COMMAND_PITCH  0.70
#define CTE_COMMAND_ROLL   0.70
#define CTE_COMMAND_HEIGHT 0.70

// Define controller commands define constants
#define CONTROLLER_CTE_COMMAND_SPEED        ( 0.30 )
#define CONTROLLER_STEP_COMMAND_POSITTION   ( 0.025 )
#define CONTROLLER_STEP_COMMAND_ALTITUDE    ( 0.025 )
#define CONTROLLER_STEP_COMMAND_YAW         ( 1.0 * (M_PI/180.0) )

#define FREQ_INTERFACE  200.0

using namespace ros;

//void drawIBVS(cv::Mat &image, int bb_x, int bb_y, int bb_width, int bb_height, float Dxcd, float Dycd, float Sd);

int main(int argc, char **argv) {
    printf("Starting ARDrone Parrot interface...\n");
    initscr();
    start_color();

    init_pair(1, COLOR_RED,     COLOR_BLACK); // status: not started
    init_pair(2, COLOR_YELLOW,  COLOR_BLACK); // status: started but not working 100% properly
    init_pair(3, COLOR_GREEN,   COLOR_BLACK); // status: ok!
    init_pair(4, COLOR_WHITE,   COLOR_BLACK); // status: ok!

    ros::init(argc, argv, "parrotInterface");
    ros::NodeHandle n;


    //Drone
    ARDrone MyDrone(parrotDriver::full, parrotDriver::logger, PARROT_DRIVER_LOG_DRONEMODES);
    MyDrone.open(n, true);
    std::string droneLogMsgStr;
    std::string droneModeEventLogMsgStr;

    //Brain
    DroneIBVSBrain parrotIBVSBrain(droneModule::active);
    parrotIBVSBrain.open(n,"brain");

    //DroneLogger
    std::string node_name("UI");
    DroneLogger drone_logger( node_name, droneModule::monitor);
    drone_logger.open(n, "droneLogger");

    // DroneLogger - Obtaining and diseminating initial timestamp
    ros::Time init_logger_timestamp;
    std::string  currentlog_path_str;
    while (!drone_logger.getCurrentLogPathInitTimeStamp( init_logger_timestamp, currentlog_path_str)) {
        ros::Duration(0.5).sleep();
    }
    MyDrone.setInitTimestamp( init_logger_timestamp );

    ros::Rate parrotInterfaceRate(FREQ_INTERFACE);



    bool endProgram=false;


    initscr();
    curs_set(0);
    noecho();
    nodelay(stdscr, TRUE);
    erase(); refresh();


    printw("ARDrone Parrot interface"); //refresh();



    //parrot navdata
    ardrone_autonomy::Navdata parrotNavData;
    droneMsgs::droneNavData   droneNavData;
    droneNavData.altitude = 0.0;
    droneNavData.pitch    = 0.0;
    droneNavData.roll     = 0.0;
    droneNavData.speedX   = 0.0;
    droneNavData.speedY   = 0.0;
    droneNavData.yaw      = 0.0;
//    droneNavData.droneNavData_ = ros::Header;
    droneNavData.time     = 0.0;

    //parrot command
    geometry_msgs::Twist parrotCommand;
    parrotCommand.linear.x=0.0;
    parrotCommand.linear.y=0.0;
    parrotCommand.linear.z=0.0;
    parrotCommand.angular.x=0.0;
    parrotCommand.angular.y=0.0;
    parrotCommand.angular.z=0.0;
    droneMsgs::droneNavCommand droneNavCommand;
    droneNavCommand.pitch = 0.0;
    droneNavCommand.roll  = 0.0;
    droneNavCommand.dyaw  = 0.0;
    droneNavCommand.dz    = 0.0;


//    //State estimator
//    droneMsgs::dronePose dronePose;
//    droneMsgs::droneSpeeds droneSpeed;

    // OpenTLD
    ros::Time bb_timestamp = ros::Time::now();
    int     bb_x = 320-50, bb_y = 180-50, bb_width = 100, bb_height = 100;
    float   bb_confidence, bb_fps;
    bool    tracker_is_tracking     = false;
    bool    tracker_object_on_frame = false;
    ros::Time init_time = ros::Time::now();
    float Dxc = 0.0, Dyc = 0.0, Dyawc = 0.0, Dzc = 0.0;

//    // Desired image features of tracked object on image
//    float Dxcd = 0.0, Dycd = 0.0; // 360.0/4.0; // centroid, positive Dycd moves the centroid towards the bottom of the image
//    float Sd = MULTIROTOR_IBVSCONTROLLER_INITVAL_FS*MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH*MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT; // size
//    cv::Mat cam_image, cam_image_modified;
//    int image_counter = 0;
//    bool is_front_image;
//    bool is_bottom_image;

//    ros::Time image_timestamp;
//    ros::Duration image_time;

//    fs::path current_img_path, current_rectimg_path, current_hudimg_path;
//    current_img_path = fs::path( currentlog_path_str ) / fs::path("imgs");
//    current_rectimg_path = current_img_path / fs::path("rect");
//    current_hudimg_path  = current_img_path / fs::path("hud");
//    if ( !(fs::is_directory(current_img_path)) )
//        fs::create_directory(current_img_path);
//    if ( !(fs::is_directory(current_rectimg_path)) )
//        fs::create_directory(current_rectimg_path);
//    if ( !(fs::is_directory(current_hudimg_path)) )
//        fs::create_directory(current_hudimg_path);

    //controller
    droneMsgs::droneNavCommand droneCntNavComm;


    char command=0;

    //Loop
    while (ros::ok())
    {

        //Read messages
        ros::spinOnce();


        //Run
        parrotIBVSBrain.run();

        //Odometry measures
        parrotNavData=MyDrone.getNavData();
        move(1,0);
        printw("+Odometry measures:");
        int lineOdometry=2; int columOdometry=0;
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Battery=%3.1f%%\n",parrotNavData.batteryPercent); //refresh();

        //Rotations
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -RotX=% -2.2fº\n",parrotNavData.rotX); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -RotY=% -2.2fº\n",parrotNavData.rotY); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -RotZ=% -2.2fº\n",parrotNavData.rotZ); //refresh();

        //altitude
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -h=%d mm\n",parrotNavData.altd); //refresh();

        //Speeds
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Vx=% 3.5f mm/s\n",parrotNavData.vx); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Vy=% 3.5f mm/s\n",parrotNavData.vy); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Vz=% 3.5f mm/s\n",parrotNavData.vz); //refresh();
        move(lineOdometry++,columOdometry); //refresh();

        //acelerations
        printw(" -Ax=% 2.5f g\n",parrotNavData.ax); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Ay=% 2.5f g\n",parrotNavData.ay); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Az=% 2.5f g\n",parrotNavData.az); //refresh();


        //Magnetometer
        /*
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -MagX=% 3.2fº",MyDrone.navData.magX); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -MagY=% 3.2fº\n",MyDrone.navData.magY); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -MagZ=% 3.2fº\n",MyDrone.navData.magZ); //refresh();
        */


        //Command measures
        parrotCommand = MyDrone.getNavCommand();
        droneNavCommand = MyDrone.getDroneNavCommand();
        int lineCommands=2; int columCommands=25;

        //Odometry measures
        droneNavData = MyDrone.getDroneNavData();
        move(1,columCommands);
        printw("+Drone navdata:");
        move(lineCommands++,columCommands); //refresh();
        printw(" \n"); //refresh();

        //Rotations
        move(lineCommands++,columCommands); //refresh();
        printw(" -Roll=% -2.2fº\n",droneNavData.roll); //refresh();
        move(lineCommands++,columCommands); //refresh();
        printw(" -Pitch=% -2.2fº\n",droneNavData.pitch); //refresh();
        move(lineCommands++,columCommands); //refresh();
        printw(" -Yaw=% -2.2fº\n",droneNavData.yaw); //refresh();

        //altitude
        move(lineCommands++,columCommands); //refresh();
        printw(" -h=% 5.3f m\n",droneNavData.altitude); //refresh();

        //Speeds
        move(lineCommands++,columCommands); //refresh();
        printw(" -Vx=% 3.5f m/s\n",droneNavData.speedX); //refresh();
        move(lineCommands++,columCommands); //refresh();
        printw(" -Vy=% 3.5f m/s\n",droneNavData.speedY); //refresh();
        move(lineCommands++,columCommands); //refresh();
//        printw(" \n"); //refresh();
        move(lineCommands++,columCommands); //refresh();

//        move(1,columCommands);
        printw("+Commands:");
        move(lineCommands++,columCommands); //refresh();
        printw(" -Pitch=%0.3f",droneNavCommand.pitch); //refresh();
        move(lineCommands++,columCommands); //refresh();
        printw(" -Roll=%0.3f",droneNavCommand.roll); //refresh();
        move(lineCommands++,columCommands); //refresh();
        printw(" -dYaw=%0.3f",droneNavCommand.dyaw); //refresh();
        move(lineCommands++,columCommands); //refresh();
        printw(" -dz=%0.3f",droneNavCommand.dz); //refresh();
        lineCommands++;
        move(lineCommands++,columCommands); //refresh();
        printw(" -Oth1=%0.3f",parrotCommand.angular.x); //refresh();
        move(lineCommands++,columCommands); //refresh();
        printw(" -Oth2=%0.3f",parrotCommand.angular.y); //refresh();



        //State of components
        int lineComponents=2; int columComponents=50;
        move(1,columComponents);
        printw("+Components status:");

        // OpenTLD
        move(lineComponents++,columComponents);
        printw("**OpenTLD**:");
        tracker_is_tracking = parrotIBVSBrain.openTLDInterface.isTrackingObject();
        tracker_object_on_frame = parrotIBVSBrain.openTLDInterface.isObjectOnFrame();
        if ( tracker_object_on_frame )
            parrotIBVSBrain.openTLDInterface.getBoundingBox( bb_timestamp, bb_x, bb_y, bb_width, bb_height, bb_confidence, bb_fps);
        move(lineComponents++,columComponents);
        if ( tracker_is_tracking && tracker_object_on_frame ) {
            attron(COLOR_PAIR(3));
            printw("[TRACKING][IN_FRAME]");
        } else {
            if (!tracker_is_tracking) {
                attron(COLOR_PAIR(1));
                printw("[NOT_TRACKING]");
            } else { // !tracker_object_on_frame
                attron(COLOR_PAIR(2));
                printw("[TRACKING][OUT_OF_FRAME]");
            }
        }
        attron(COLOR_PAIR(4));
        move(lineComponents++,columComponents);
        printw("timestamp =%7.4f",(bb_timestamp - init_time).toSec());
        move(lineComponents++,columComponents);
        printw("x=%4d y=%4d",bb_x,bb_y);
        move(lineComponents++,columComponents);
        printw("width=%4d height=%4d",bb_width,bb_height);
        move(lineComponents++,columComponents);
//        printw("bb_confidence=%3.1f bb_fps=%3.1f",bb_confidence,bb_fps);
        printw("bb_fps=%3.2f", bb_fps);


//        //Image show
////        MyDrone.displayFrontImage("Image");
////        MyDrone.displayBottomImage("Image");
//        is_front_image = MyDrone.isFrontImageRead();
//        is_bottom_image = MyDrone.isBottomImageRead();
//        if (is_front_image) {
//            MyDrone.getFrontImage(&cam_image);
//            MyDrone.getFrontImage(&cam_image_modified);
//            image_timestamp = MyDrone.getFrontImageTimestamp();
//            is_bottom_image = false;
//        } else {
//            if (is_bottom_image) {
//                MyDrone.getBottomImage(&cam_image);
//                MyDrone.getBottomImage(&cam_image_modified);
//                image_timestamp = MyDrone.getBottomImageTimestamp();
//                is_front_image = false;
//            } else {
//                is_bottom_image = false;
//                is_front_image = false;
//            }
//        }
//        //        ROS_INFO("cam_image.size (%d,%d)", cam_image.size[0], cam_image.size[1]);
//        //        front_image: {size[0], height=360} {size[1], width=640}

//        std::ostringstream filename_oss;
//        std::ostringstream filename_timestamp;
//        if (is_front_image || is_bottom_image) {
//            if ( tracker_object_on_frame )
//                drawIBVS( cam_image_modified, bb_x, bb_y, bb_width, bb_height, Dxcd, Dycd, Sd);
//            cv::imshow( "Image", cam_image_modified);
//            cv::waitKey(1);
//            filename_oss << std::setfill('0') << std::setw(10) << image_counter;
//            image_time = image_timestamp - init_logger_timestamp;
////            imwrite( cvgString("/home/jespestana/Documents/ROS_workspace/rosWS_IMAV2013/launch_dir/imgs/hud") + "/img_" + filename_oss.str() + ".png", cam_image_modified );
////            imwrite( cvgString("/home/jespestana/Documents/ROS_workspace/rosWS_IMAV2013/launch_dir/imgs/rect") + "/img_" + filename_oss.str() + ".png", cam_image );
//            imwrite( cvgString( current_hudimg_path.string()  ) + "/img_" + filename_oss.str() + ".png", cam_image_modified );
//            imwrite( cvgString( current_rectimg_path.string() ) + "/img_" + filename_oss.str() + ".png", cam_image );
//            filename_timestamp << image_time.sec << "." << std::setfill('0') << std::setw(9) << image_time.nsec;
//            drone_logger.publishEventString(cvgString("") + filename_timestamp.str() + " [ardrone;image] " + "image:img_" + filename_oss.str() + ".png\n");
//            image_counter ++;
//        }

////        parrotIBVSBrain.openTLDInterface.getEstimatedPose(&dronePose);
////        parrotIBVSBrain.openTLDInterface.getEstimatedSpeeds(&droneSpeed);
//        // TODO_JP: Calculate xs, ys, yaws, zs, vxs, vys from OpenTLD feedback
//        double xs = 0.0, ys = 0.0, yaws = 0.0, zs = 0.7, vxs = 0.0, vys = 0.0;
////        xs   = dronePose.x;
////        ys   = dronePose.y;
////        zs   = dronePose.z;
////        yaws = dronePose.yaw;
////        vxs  = droneSpeed.dx;
////        vys  = droneSpeed.dy;
////        move(lineComponents++,columComponents);
////        printw("  -x=%f",dronePose.x);
////        move(lineComponents++,columComponents);
////        printw("  -y=%f",dronePose.y);
////        move(lineComponents++,columComponents);
////        printw("  -z=%f",dronePose.z);
////        move(lineComponents++,columComponents);
////        printw("  -yaw=%f",dronePose.yaw * (180.0/M_PI));
////        move(lineComponents++,columComponents);
////        printw("  -pitch=%f",dronePose.pitch);
////        move(lineComponents++,columComponents);
////        printw("  -roll=%f",dronePose.roll);




        //Controller
        lineComponents++;
        move(lineComponents++,columComponents); //refresh();
        int isControllerStarted = parrotIBVSBrain.droneIBVSController.isStarted();
        printw("**Controller**: %d", isControllerStarted);
        move(lineComponents++,columComponents);
        if ( isControllerStarted ) {
            if (tracker_object_on_frame) {
                attron(COLOR_PAIR(3));
                printw("[CONTROL ON]");
            } else {
                attron(COLOR_PAIR(2));
                printw("[ON][OUT_OF_FRAME]>>HOVER");
            }
        } else { // !isControllerStarted
            attron(COLOR_PAIR(1));
            printw("[CONTROL_OFF]");
        }
        attron(COLOR_PAIR(4));
        move(lineComponents++,columComponents);
        printw("ControlMode: ");
        switch(parrotIBVSBrain.droneIBVSController.getControlMode())
        {
        case DroneIBVSController::CTRL_NOT_STARTED:
            printw("NOT_STARTED\n");
            break;
        case DroneIBVSController::PERSON_FOLLOWING:
            printw("TOF_IBVS\n");
            break;
        case DroneIBVSController::PERSON_NOT_ON_FRAME:
            printw("TNOF_IBVS\n");
            break;
        }
        droneCntNavComm=parrotIBVSBrain.droneIBVSController.getNavCommand();
        move(lineComponents++,columComponents);
        printw("pitch= %f",droneCntNavComm.pitch);
        move(lineComponents++,columComponents);
        printw("roll = %f",droneCntNavComm.roll);
        move(lineComponents++,columComponents);
        printw("dz   = %f",droneCntNavComm.dz);
        move(lineComponents++,columComponents);
        printw("dyaw = %f",droneCntNavComm.dyaw);

        double xc = 0.0, yc = 0.0, yawc = 0.0, zc = 0.7, vxc = 0.0, vyc = 0.0;
//        move(lineComponents++,columComponents);
//        move(lineComponents++,columComponents);
        parrotIBVSBrain.droneIBVSController.getTargetRelPositionRefs( xc, yc, yawc, zc);
        parrotIBVSBrain.droneIBVSController.getTargetRelSpeedRefs( vxc, vyc);
//        printw("  -xc   = %f\n",xc);
//        move(lineComponents++,columComponents);
//        move(lineComponents++,columComponents);
//        printw("  -yc   = %f\n",yc);
//        move(lineComponents++,columComponents);
//        printw("  -yawc = %f\n",yawc*180.0/M_PI);
//        move(lineComponents++,columComponents);
//        printw("  -zc   = %f\n",zc);
//        move(lineComponents++,columComponents);
//        printw("  -vxc  = %f\n",vxc);
//        move(lineComponents++,columComponents);
//        printw("  -vyc  = %f\n",vyc);

        //State of components
        int lineIBVS=2; int columIBVS=75;
        move(1, columIBVS);
        printw("+IBVS features:");

        float fx,    fy,    fs,    fD;
        float fxref, fyref, fsref, fDref;
        parrotIBVSBrain.droneIBVSController.boundingBox2ImageFeatures( bb_x, bb_y, bb_width, bb_height, fx, fy, fs, fD, tracker_object_on_frame);
        parrotIBVSBrain.droneIBVSController.getImFeatReferences( fxref, fyref, fsref, fDref);

        // OpenTLD
        move(lineIBVS++,columIBVS);
        printw(">>References");
        move(lineIBVS++,columIBVS);
        printw("[R,dY] fx*= %7.4f", fxref);
        move(lineIBVS++,columIBVS);
        printw("[dz]   fy*= %7.4f", fyref);
        move(lineIBVS++,columIBVS);
        printw("[P]def fD*= %7.4f", fDref);
        move(lineIBVS++,columIBVS);
        printw("[P]    fs*= %7.4f", fsref);
        move(lineIBVS++,columIBVS);
        printw(">>Measurements");
        move(lineIBVS++,columIBVS);
        printw("[R,dY] fx = %7.4f", fx);
        move(lineIBVS++,columIBVS);
        printw("[dz]   fy = %7.4f", fy);
        move(lineIBVS++,columIBVS);
        printw("[P]def fD = %7.4f", fD);
        move(lineIBVS++,columIBVS);
        printw("[P]    fs = %7.4f", fs);
        move(lineIBVS++,columIBVS);
//        printw("Depth2target:%6.3fm", parrotIBVSBrain.droneIBVSController.distanceToTarget(fD));
        printw("Depth2target:%6.3fft", parrotIBVSBrain.droneIBVSController.distanceToTarget(fD)*3.28084); // 3.28084 feet / meter
        move(lineIBVS++,columIBVS);
        printw(">>Control error");
        move(lineIBVS++,columIBVS);
        printw("[R,dY] Dfx= %7.4f", fxref - fx);
        move(lineIBVS++,columIBVS);
        printw("[dz]   Dfy= %7.4f", fyref - fy);
        move(lineIBVS++,columIBVS);
        printw("[P]def DfD= %7.4f", fDref - fD);
        move(lineIBVS++,columIBVS);
        printw("[P]    Dfs= %7.4f", fsref - fs);

        move(20,0);
        printw("User Last Command: "); //refresh();

//        parrotIBVSBrain.droneIBVSController.setImFeatReferences( fxref, fyref, fsref, fDref);

        command=getch();
        printw("[#ASCII] %3d ; ",command);

        // Log MyDrone navdata and ModeEvent
        // I comment the navdata part, because this will not log anything
        // since MyDrone in this node is configured to save only ModeEvents
        MyDrone.getDroneDriverLogMsgStr( droneLogMsgStr );
        if (droneLogMsgStr.length() > 0)
            drone_logger.publishEventString( droneLogMsgStr );

        MyDrone.getDroneDriverModeEventLogMsgStr( droneModeEventLogMsgStr );
        if (droneModeEventLogMsgStr.length() > 0)
            while (!drone_logger.logThisStringService( droneModeEventLogMsgStr )) {
                ros::Duration(0.005).sleep();
            }

        bool controller_started = false;
        DroneIBVSController::controlMode controller_mode = DroneIBVSController::PERSON_FOLLOWING;
        controller_mode = parrotIBVSBrain.droneIBVSController.getControlMode();
        switch(command)
        {
        //        //Led Animations -> Service
        //        case 'z':

        //            printw("LED animation\n");

        //            if (MyDrone.setLedAnimation(1,10.0,10))
        //            {
        //                //ROS_INFO("Sum: %d", (bool)srv.response.result);
        //            }
        //            else
        //            {
        //                move(22,0); //refresh();
        //                //ROS_ERROR("Failed to call service ledanimation\n");
        //                printw("Failed to call service ledanimation"); refresh();
        //            }
        //            break;

        //Takeoff -> Publish Topic
        case 't':
            MyDrone.setNavCommandToZero();
            MyDrone.takeOff();
            printw("Taking off\n"); //refresh();
            break;

            //Landing -> Publish Topic
        case 'y':
            MyDrone.setNavCommandToZero();
            MyDrone.land();
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( controller_started )
                parrotIBVSBrain.droneIBVSController.stop();
            printw("Landing\n"); //refresh();
            break;

            //Emergency ??
//        case ' ':
//            MyDrone.setNavCommandToZero();
//            MyDrone.land();
//            printw("Emergency -> Landing\n"); //refresh();
//            break;

           //  Stop -> Publish Topic
        case 's':
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( !controller_started ) {
                MyDrone.setNavCommandToZero();
                printw("Stoping\n"); //refresh();
            } else {
                switch ( controller_mode ) {
                case DroneIBVSController::PERSON_NOT_ON_FRAME:
                case DroneIBVSController::PERSON_FOLLOWING:
                    Dxc = 0.0, Dyc = 0.0, Dyawc = 0.0, Dzc = 0.0;
                    parrotIBVSBrain.droneIBVSController.setTargetRelPositionRefs( Dxc, Dyc, Dyawc, Dzc, true);
                    printw("control mode: person following, stop command\n");
                    break;
                case DroneIBVSController::CTRL_NOT_STARTED:
                default:
                    break;
                }
            }
            break;

            // TODO_JL: Doubt, what is the purpose of the the reset() function?
//          //  Reset -> Publish Topic
        case ' ': // Emergency
            MyDrone.setNavCommandToZero();
            MyDrone.reset();
            printw("Reseting\n"); //refresh();
            break;


            //Hover
        case 'h':
            MyDrone.hover();
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( controller_started )
                parrotIBVSBrain.droneIBVSController.stop();
            printw("Hover\n"); //refresh();
            break;


            //Move
        case 'm':
            MyDrone.move();
            printw("Move\n"); //refresh();
            break;


            //Altitude movs
        case 'q':
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( !controller_started ) {
                droneNavCommand = MyDrone.getDroneNavCommand();
                droneNavCommand.dz = CTE_COMMAND_HEIGHT; //Move up
                MyDrone.setDroneNavCommand(droneNavCommand);
                printw("Height=%f\n",droneNavCommand.dz); //refresh();
            } else {
                switch ( controller_mode ) {
                case DroneIBVSController::PERSON_FOLLOWING:
                case DroneIBVSController::PERSON_NOT_ON_FRAME:
                    Dxc = 0.0;
                    Dyc = 0.0;
                    Dyawc = 0.0;
                    Dzc   = -CONTROLLER_STEP_COMMAND_ALTITUDE;
                    parrotIBVSBrain.droneIBVSController.setTargetRelPositionRefs(Dxc, Dyc, Dyawc, Dzc);
                    printw("control mode: person following, upwards altitude step %f\n", +CONTROLLER_STEP_COMMAND_ALTITUDE);
                    break;
                case DroneIBVSController::CTRL_NOT_STARTED:
                default:
                    break;
                }
            }
            break;

        case 'a':
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( !controller_started ) {
                droneNavCommand=MyDrone.getDroneNavCommand();
                droneNavCommand.dz = -CTE_COMMAND_HEIGHT; //Move down
                MyDrone.setDroneNavCommand(droneNavCommand);
                printw("Height=%f\n",droneNavCommand.dz); //refresh();
            } else {
                switch ( controller_mode ) {
                case DroneIBVSController::PERSON_FOLLOWING:
                case DroneIBVSController::PERSON_NOT_ON_FRAME:
                    Dxc = 0.0;
                    Dyc = 0.0;
                    Dyawc = 0.0;
                    Dzc   = +CONTROLLER_STEP_COMMAND_ALTITUDE;
                    parrotIBVSBrain.droneIBVSController.setTargetRelPositionRefs(Dxc, Dyc, Dyawc, Dzc);
                    printw("control mode: person following, downwards altitude step %f\n", -CONTROLLER_STEP_COMMAND_ALTITUDE);
                    break;
                case DroneIBVSController::CTRL_NOT_STARTED:
                default:
                    break;
                }
            }
            break;

            //Yaw
        case 'z': // leftwards, counter-clockwise
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( !controller_started ) {
                droneNavCommand = MyDrone.getDroneNavCommand();
                droneNavCommand.dyaw =-CTE_COMMAND_YAW; //turn left
                MyDrone.setDroneNavCommand(droneNavCommand);
                printw("Yaw=%f\n",droneNavCommand.dyaw); //refresh();
            } else {
                Dxc = 0.0;
                Dyc = 0.0;
                Dyawc = -CONTROLLER_STEP_COMMAND_YAW ;
                Dzc   = 0.0;
                parrotIBVSBrain.droneIBVSController.setTargetRelPositionRefs(Dxc, Dyc, Dyawc, Dzc);
                switch ( controller_mode ) {
                case DroneIBVSController::PERSON_FOLLOWING:
                case DroneIBVSController::PERSON_NOT_ON_FRAME:
                    printw("control mode: person following, yaw leftwards rotation %f\n", -CONTROLLER_STEP_COMMAND_YAW);
                    break;
                case DroneIBVSController::CTRL_NOT_STARTED:
                default:
                    break;
                }
            }
            break;

        case 'x': // rightwards,        clockwise
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( !controller_started ) {
                droneNavCommand = MyDrone.getDroneNavCommand();
                droneNavCommand.dyaw = CTE_COMMAND_YAW; //turn right
                MyDrone.setDroneNavCommand(droneNavCommand);
                printw("Yaw=%f\n",droneNavCommand.dyaw); //refresh();
            } else {
                Dxc = 0.0;
                Dyc = 0.0;
                Dyawc = +CONTROLLER_STEP_COMMAND_YAW ;
                Dzc   = 0.0;
                parrotIBVSBrain.droneIBVSController.setTargetRelPositionRefs(Dxc, Dyc, Dyawc, Dzc);
                switch ( controller_mode ) {
                case DroneIBVSController::PERSON_FOLLOWING:
                case DroneIBVSController::PERSON_NOT_ON_FRAME:
                    printw("control mode: person following, yaw rightwards rotation %f\n", +CONTROLLER_STEP_COMMAND_YAW);
                    break;
                case DroneIBVSController::CTRL_NOT_STARTED:
                default:
                    break;
                }
            }
            break;

        case '0': // set yaw to zero (parrot should head north after resposne time)
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( !controller_started ) {
                droneNavCommand = MyDrone.getDroneNavCommand();
                droneNavCommand.dyaw = CTE_COMMAND_YAW; //turn right
                MyDrone.setDroneNavCommand(droneNavCommand);
                printw("Yaw=%f\n",droneNavCommand.dyaw); //refresh();
            } else {
                yawc = 0.0;
                Dxc = 0.0; Dyc = 0.0; Dyawc = 0.0; Dzc = 0.0;
                parrotIBVSBrain.droneIBVSController.setTargetRelPositionRefs( Dxc, Dyc, Dyawc, Dzc, false, true);
                switch ( controller_mode ) {
                case DroneIBVSController::PERSON_FOLLOWING:
                case DroneIBVSController::PERSON_NOT_ON_FRAME:
                    printw("control mode: person following, center head yaw \n");
                    break;
                case DroneIBVSController::CTRL_NOT_STARTED:
                default:
                    break;
                }
            }
            break;


            //Roll
        case ASCII_KEY_RIGHT:
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( !controller_started ) {
                droneNavCommand = MyDrone.getDroneNavCommand();
                droneNavCommand.roll = CTE_COMMAND_ROLL; //Move rigth
                MyDrone.setDroneNavCommand(droneNavCommand);
                printw("Roll=%f\n",droneNavCommand.roll); //refresh();
            } else {
                switch ( controller_mode ) {
                case DroneIBVSController::PERSON_FOLLOWING:
                case DroneIBVSController::PERSON_NOT_ON_FRAME:
                    Dxc = 0.0;
                    Dyc = +CONTROLLER_STEP_COMMAND_POSITTION;
                    Dyawc = 0.0;
                    Dzc   = 0.0;
                    parrotIBVSBrain.droneIBVSController.setTargetRelPositionRefs(Dxc, Dyc, Dyawc, Dzc);
                    printw("control mode: person following, Dyc =%f\n", +CONTROLLER_STEP_COMMAND_POSITTION);
                    break;
                case DroneIBVSController::CTRL_NOT_STARTED:
                default:
                    break;
                }
            }
            break;

        case ASCII_KEY_LEFT:
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( !controller_started ) {
                droneNavCommand=MyDrone.getDroneNavCommand();
                droneNavCommand.roll = -CTE_COMMAND_ROLL; //Move left
                MyDrone.setDroneNavCommand(droneNavCommand);
                printw("Roll=%f\n",droneNavCommand.roll); //refresh();
            } else {
                switch ( controller_mode ) {
                case DroneIBVSController::PERSON_FOLLOWING:
                case DroneIBVSController::PERSON_NOT_ON_FRAME:
                    Dxc = 0.0;
                    Dyc = -CONTROLLER_STEP_COMMAND_POSITTION;
                    Dyawc = 0.0;
                    Dzc   = 0.0;
                    parrotIBVSBrain.droneIBVSController.setTargetRelPositionRefs(Dxc, Dyc, Dyawc, Dzc);
                    printw("control mode: person following, Dyc =%f\n", -CONTROLLER_STEP_COMMAND_POSITTION );
                    break;
                case DroneIBVSController::CTRL_NOT_STARTED:
                default:
                    break;
                }
            }
            break;

            //Pitch
        case ASCII_KEY_UP:
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( !controller_started ) {
                droneNavCommand=MyDrone.getDroneNavCommand();
                droneNavCommand.pitch = -CTE_COMMAND_PITCH; //Move forward
                MyDrone.setDroneNavCommand(droneNavCommand);
                printw("Pitch=%f\n",droneNavCommand.pitch); //refresh();
            } else {
                switch ( controller_mode ) {
                case DroneIBVSController::PERSON_FOLLOWING:
                case DroneIBVSController::PERSON_NOT_ON_FRAME:
                    Dxc = +CONTROLLER_STEP_COMMAND_POSITTION;
                    Dyc = 0.0;
                    Dyawc = 0.0;
                    Dzc   = 0.0;
                    parrotIBVSBrain.droneIBVSController.setTargetRelPositionRefs(Dxc, Dyc, Dyawc, Dzc);
                    printw("control mode: person following, Dxc =%f\n", +CONTROLLER_STEP_COMMAND_POSITTION );
                    break;
                case DroneIBVSController::CTRL_NOT_STARTED:
                default:
                    break;
                }
            }
            break;

        case ASCII_KEY_DOWN:
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( !controller_started ) {
                droneNavCommand=MyDrone.getDroneNavCommand();
                droneNavCommand.pitch =  CTE_COMMAND_PITCH; //Move backward
                MyDrone.setDroneNavCommand(droneNavCommand);
                printw("Pitch=%f\n",droneNavCommand.pitch); //refresh();
            } else {
                switch ( controller_mode ) {
                case DroneIBVSController::PERSON_FOLLOWING:
                case DroneIBVSController::PERSON_NOT_ON_FRAME:
                    Dxc = -CONTROLLER_STEP_COMMAND_POSITTION;
                    Dyc = 0.0;
                    Dyawc = 0.0;
                    Dzc   = 0.0;
                    parrotIBVSBrain.droneIBVSController.setTargetRelPositionRefs(Dxc, Dyc, Dyawc, Dzc);
                    printw("control mode: person following, Dxc =%f\n", -CONTROLLER_STEP_COMMAND_POSITTION );
                    break;
                case DroneIBVSController::CTRL_NOT_STARTED:
                default:
                    break;
                }
            }
            break;



            //Set front camera
        case 'c':
            MyDrone.setCamChannel(0);
            printw("Camera set to front\n"); //refresh();
            break;


            //Set bottom camera
        case 'v':
            MyDrone.setCamChannel(1);
            printw("Camera set to bottom\n"); //refresh();
            break;


        case ASCII_KEY_DEL:
            endProgram=true;
            printw("Ending..\n"); //refresh();
            break;

            //State Estimator
        case 'l':
            parrotIBVSBrain.TheDroneStateEstimator.start();
            printw("State estimator started\n");
            break;

        case 'k':
            parrotIBVSBrain.TheDroneStateEstimator.stop();
            printw("State estimator stoped\n");
            break;

        case 'j':
            parrotIBVSBrain.TheDroneStateEstimator.reset();
            printw("State estimator reseted\n");
            break;

            //Controller
        case 'o':
            //Start
//            parrotIBVSBrain.openTLDInterface.start();
            parrotIBVSBrain.droneIBVSController.start();
            printw("Controller started\n");
            break;

        case 'i':
            //stop
            MyDrone.hover();
//            ROS_INFO("parrotIBVSBrain.droneIBVSController.stop(): %d", (int) parrotIBVSBrain.droneIBVSController.stop() );
            parrotIBVSBrain.droneIBVSController.stop();
            printw("Controller stoped\n");
            break;

        case 'u':
            //Reset
            parrotIBVSBrain.droneIBVSController.reset();
            printw("Controller reseted\n");
            break;


//        case '9':
//            //control mode: traject
//            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
//            if ( !controller_started ) {
////                parrotIBVSBrain.openTLDInterface.start();
//                parrotIBVSBrain.droneIBVSController.start();
//            }
//            if(parrotIBVSBrain.droneIBVSController.setControlMode(Controller_MidLevel_controlMode::TRAJECTORY_CONTROL))
//                printw("control mode: traject\n");
//            else {
//                MyDrone.hover();
//                parrotIBVSBrain.droneIBVSController.stop();
//                printw("error changing mode\n");
//            }
//            break;

//        case '8':
//            //control mode: position
//            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
//            if ( !controller_started ) {
////                parrotIBVSBrain.openTLDInterface.start();
//                parrotIBVSBrain.droneIBVSController.start();
//            }
//            if(parrotIBVSBrain.droneIBVSController.setControlMode(Controller_MidLevel_controlMode::POSITION_CONTROL)) {
//                parrotIBVSBrain.droneIBVSController.setAllReferences2ActualValues(xs, ys, yaws, zs);
//                printw("control mode: position\n");
//            } else {
//                MyDrone.hover();
//                parrotIBVSBrain.droneIBVSController.stop();
//                printw("error changing mode\n");
//            }
//            break;

        case '7':
            //control mode: speed
            controller_started = parrotIBVSBrain.droneIBVSController.isStarted();
            if ( !controller_started ) {
//                parrotIBVSBrain.openTLDInterface.start();
                parrotIBVSBrain.droneIBVSController.start();
            }
            if(parrotIBVSBrain.droneIBVSController.setControlMode(DroneIBVSController::PERSON_FOLLOWING)) {
//                parrotIBVSBrain.droneIBVSController.setAllReferences2ActualValues(xs, ys, yaws, zs);
////                parrotIBVSBrain.droneIBVSController.setPositionRefs( xs, ys, yaws, zs);
////                parrotIBVSBrain.droneIBVSController.setVFIReference( 0.0, 0.0 );
////                parrotIBVSBrain.droneIBVSController.setPitchRollReference( 0.0, 0.0 );
                printw("control mode: speed\n");
            } else {
                MyDrone.hover();
                parrotIBVSBrain.droneIBVSController.stop();
                printw("error changing mode\n");
            }
            break;


        default:

            break;


        }






        //State info
        move(18,0); //refresh();
        printw("State: "); //refresh();

        switch(parrotNavData.state)
        {
        case 0:
            printw("Unknown"); //refresh();
            break;
        case 1:
            printw("Init"); //refresh();
            break;
        case 2:
            printw("Landed"); //refresh();
            break;
        case 3:
            printw("Flying"); //refresh();
            break;
        case 4:
            printw("Hoovering"); //refresh();
            break;
        case 5:
            printw("Test"); //refresh();
            break;
        case 6:
            printw("Taking off"); //refresh();
            break;
        case 7:
            printw("Goto Fix Point"); //refresh();
            break;
        case 8:
            printw("Landing"); //refresh();
            break;
        case 9:
            printw("Looping"); //refresh();
            break;
        }
        clrtoeol();

        //Refresh
        refresh();
        // TODO_ALL: comment this sleep
//        sleep(1);

        if(endProgram)
            break;

        parrotInterfaceRate.sleep();

    }
    endwin();

    printf("Parrot Interface ended..\n");

    return 0;
}

//void drawArrow(cv::Mat &image, cv::Point pi, cv::Point pe, cv::Scalar color) {
//    const int arrow_width = 2;
//    cv::line( image, pi, pe, color, arrow_width);

//    float L = sqrt( (pi.x-pe.x)*(pi.x-pe.x) + (pi.y-pe.y)*(pi.y-pe.y) );
//    float l = ((float)image.size[1])/40.0;
//    if ( L < l )
//        return;
//    float phi = atan2( (pe.y-pi.y), (pe.x-pi.x));

//    // Draw triangle at end of arrow
//    cv::Point pauxc, dp, p1, p2;
//    pauxc.x = pe.x - (2.0/3.0)*l*cos(phi);
//    pauxc.y = pe.y - (2.0/3.0)*l*sin(phi);
//    dp.x = (2.0/3.0)*l*cos(phi + (120.0*M_PI/180.0));
//    dp.y = (2.0/3.0)*l*sin(phi + (120.0*M_PI/180.0));
//    p1   = pauxc + dp;
//    dp.x = (2.0/3.0)*l*cos(phi - (120.0*M_PI/180.0));
//    dp.y = (2.0/3.0)*l*sin(phi - (120.0*M_PI/180.0));
//    p2   = pauxc + dp;
//    cv::line( image, p1, pe, color, arrow_width);
//    cv::line( image, p2, pe, color, arrow_width);
//    cv::line( image, p1, p2, color, arrow_width);
//    return;
//}

//void drawIBVS(cv::Mat &image, int bb_x, int bb_y, int bb_width, int bb_heigth, float Dxcd, float Dycd, float Sd ) {
//    const cv::Scalar blue_color( 214,   0,   0);
//    const cv::Scalar green_color(  0, 153,   0);
//    const cv::Scalar red_color(    0,   0, 214);
//    const cv::Scalar orange_color( 0, 102, 204);

//    // Detected target
//    cv::Point p1( bb_x, bb_y);
//    cv::Point p2( bb_x+bb_width, bb_y+bb_heigth);
//    cv::rectangle( image, p1, p2, blue_color, 3);

//    // Desired centroid
//    float xcd = ((float)image.size[1])/2.0+Dxcd;
//    float ycd = ((float)image.size[0])/2.0+Dycd;
//    // Current centroid
//    float xc = bb_x+((float)bb_width)/2.0;
//    float yc = bb_y+((float)bb_heigth)/2.0;
//    p1.x = xc;
//    p1.y = yc;
//    p2.x = xcd;
//    p2.y = ycd;
//    drawArrow( image, p1, p2, red_color);
//    cv::circle( image, p1, 2, blue_color,  4);
//    cv::circle( image, p2, 2, green_color, 4);

//    // Desired size
//    float k = sqrt( Sd/(bb_width*bb_heigth) );
//    float des_bb_w = k*bb_width;
//    float des_bb_h = k*bb_heigth;
//    // desired size error arrows
//    p1.x = xc-des_bb_w/2.0;
//    p1.y = yc-des_bb_h/2.0;
//    p2.x = xc-bb_width/2.0;
//    p2.y = yc-bb_heigth/2.0;
//    drawArrow( image, p2, p1, orange_color);
//    p1.y = yc+des_bb_h/2.0;
//    p2.y = yc+bb_heigth/2.0;
//    drawArrow( image, p2, p1, orange_color);
//    p1.x = xc+des_bb_w/2.0;
//    p2.x = xc+bb_width/2.0;
//    drawArrow( image, p2, p1, orange_color);
//    p1.y = yc-des_bb_h/2.0;
//    p2.y = yc-bb_heigth/2.0;
//    drawArrow( image, p2, p1, orange_color);
//    // desired size target rectangle
//    p1.x = xc-des_bb_w/2.0;
//    p1.y = yc-des_bb_h/2.0;
//    p2.x = xc+des_bb_w/2.0;
//    p2.y = yc+des_bb_h/2.0;
//    cv::rectangle( image, p1, p2, green_color, 3);
//    return;
//}
