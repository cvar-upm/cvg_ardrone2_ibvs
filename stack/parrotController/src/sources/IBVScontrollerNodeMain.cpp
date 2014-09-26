#include <stdio.h>
#include <iostream>

// ROS
#include "ros/ros.h"

//parrotDriver
#include "Drone.h"

#include <sstream>
//#include "atlante.h"

// OpenTLD
#include "tld_msgs/BoundingBox.h"
#include "std_msgs/Float32.h"
#include "droneopentldinterface.h"

//controller
#include "droneIBVSController.h"

// DroneLogger - include
#include "DroneLogger.h"
#include "communication_definition.h"

#define FRONT_CAM_SAVE_IMAGES_2DISK

void drawIBVS(cv::Mat &image, int bb_x, int bb_y, int bb_width, int bb_heigth, float Dxcd, float Dycd, float Sd);
void drawIBVS_v2(cv::Mat &image, int bb_x, int bb_y, int bb_width, int bb_heigth, float Dxcd, float Dycd, float Sd, float Dxs2, float Dys2);
void drawText(cv::Mat &image, bool tracker_is_tracking, bool tracker_object_on_frame, bool isControllerStarted);


int main(int argc, char **argv)
{
    printf("Starting ARDrone Parrot IBVS Controller...\n");

    ros::init(argc, argv, "parrotIBVSController");
    ros::NodeHandle n;

    //Drone
    ARDrone MyDrone(parrotDriver::full, parrotDriver::logger, PARROT_DRIVER_LOG_NAVDATA | PARROT_DRIVER_LOG_DRONEMODES); //Bastaría con monitor???
    MyDrone.open(n, true);
    std::string droneLogMsgStr;          // DroneLogger - String
    std::string droneModeEventLogMsgStr; // DroneLogger - String
    //Nav Commands
    droneMsgs::droneNavCommand droneNavCommands;

    //OpenTLD
    DroneOpenTLDInterface openTLDInterface( droneOpenTLDInterface::logger );
    openTLDInterface.open( n, "ardrone");
    std::string opentldLogMsgStr; // DroneLogger - log string

    //Controller
    DroneIBVSController ParrotIBVSController(droneModule::active, droneModule::logger);
    ParrotIBVSController.open(n,MODULE_NAME_TRAJECTORY_CONTROLLER);
    std::string ibvsCtrlrLogMsgStr; // DroneLogger - log string

    // DroneLogger - Object initialization
    std::string node_name("IBVS-Controller");
    DroneLogger drone_logger( node_name, droneModule::monitor);
    drone_logger.open(n, "droneLogger");

    // DroneLogger - Obtaining and diseminating initial timestamp
    ros::Time init_logger_timestamp;
    std::string  currentlog_path_str;
    while (!drone_logger.getCurrentLogPathInitTimeStamp( init_logger_timestamp, currentlog_path_str)) {
        ros::Duration(0.5).sleep();
    }
    ParrotIBVSController.setInitTimestamp( init_logger_timestamp );
    openTLDInterface.setInitTimestamp( init_logger_timestamp );
    MyDrone.setInitTimestamp( init_logger_timestamp );

    // DroneLogger - Saving controller gains
    ParrotIBVSController.getIBVSControllerLogMsgStr( ibvsCtrlrLogMsgStr );
    while (!drone_logger.logThisStringService( ibvsCtrlrLogMsgStr )) {
        ros::Duration(0.5).sleep();
    }

    // Hud and rect image
    // Desired image features of tracked object on image
    float fxci = 0.0, fyci = 0.0, fsci = 0.1, fDci = 1.0/sqrt(0.1);
    cv::Mat cam_image, cam_image_modified;
    uint32_t last_image_seq_num = -1, image_seq_num = -1;
    int image_counter = 0;
    bool is_front_image;
    bool is_bottom_image;
    ros::Time image_timestamp;
    ros::Duration image_time;
    std::ostringstream filename_oss;        // DroneLogger - Image logging - image_counter %d10
    std::ostringstream filename_timestamp;  // DroneLogger - Image logging - timestamp > str
    std::ostringstream ss;                  // DroneLogger - Image logging - log string
    // DroneLogger - Creating image logging paths
    fs::path current_img_path, current_rectimg_path, current_hudimg_path;
    current_img_path = fs::path( currentlog_path_str ) / fs::path("imgs");
    current_rectimg_path = current_img_path / fs::path("rect");
    current_hudimg_path  = current_img_path / fs::path("hud");
    if ( !(fs::is_directory(current_img_path)) )
        fs::create_directory(current_img_path);
    if ( !(fs::is_directory(current_rectimg_path)) )
        fs::create_directory(current_rectimg_path);
    if ( !(fs::is_directory(current_hudimg_path)) )
        fs::create_directory(current_hudimg_path);

//  DroneLogger - This thing never worked, because I could not give a pointer to a member function
//    ParrotIBVSController.setLoggingFunction( &(drone_logger.publishEventString) );
//    ParrotIBVSController.setDroneLogger( &drone_logger );

    ros::Time bb_timestamp;
    int     bb_x, bb_y, bb_width, bb_height;
    float   bb_confidence, bb_fps;
    bool    bb_object_is_on_frame = false;
//    bool    bb_opentld_is_tracking = false;

    // Controller
    float  fxs,  fys,  fss,   fDs;   // image feature measurements
    droneMsgs::droneNavData drone_navdata;
    double yaw_t, pitch_t, roll_t;

    //Loop
    while (ros::ok())
    {
        //Read messages
        ros::spinOnce();

//        openTLDInterface.print();
//        std::cout << std::endl << std::endl;

        // get controller feedback
//        bb_opentld_is_tracking =openTLDInterface.isTrackingObject();
        bb_object_is_on_frame = openTLDInterface.getBoundingBox( bb_timestamp, bb_x, bb_y, bb_width, bb_height, bb_confidence, bb_fps);
        drone_navdata = MyDrone.getDroneNavData();
        yaw_t   = drone_navdata.yaw  *M_PI/180.0;
        pitch_t = drone_navdata.pitch*M_PI/180.0;
        roll_t  = drone_navdata.roll *M_PI/180.0;

        // set controller feedback
        ParrotIBVSController.setTelemetryAttitude_rad( yaw_t, pitch_t, roll_t);
        ParrotIBVSController.setTargetIsOnFrame(bb_object_is_on_frame);
        ParrotIBVSController.boundingBox2ImageFeatures( bb_x, bb_y, bb_width, bb_height, fxs, fys, fss, fDs, bb_object_is_on_frame);
        ParrotIBVSController.setImFeatMeasurements( fxs, fys, fss, fDs, bb_width, bb_height);

        // DONE_JP: implementada maquina de estados para hacer un seguimiento logico del control, se ejecuta en droneIBVSController::run()
        //Run controller
        if(ParrotIBVSController.run()) {
            //Controller
            //Set commands
            droneNavCommands=ParrotIBVSController.getNavCommand();

            //Send command to the parrot
            MyDrone.setDroneNavCommand(droneNavCommands);
        }

        // DroneLogger - ParrotIBVSController - Retrieving and logging stacked log string
        ParrotIBVSController.getIBVSControllerLogMsgStr( ibvsCtrlrLogMsgStr );
        if (ibvsCtrlrLogMsgStr.length() > 0)
            drone_logger.publishEventString( ibvsCtrlrLogMsgStr );

        // DroneLogger - openTLDInterface - Retrieving and logging stacked log string
        if (!bb_object_is_on_frame)     // want to log even when object is not on frame
            openTLDInterface.updateOpenTLDLogMsgStr();
        openTLDInterface.getOpenTLDLogMsgStr( opentldLogMsgStr );
        if (opentldLogMsgStr.length() > 0)
            drone_logger.publishEventString( opentldLogMsgStr );

        // DroneLogger - MyDrone - Retrieving and logging stacked log string
        MyDrone.getDroneDriverLogMsgStr( droneLogMsgStr );
        if (droneLogMsgStr.length() > 0)
            drone_logger.publishEventString( droneLogMsgStr );
        // DroneLogger - MyDrone - Same with event logging
        MyDrone.getDroneDriverModeEventLogMsgStr( droneModeEventLogMsgStr );
        if (droneModeEventLogMsgStr.length() > 0)
            while (!drone_logger.logThisStringService( droneModeEventLogMsgStr )) {
                ros::Duration(0.005).sleep();
            }

        // Hud and rect image
        //Image show
//        MyDrone.displayFrontImage("Image");
//        MyDrone.displayBottomImage("Image");
        is_front_image = MyDrone.isFrontImageRead();
        is_bottom_image = MyDrone.isBottomImageRead();
        ParrotIBVSController.getImFeatReferences( fxci, fyci, fsci, fDci);
        // std::cout << "fyci:" << fyci << " fys:" << fys << std::endl;
        float fxs_4DyC, fxs_4DYC, fys_4DzC, fDs_4DxC;
        ParrotIBVSController.getImFeatFeedback2PIDs(fxs_4DyC, fxs_4DYC, fys_4DzC, fDs_4DxC);
        if (is_front_image) {
//#ifdef FRONT_CAM_SAVE_IMAGES_2DISK
            MyDrone.getFrontImage(&cam_image, &image_seq_num);
//#endif // FRONT_CAM_SAVE_IMAGES_2DISK
            MyDrone.getFrontImage(&cam_image_modified);
            image_timestamp = MyDrone.getFrontImageTimestamp();
            is_bottom_image = false;
        } else {
            if (is_bottom_image) {
                MyDrone.getBottomImage(&cam_image, &image_seq_num);
                MyDrone.getBottomImage(&cam_image_modified);
                image_timestamp = MyDrone.getBottomImageTimestamp();
                is_front_image = false;
            } else {
                is_bottom_image = false;
                is_front_image = false;
            }
        }
        //        ROS_INFO("cam_image.size (%d,%d)", cam_image.size[0], cam_image.size[1]);
        //        front_image: {size[0], height=360} {size[1], width=640}

        if ( (is_front_image || is_bottom_image) && (last_image_seq_num < image_seq_num) ) {
            // DroneLogger - Flushing string streams
            filename_oss.str(std::string());
            filename_timestamp.str(std::string());
            bool tracker_is_tracking, tracker_object_on_frame, isControllerStarted;
            tracker_is_tracking = openTLDInterface.isTrackingObject();
            tracker_object_on_frame = openTLDInterface.isObjectOnFrame();
            isControllerStarted = ParrotIBVSController.isStarted();
            drawText( cam_image_modified, tracker_is_tracking, tracker_object_on_frame, isControllerStarted);
            if ( bb_object_is_on_frame ) {
                float fxci_scc = (fxci - 0.5)*MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH;
                float fyci_scc = (fyci - 0.5)*MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT;
                float fsci_scc = MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH*MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT/(fDci*fDci);
                float fxs_DyC_scc = (fxs_4DyC - 0.5)*MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH;
                float fys_DzC_scc = (fys_4DzC - 0.5)*MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT;
//                drawIBVS( cam_image_modified, bb_x, bb_y, bb_width, bb_height, fxci_scc, fyci_scc, fsci_scc);
                drawIBVS_v2( cam_image_modified, bb_x, bb_y, bb_width, bb_height, fxci_scc, fyci_scc, fsci_scc, fxs_DyC_scc, fys_DzC_scc);
            }
            cv::imshow( "Image", cam_image_modified);
            cv::waitKey(1);
#ifdef FRONT_CAM_SAVE_IMAGES_2DISK
            filename_oss << std::setfill('0') << std::setw(10) << image_counter;
            image_time = image_timestamp - init_logger_timestamp;
            ss.str(std::string());
            ss << current_hudimg_path.string() << "/img_" << filename_oss.str() << ".png";
            cv::imwrite( ss.str(), cam_image_modified);
            ss.str(std::string());
            ss << current_rectimg_path.string() << "/img_" << filename_oss.str() << ".png";
            cv::imwrite( ss.str(), cam_image );
            filename_timestamp << image_time.sec << "." << std::setfill('0') << std::setw(9) << image_time.nsec;
            ss.str(std::string());
            ss << filename_timestamp.str() << " [ardrone;image] " << "image:img_" << filename_oss.str() << ".png\n";
            drone_logger.publishEventString( ss.str() );
            image_counter ++;
#endif // FRONT_CAM_SAVE_IMAGES_2DISK
        }
        last_image_seq_num = image_seq_num;

        //Sleep
        ParrotIBVSController.sleep();
    }
    return 1;
}

void drawArrow(cv::Mat &image, cv::Point pi, cv::Point pe, cv::Scalar color) {
    const int arrow_width = 2;
    cv::line( image, pi, pe, color, arrow_width);

    float L = sqrt( (pi.x-pe.x)*(pi.x-pe.x) + (pi.y-pe.y)*(pi.y-pe.y) );
    float l = ((float)image.size[1])/40.0;
    if ( L < l )
        return;
    float phi = atan2( (pe.y-pi.y), (pe.x-pi.x));

    // Draw triangle at end of arrow
    cv::Point pauxc, dp, p1, p2;
    pauxc.x = pe.x - (2.0/3.0)*l*cos(phi);
    pauxc.y = pe.y - (2.0/3.0)*l*sin(phi);
    dp.x = (2.0/3.0)*l*cos(phi + (120.0*M_PI/180.0));
    dp.y = (2.0/3.0)*l*sin(phi + (120.0*M_PI/180.0));
    p1   = pauxc + dp;
    dp.x = (2.0/3.0)*l*cos(phi - (120.0*M_PI/180.0));
    dp.y = (2.0/3.0)*l*sin(phi - (120.0*M_PI/180.0));
    p2   = pauxc + dp;
    cv::line( image, p1, pe, color, arrow_width);
    cv::line( image, p2, pe, color, arrow_width);
    cv::line( image, p1, p2, color, arrow_width);
    return;
}

void drawIBVS(cv::Mat &image, int bb_x, int bb_y, int bb_width, int bb_heigth, float Dxcd, float Dycd, float Sd) {
    const cv::Scalar blue_color( 214,   0,   0);
    const cv::Scalar green_color(  0, 153,   0);
    const cv::Scalar red_color(    0,   0, 214);
    const cv::Scalar orange_color( 0, 102, 204);

    // Detected target
    cv::Point p1( bb_x, bb_y);
    cv::Point p2( bb_x+bb_width, bb_y+bb_heigth);
    cv::rectangle( image, p1, p2, blue_color, 3);

    // Desired centroid
    float xcd = ((float)image.size[1])/2.0+Dxcd;
    float ycd = ((float)image.size[0])/2.0+Dycd;
    // Current centroid
    float xc = bb_x+((float)bb_width)/2.0;
    float yc = bb_y+((float)bb_heigth)/2.0;
    p1.x = xc;
    p1.y = yc;
    p2.x = xcd;
    p2.y = ycd;
    drawArrow( image, p1, p2, red_color);
    cv::circle( image, p1, 2, blue_color,  4);
    cv::circle( image, p2, 2, green_color, 4);

    // Desired size
    float k = sqrt( Sd/(bb_width*bb_heigth) );
    float des_bb_w = k*bb_width;
    float des_bb_h = k*bb_heigth;
    // desired size error arrows
    p1.x = xc-des_bb_w/2.0;
    p1.y = yc-des_bb_h/2.0;
    p2.x = xc-bb_width/2.0;
    p2.y = yc-bb_heigth/2.0;
    drawArrow( image, p2, p1, orange_color);
    p1.y = yc+des_bb_h/2.0;
    p2.y = yc+bb_heigth/2.0;
    drawArrow( image, p2, p1, orange_color);
    p1.x = xc+des_bb_w/2.0;
    p2.x = xc+bb_width/2.0;
    drawArrow( image, p2, p1, orange_color);
    p1.y = yc-des_bb_h/2.0;
    p2.y = yc-bb_heigth/2.0;
    drawArrow( image, p2, p1, orange_color);
    // desired size target rectangle
    p1.x = xc-des_bb_w/2.0;
    p1.y = yc-des_bb_h/2.0;
    p2.x = xc+des_bb_w/2.0;
    p2.y = yc+des_bb_h/2.0;
    cv::rectangle( image, p1, p2, green_color, 3);

    return;
}

void drawIBVS_v2(cv::Mat &image, int bb_x, int bb_y, int bb_width, int bb_heigth, float Dxcd, float Dycd, float Sd, float Dxs2, float Dys2) {
    const cv::Scalar blue_color( 214,   0,   0);
    const cv::Scalar green_color(  0, 153,   0);
    const cv::Scalar red_color(    0,   0, 214);
    const cv::Scalar orange_color( 0, 102, 204);

    // Detected target
    cv::Point p1( bb_x, bb_y);
    cv::Point p2( bb_x+bb_width, bb_y+bb_heigth);
    cv::rectangle( image, p1, p2, blue_color, 3);

    // Desired centroid
    float xcd = ((float)image.size[1])/2.0+Dxcd;
    float ycd = ((float)image.size[0])/2.0+Dycd;
    // Current centroid
    float xc = bb_x+((float)bb_width)/2.0;
    float yc = bb_y+((float)bb_heigth)/2.0;
    p1.x = xc;
    p1.y = yc;
    p2.x = xcd;
    p2.y = ycd;
//    drawArrow( image, p1, p2, red_color);
    cv::circle( image, p1, 2, blue_color,  4);
    cv::circle( image, p2, 2, green_color, 4);

    // Desired size
    float k = sqrt( Sd/(bb_width*bb_heigth) );
    float des_bb_w = k*bb_width;
    float des_bb_h = k*bb_heigth;
    // desired size error arrows
    p1.x = xc-des_bb_w/2.0;
    p1.y = yc-des_bb_h/2.0;
    p2.x = xc-bb_width/2.0;
    p2.y = yc-bb_heigth/2.0;
    drawArrow( image, p2, p1, orange_color);
    p1.y = yc+des_bb_h/2.0;
    p2.y = yc+bb_heigth/2.0;
    drawArrow( image, p2, p1, orange_color);
    p1.x = xc+des_bb_w/2.0;
    p2.x = xc+bb_width/2.0;
    drawArrow( image, p2, p1, orange_color);
    p1.y = yc-des_bb_h/2.0;
    p2.y = yc-bb_heigth/2.0;
    drawArrow( image, p2, p1, orange_color);
    // desired size target rectangle
    p1.x = xc-des_bb_w/2.0;
    p1.y = yc-des_bb_h/2.0;
    p2.x = xc+des_bb_w/2.0;
    p2.y = yc+des_bb_h/2.0;
    cv::rectangle( image, p1, p2, green_color, 3);

    // Desired centroid
    xcd = ((float)image.size[1])/2.0+Dxcd;
    ycd = ((float)image.size[0])/2.0+Dycd;
    // Current centroid
    xc = ((float)image.size[1])/2.0+Dxs2;
    yc = ((float)image.size[0])/2.0+Dys2;
    // Current centroid
    float xc2 = bb_x+((float)bb_width)/2.0;
    float yc2 = bb_y+((float)bb_heigth)/2.0;
//    p1.x = xc;
//    p1.y = yc;
//    p2.x = xcd;
//    p2.y = ycd;
    p1.x = xcd;
    p1.y = ycd;
    p2.x = xcd+(xc-xcd);
    p2.y = ycd+(yc-ycd);
    drawArrow( image, p1, p2, orange_color);
// //    cv::circle( image, p1, 2, blue_color,  4);
// //    cv::circle( image, p2, 2, green_color, 4);

    return;
}

void drawText(cv::Mat &image, bool tracker_is_tracking, bool tracker_object_on_frame, bool isControllerStarted) {
    // Add controller state and tracker state to HUD image
//    const cv::Scalar blue_color( 214,   0,   0);
    const cv::Scalar green_color(  0, 153,   0);
    const cv::Scalar red_color(    0,   0, 214);
    const cv::Scalar orange_color( 0, 102, 204);
    const double font_scale = 0.6;
    const float text_size = 40*font_scale;
    const int text_thickness = 2;
    const int first_line_position = 7;

    cv::Point p1;
    p1.x = 0;
    p1.y = MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT - text_size - first_line_position;

    if ( tracker_is_tracking && tracker_object_on_frame ) {
        cv::putText( image, "[TRACKER][ON]", p1, cv::FONT_HERSHEY_SIMPLEX, font_scale, green_color, text_thickness);
    } else {
        if (!tracker_is_tracking) {
            cv::putText( image, "[TRACKER][OFF]", p1, cv::FONT_HERSHEY_SIMPLEX, font_scale, red_color, text_thickness);
        } else { // !tracker_object_on_frame
            cv::putText( image, "[TRACKER][ON][OUT_OF_FRAME]", p1, cv::FONT_HERSHEY_SIMPLEX, font_scale, orange_color, text_thickness);
        }
    }

    p1.x = 0;
    p1.y = MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT - first_line_position;

    if ( isControllerStarted ) {
        if (tracker_object_on_frame) {
            cv::putText( image, "[CONTROL][ON]", p1, cv::FONT_HERSHEY_SIMPLEX, font_scale, green_color, text_thickness);
        } else {
            cv::putText( image, "[CONTROL][ON][OFM]>>HOVER", p1, cv::FONT_HERSHEY_SIMPLEX, font_scale, orange_color, text_thickness);
        }
    } else { // !isControllerStarted
        cv::putText( image, "[CONTROL][OFF]", p1, cv::FONT_HERSHEY_SIMPLEX, font_scale, red_color, text_thickness);
    }

    return;
}


