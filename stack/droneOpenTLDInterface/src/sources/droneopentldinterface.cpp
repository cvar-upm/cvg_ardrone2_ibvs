#include "droneopentldinterface.h"

DroneOpenTLDInterface::DroneOpenTLDInterface(droneOpenTLDInterface::LoggerTypes logger_type_in) :
    moduleOpened(false),
    logger_type(logger_type_in)
    {

    init_timestamp = ros::Time::now();
    opentldLogMsgStrm.str(std::string());
    reset();

    return;
}

bool DroneOpenTLDInterface::reset() {
    bounding_box.x          = 0;
    bounding_box.y          = 0;
    bounding_box.width      = -1;
    bounding_box.height     = -1;
    bounding_box.confidence = 0.0;
    fps.data = -1.0;

    tracking_object         = false;
    is_object_on_frame      = false;
    received_fps_atleast_once = false;

    last_time_recieved_fps = ros::Time::now();
    last_time_recieved_bb  = ros::Time::now();

#ifdef DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION
    // Testing data corruption between rostopic subscription callbacks and getBoundingBox callbacks
    inside_bb_callback_function = false;
    inside_getbb_function       = false;
    data_corruption_counter     = 0;
#endif // DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION

    run_timestamp  = ros::Duration(0.0);

    return true;
}

DroneOpenTLDInterface::~DroneOpenTLDInterface() {
    reset();
}

void DroneOpenTLDInterface::boundingBoxSubCallback(const tld_msgs::BoundingBox::ConstPtr &msg) {
//    std::cout << "DroneOpenTLDInterface::boundingBoxSubCallback()" << std::endl;
#ifdef DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION
    inside_bb_callback_function = true;
#endif // DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION

    // modified to comply with this ros_opentld commit:
    //   https://github.com/Ronan0912/ros_opentld/commit/f07234fee2a3a6a0373a4312163e63d2d935135c
    if ( (msg->height==1) && (msg->width==1) &&
         (msg->x     ==1) && (msg->y    ==1) ) {
        return;
    } else {
        bounding_box = (*msg);

        // updating OpenTLD status information
        last_time_recieved_bb = ros::Time::now();
        is_object_on_frame = true;

#ifdef DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION
        usleep(100e3);
        inside_bb_callback_function = false;
#endif // DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION

        run_timestamp = bounding_box.header.stamp- init_timestamp;
        isObjectOnFrame();
        updateOpenTLDLogMsgStr();
        return;
    }
}

void DroneOpenTLDInterface::fpsSubCallback(const std_msgs::Float32::ConstPtr &msg) {
//    std::cout << "DroneOpenTLDInterface::fpsSubCallback()" << std::endl;
    fps = (*msg);

    // updating OpenTLD status information
    received_fps_atleast_once = true;
    last_time_recieved_fps = ros::Time::now();

    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - last_time_recieved_bb;

    if ( elapsed_time.toSec() > 3.0*(25.0/15.0)*(1.0/DRONEOPENTLDINTERFACE_TLD_EXPECTED_FPSCHANNEL_RATE) )
        is_object_on_frame = false;

    return;
}

void DroneOpenTLDInterface::open(ros::NodeHandle & nIn, std::string ardroneName) {

    if (!moduleOpened) {
        n = nIn;

        //Services

        //Topics. Publishers

        //Subscribers
        bounding_box_sub=n.subscribe(ardroneName+"/OpenTLD/tld_tracked_object", 1, &DroneOpenTLDInterface::boundingBoxSubCallback, this);
        fps_sub         =n.subscribe(ardroneName+"/OpenTLD/tld_fps", 1, &DroneOpenTLDInterface::fpsSubCallback, this);
    }

    moduleOpened  = true;

    return;
}

bool DroneOpenTLDInterface::isTrackingObject() {

    // updating OpenTLD status information
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - last_time_recieved_fps;

    if (received_fps_atleast_once)
        if ( elapsed_time.toSec() > 5.0*(1.0/DRONEOPENTLDINTERFACE_TLD_EXPECTED_FPSCHANNEL_RATE) )
            tracking_object = false;
        else
            tracking_object = true;
    else
        tracking_object = false;

    if (!tracking_object)
        is_object_on_frame = false;

    return tracking_object;
}

bool DroneOpenTLDInterface::isObjectOnFrame() {

    if ( !isTrackingObject() )
        return false;

    return is_object_on_frame;
}

void DroneOpenTLDInterface::print() {
    std::cout << "DroneOpenTLDInterface::print() " << std::endl;
    // OpenTLD status information
    std::cout << "tracking_object:" << (isTrackingObject() ? "True" : "False");
    std::cout << " is_object_on_frame:" << (is_object_on_frame ? "True" : "False") << std::endl;
    std::cout << "last tld_msgs::BoundingBox:" << std::endl;
    std::cout << bounding_box;
    std::cout << "last tld_fps:" << fps.data << std::endl;
#ifdef DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION
    std::cout << "data_corruption_counter[bb]:" << data_corruption_counter << std::endl;
#endif // DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION
}

bool DroneOpenTLDInterface::getBoundingBox(ros::Time &timestamp, int &x, int &y, int &width, int &height, float &confidence, float &current_fps) {
#ifdef DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION
    bool inside_getbb_function = true;
#endif // DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION

    timestamp   = bounding_box.header.stamp;
    current_fps = fps.data;

#ifdef DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION
    if (inside_bb_callback_function &&  inside_getbb_function)
        data_corruption_counter++;
    inside_getbb_function = false;
#endif // DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION

    isTrackingObject();
    if (is_object_on_frame && tracking_object && (bounding_box.width*bounding_box.height > 0)) {
        x           = bounding_box.x;
        y           = bounding_box.y;
        width       = bounding_box.width;
        height      = bounding_box.height;
        confidence  = bounding_box.confidence;
    }

    return is_object_on_frame;
}

void DroneOpenTLDInterface::updateOpenTLDLogMsgStr() {

    if (!tracking_object || !is_object_on_frame) {
        run_timestamp = ros::Time::now() - init_timestamp;
    }

    if ( logger_type == droneOpenTLDInterface::logger ) {
        opentldLogMsgStrm
            /* timestamp   */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
            /* tag         */   << " [opentld]"
     /* isTrackingObject() */   << " tracking:" << tracking_object
     /* isObjectOnFrame()  */   << " tof:"      << is_object_on_frame
            /* bb_x        */   << " x:"    << bounding_box.x
            /* bb_y        */   << " y:"    << bounding_box.y
            /* bb_width    */   << " w:"    << bounding_box.width
            /* bb_height   */   << " h:"    << bounding_box.height
            /*bb_confidence*/   << " conf:" << bounding_box.confidence
            /* bb_fps      */   << " fps:"  << fps.data
                                << std::endl;
    }

}

void DroneOpenTLDInterface::getOpenTLDLogMsgStr( std::string &str_in ) {

    if (logger_type == droneOpenTLDInterface::logger) {
        str_in = opentldLogMsgStrm.str();
        opentldLogMsgStrm.str(std::string());
    } else {
        str_in = "";
    }
    return;
}
