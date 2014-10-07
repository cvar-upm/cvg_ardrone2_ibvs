#ifndef DRONEOPENTLDINTERFACE_H
#define DRONEOPENTLDINTERFACE_H

#include <string>
#include <ostream>

////// ROS  ///////
#include "ros/ros.h"

// module dependencies
//#include "droneModule.h"

#include "tld_msgs/BoundingBox.h"
#include "std_msgs/Float32.h"

// rostopic hz /OpenTLD/tld_fps
#define DRONEOPENTLDINTERFACE_TLD_EXPECTED_FPSCHANNEL_RATE ( 15.0 )
//#define DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION

#ifdef DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION
#include <unistd.h>
#endif // DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION

namespace droneOpenTLDInterface
{
    enum LoggerTypes
    {
        non_logger, // logs data
        logger      // does not log data
    };
}

class DroneOpenTLDInterface
{
private:
    ros::NodeHandle n;
    // OpenTLD related resources
    ros::Subscriber bounding_box_sub;
    ros::Subscriber fps_sub;
    tld_msgs::BoundingBox bounding_box;
    std_msgs::Float32 fps;
    void boundingBoxSubCallback(const tld_msgs::BoundingBox::ConstPtr &msg);
    void fpsSubCallback(const std_msgs::Float32::ConstPtr &msg);
protected:
    bool reset();

private: // OpenTLD node status
    bool tracking_object;
    bool is_object_on_frame;
    bool received_fps_atleast_once;
    ros::Time last_time_recieved_fps;
    ros::Time last_time_recieved_bb;

public:
    DroneOpenTLDInterface( droneOpenTLDInterface::LoggerTypes logger_type_in = droneOpenTLDInterface::non_logger);
    ~DroneOpenTLDInterface();

private: bool moduleOpened;
public:
    void open(ros::NodeHandle & nIn, std::string ardroneName);
    bool isTrackingObject();
    bool isObjectOnFrame();
    void print();
    bool getBoundingBox( ros::Time &timestamp, int &x, int &y, int &width, int &height, float &confidence, float &current_fps);

#ifdef DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION
private: // Testing data corruption between rostopic subscription callbacks and getBoundingBox callbacks
    bool inside_bb_callback_function;
    bool inside_getbb_function;
    int  data_corruption_counter;
#endif // DRONEOPENTLDINTERFACE_TEST_CALLBACK_GET_DATA_CORRUPTION

    // DroneLogger
private:
    droneOpenTLDInterface::LoggerTypes logger_type;
    ros::Duration run_timestamp;
    ros::Time     init_timestamp;
    std::ostringstream opentldLogMsgStrm;

public:
    void updateOpenTLDLogMsgStr();
    inline void setInitTimestamp( const ros::Time &init_timestamp_in) { init_timestamp = init_timestamp_in; }
    void        getOpenTLDLogMsgStr( std::string &str_in );
};

#endif // DRONEOPENTLDINTERFACE_H
