#ifndef DRONELOGGER_H
#define DRONELOGGER_H

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <sstream>
#include <ostream>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"
// Boost::date_time :
// http://www.boost.org/doc/libs/1_51_0/doc/html/date_time/posix_time.html#date_time.posix_time.ptime_class
#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/o
//#include "boost/date_time/posix_time/posix_time_types.hpp" //no i/o just types
// http://www.boost.org/doc/libs/1_42_0/doc/html/date_time/examples.html
#include "boost/date_time/gregorian/gregorian.hpp"
//#include <atlante.h>
#include "droneModule.h"
#include "std_msgs/String.h"
// services
#include "droneLogger/getCLogPathInitTStamp.h"
#include "droneLogger/logThisString.h"

namespace fs = boost::filesystem;
namespace io = boost::iostreams;

class DroneLogger : public DroneModule {
private:
    boost::gregorian::date      init_date;
    boost::posix_time::ptime    init_time;
    ros::Time                   init_rostime;
    std::string                 node_name;

    ros::Publisher      eventStringPub;
    ros::Subscriber     eventStringSub;
    std_msgs::String    eventStringMsg;

    fs::path currentlog_path;

    io::stream_buffer<io::file_sink> eventslog_buf;
//    std::ofstream                   eventslog_buf;
    std::ostream                     eventslog_out;

    io::stream_buffer<io::file_sink> flight_diary_buf;
    std::ostream                     flight_diary_out;

    // Services
        // getCLogPathInitTStamp
    ros::ServiceServer getCLogPathInitTStampServerSrv;
    ros::ServiceClient getCLogPathInitTStampClientSrv;
    droneLogger::getCLogPathInitTStamp getCLogPathInitTStampSrvVar;
        // logThisString
    ros::ServiceServer logThisStringServerSrv;
    ros::ServiceClient logThisStringClientSrv;
    droneLogger::logThisString logThisStringSrvVar;

public:
    bool getCurrentLogPathInitTimeStamp( ros::Time &init_timestamp, std::string &currentlog_path );
    bool logThisStringService( std::string &str2belogged );
private:
    bool getCLogPathInitTStampServCall(droneLogger::getCLogPathInitTStamp::Request& request, droneLogger::getCLogPathInitTStamp::Response& response);
    bool logThisStringServCall(droneLogger::logThisString::Request& request, droneLogger::logThisString::Response& response);

protected:
    void eventStringSubCallback(const std_msgs::String::ConstPtr &msg);

public:
    DroneLogger( std::string &node_name_in, droneModule::droneModuleTypes droneModuleTypeIn=droneModule::monitor);
    ~DroneLogger();

    void open(ros::NodeHandle & nIn, std::string moduleName);
    void init();
    void close();
    void publishEventString( const std::string &event_string); //Publish function

};

#endif // DRONELOGGER_H
