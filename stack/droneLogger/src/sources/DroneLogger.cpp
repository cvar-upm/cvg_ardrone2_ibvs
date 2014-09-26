#include "DroneLogger.h"

DroneLogger::DroneLogger( std::string &node_name_in, droneModule::droneModuleTypes droneModuleTypeIn ) :
    DroneModule(droneModuleTypeIn,100.0),
    init_date(boost::gregorian::day_clock::local_day()),
    init_time(boost::posix_time::microsec_clock::local_time()),
    node_name( node_name_in ),
    eventslog_out(&eventslog_buf),
    flight_diary_out(&flight_diary_buf)
    {
//    node_name = node_name_in;

    init();
    return;
}

DroneLogger::~DroneLogger() {
    close();
    return;
}

void DroneLogger::open(ros::NodeHandle & nIn, std::string moduleName) {

    //Node
    DroneModule::open(nIn,moduleName);

    if(droneModuleType==droneModule::active) {
        //fs::path cwd = fs::current_path();
        //std::string my_cwd=cwd.c_str();
        //ros::param::get("~stackPath",my_cwd);

        //my_cwd=stackPath;

        // creating events_logfile and fligth_diary_logfile
        //std::cout << "my_cwd:" << my_cwd << std::endl;
        std::stringstream s0, s1, s2, msg;
        s0 << stackPath << "launch_dir/logs";
        s1 << s0.str()
           << "/date_"
           << "y" << (int) init_date.year()
           << "m" << std::setfill('0') << std::setw(2) << (int) init_date.month()
           << "d" << std::setfill('0') << std::setw(2) << (int) init_date.day();
        s2 << s1.str() << "/time_" << init_time.time_of_day();

//        fs::path logs_path; logs_path = (cwd / fs::path(s0.str()));
        fs::path logs_path; logs_path = fs::path(s0.str());
        //    std::cout << "is_directory ./" << logs_path   << ": " << fs::is_directory(logs_path) << std::endl;
        if ( !(fs::is_directory(logs_path)) ) {
            msg.flush();
            msg << "[" << node_name << "]" << " info: creating directory" << logs_path;
            ROS_INFO("%s\n", msg.str().c_str() );
            fs::create_directory(logs_path);
            //        std::cout << "[] is_directory ./" << logs_path   << ": " << fs::is_directory(logs_path) << std::endl;
        }

//        fs::path daylogs_path = (cwd / fs::path(s1.str()) );
        fs::path daylogs_path = fs::path(s1.str());
        //    std::cout << "is_directory ./" << daylogs_path << ": " << fs::is_directory(daylogs_path) << std::endl;
        if ( !(fs::is_directory(daylogs_path)) ) {
            msg.flush();
            msg << "[" << node_name << "]" << " info: creating directory" << daylogs_path;
            ROS_INFO("%s\n", msg.str().c_str() );
            fs::create_directory(daylogs_path);
            //        std::cout << "[] is_directory ./" << daylogs_path   << ": " << fs::is_directory(daylogs_path) << std::endl;
        }

//        currentlog_path = (cwd / fs::path(s2.str()) );
        currentlog_path = fs::path(s2.str());
        //    std::cout << "is_directory ./" << currentlog_path << ": " << fs::is_directory(currentlog_path) << std::endl;
        if ( !(fs::is_directory(currentlog_path)) ) {
            msg.flush();
            msg << "[" << node_name << "]" << " info: creating directory" << currentlog_path;
            ROS_INFO("%s\n", msg.str().c_str() );
            fs::create_directory(currentlog_path);
            //        std::cout << "[] is_directory ./" << currentlog_path   << ": " << fs::is_directory(currentlog_path) << std::endl;
        }

        eventslog_buf.open( (currentlog_path / fs::path("events_log.txt")).string() );
//        eventslog_buf.open( (currentlog_path / fs::path("events_log.txt")).string().c_str() );
        //    eventslog_out = std::ostream(&eventslog_buf);
        eventslog_out << 0 << "." << std::setfill('0') << std::setw(9) << 0
                      << " [DroneLogger]"
                      << " init_rostime:" << init_rostime.sec << "." << std::setfill('0') << std::setw(9) << init_rostime.nsec << std::endl;

        flight_diary_buf.open( (currentlog_path / fs::path("flight_diary.txt")).string() );
        //    flight_diary_out = std::ostream(&flight_diary_buf);
//        flight_diary_out << "prueba" << std::endl;
        flight_diary_out << "Start time: " << init_time.time_of_day()
                         << " of"
                         << " year:" << (int) init_date.year()
                         << " month:" << std::setfill('0') << std::setw(2) << (int) init_date.month()
                         << " day:" << std::setfill('0') << std::setw(2) << (int) init_date.day()
                         << std::endl;
    }

    if(droneModuleType==droneModule::active)
    {
        //Services
        getCLogPathInitTStampServerSrv=n.advertiseService( moduleName+"/getCLogPathInitTStamp", &DroneLogger::getCLogPathInitTStampServCall, this);
        logThisStringServerSrv=n.advertiseService( moduleName+"/logThisString", &DroneLogger::logThisStringServCall, this);
        //Topics. Publishers
        //Subscribers
        eventStringSub=n.subscribe( moduleName+"/eventString", 30, &DroneLogger::eventStringSubCallback, this);
    }

    if(droneModuleType==droneModule::monitor)
    {
        //Services
        getCLogPathInitTStampClientSrv=n.serviceClient<droneLogger::getCLogPathInitTStamp>( moduleName+"/getCLogPathInitTStamp");
        logThisStringClientSrv=n.serviceClient<droneLogger::logThisString>( moduleName+"/logThisString");
        //Topics. Publishers
        eventStringPub=n.advertise<std_msgs::String>( moduleName+"/eventString", 10);
        //Topics. Subscriber

    }

    //Flag of module opened
    droneModuleOpened=true;


    //End
    return;
}

void DroneLogger::init() {
    init_rostime = ros::Time::now();
}

void DroneLogger::close() {
    // Note: for the buffer to be emptied into the logfile, the program must be closed with control+c
//    eventslog_buf.flush();
    if ( droneModuleType==droneModule::active ) {
        eventslog_buf.close();
        flight_diary_buf.close();
    }
    DroneModule::close();
}


bool DroneLogger::getCurrentLogPathInitTimeStamp( ros::Time &init_timestamp, std::string &currentlog_path ) {

    if(droneModuleType==droneModule::active)
    {
        // This function/method should not be called from an active node
        return false;
    }
    else
    {
        //Prepare msg
//        getCLogPathInitTStampSrvVar.request;

        //use service
        if(getCLogPathInitTStampClientSrv.call(getCLogPathInitTStampSrvVar))
        {
            init_timestamp  = getCLogPathInitTStampSrvVar.response.init_stamp;
            currentlog_path = getCLogPathInitTStampSrvVar.response.currentlog_path;
            return getCLogPathInitTStampSrvVar.response.ack;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool DroneLogger::logThisStringService( std::string &str2belogged_in ) {

    if(droneModuleType==droneModule::active)
    {
        // This function/method should not be called from an active node
        return false;
    }
    else
    {
        //Prepare msg
        logThisStringSrvVar.request.str2belogged = str2belogged_in;

        //use service
        if(logThisStringClientSrv.call(logThisStringSrvVar))
        {
            return logThisStringSrvVar.response.ack;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool DroneLogger::getCLogPathInitTStampServCall(droneLogger::getCLogPathInitTStamp::Request& request, droneLogger::getCLogPathInitTStamp::Response& response) {

    response.init_stamp         = init_rostime;
    response.currentlog_path    = currentlog_path.string();
    response.ack = true;
    return response.ack;
}

bool DroneLogger::logThisStringServCall(droneLogger::logThisString::Request& request, droneLogger::logThisString::Response& response) {

    if(droneModuleOpened==false || droneModuleType==droneModule::monitor) {
        response.ack = false;
    } else {
        eventslog_out << request.str2belogged;
        response.ack = true;
    }

    return response.ack;
}



void DroneLogger::eventStringSubCallback(const std_msgs::String::ConstPtr &msg) {

    if(droneModuleOpened==false || droneModuleType==droneModule::monitor)
        return;

//    eventslog_out << msg->data << std::endl;
    eventslog_out << msg->data;
}

void DroneLogger::publishEventString(const std::string &event_string) { //Publish function

    if(droneModuleOpened==false || droneModuleType==droneModule::active)
        return;

    eventStringMsg.data = event_string;
    eventStringPub.publish( eventStringMsg );
}

