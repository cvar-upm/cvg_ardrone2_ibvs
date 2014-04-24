#include "DroneLogger.h"

#include <iostream>
#include <string>
#include <sstream>
#include <ostream>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream.hpp>

// the flag definitions, mask
#define MASK_RESET      0x00  // 0000 0000
#define TAG1    0x01  // 0000 0001
#define TAG2    0x02  // 0000 0010
#define TAG3    0x04  // 0000 0010
#define TAG4    0x08  // 0000 0010
#define TAG5    0x10  // 0000 0010
#define TAG6    0x20  // 0000 0010
#define TAG7    0x40  // 0000 0010
#define TAG8    0x80  // 0000 0010


namespace fs = boost::filesystem;
namespace io = boost::iostreams;

int main(int argc, char *argv[]) {

    std::cout << "Hello world!" << std::endl;

    boost::gregorian::date      init_date(boost::gregorian::day_clock::local_day());
//    boost::posix_time::ptime    init_time(boost::posix_time::second_clock::local_time());
    boost::posix_time::ptime    init_time(boost::posix_time::microsec_clock::local_time());

    std::cout << "Start time: " << init_time.time_of_day()
                 << " of"
                 << " year:" << (int) init_date.year()
                 << " month:" << std::setfill('0') << std::setw(2) << (int) init_date.month()
                 << " day:" << std::setfill('0') << std::setw(2) << (int) init_date.day()
                 << std::endl;

    std::stringstream s1, s2;
    s1 << "logs/date_"
      << "y" << (int) init_date.year()
      << "m" << std::setfill('0') << std::setw(2) << (int) init_date.month()
      << "d" << std::setfill('0') << std::setw(2) << (int) init_date.day();
    std::cout << "s1: " << s1.str() << std::endl;
    s2 << s1.str() << "/time_" << init_time.time_of_day();
    std::cout << "s2: " << s2.str() << std::endl;

    fs::path cwd = fs::current_path();
    std::cout << "cwd: " << cwd << std::endl;
    std::cout << "is_directory: " << fs::is_directory(cwd) << std::endl;

    fs::path logs_path; logs_path = (cwd / fs::path("logs"));
    std::cout << "is_directory ./" << logs_path   << ": " << fs::is_directory(logs_path) << std::endl;
    if ( !(fs::is_directory(logs_path)) ) {
        std::cout << "[] creating directory... ./" << logs_path << std::endl;
        fs::create_directory(logs_path);
        std::cout << "[] is_directory ./" << logs_path   << ": " << fs::is_directory(logs_path) << std::endl;
    }

    fs::path daylogs_path = (cwd / fs::path(s1.str()) );
    std::cout << "is_directory ./" << daylogs_path << ": " << fs::is_directory(daylogs_path) << std::endl;
    if ( !(fs::is_directory(daylogs_path)) ) {
        std::cout << "[] creating directory... ./" << daylogs_path << std::endl;
        fs::create_directory(daylogs_path);
        std::cout << "[] is_directory ./" << daylogs_path   << ": " << fs::is_directory(daylogs_path) << std::endl;
    }

    fs::path currentlog_path =(cwd / fs::path(s2.str()) );
    std::cout << "is_directory ./" << currentlog_path << ": " << fs::is_directory(currentlog_path) << std::endl;
    if ( !(fs::is_directory(currentlog_path)) ) {
        std::cout << "[] creating directory... ./" << currentlog_path << std::endl;
        fs::create_directory(currentlog_path);
        std::cout << "[] is_directory ./" << currentlog_path   << ": " << fs::is_directory(currentlog_path) << std::endl;
    }

    io::stream_buffer<io::file_sink> eventslog_buf( (currentlog_path / fs::path("events_log.txt")).string() );
    std::ostream                     eventslog_out(&eventslog_buf);

    eventslog_out << "prueba" << std::endl;
    eventslog_out << "prueba" << std::endl;

    io::stream_buffer<io::file_sink> flight_diary_buf( (currentlog_path / fs::path("flight_diary.txt")).string() );
    std::ostream                     flight_diary_out(&flight_diary_buf);

    flight_diary_out << "prueba" << std::endl;
    flight_diary_out << "prueba" << std::endl;

    std::ofstream outfile;
    outfile.open( (currentlog_path / fs::path("events_log2.txt")).string().c_str() );
    outfile << "prueba" << std::endl;
    for (int i = 0; i<100; i++)
        outfile << "prueba" << i << "\n";
//    outfile.flush();

    unsigned char my_tag = TAG1 | TAG2 | TAG6;

    if ( my_tag & TAG1 )
        printf(" ( my_tag & TAG1 )\n");
    else
        printf("!( my_tag & TAG1 )\n");

    if ( my_tag & TAG3 )
        printf(" ( my_tag & TAG3 )\n");
    else
        printf("!( my_tag & TAG3 )\n");

    return 1;
}
