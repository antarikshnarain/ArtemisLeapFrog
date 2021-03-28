// Standard Libs
#include <string>

using namespace std;

// Boost Libs
#define BOOST_LOG_DYN_LINK 1
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;
namespace severity = boost::log::trivial;

void initialize()
{
    //logging::add_file_log("sample.log");
    logging::add_file_log(
        keywords::file_name = "sample_%N.log",
        keywords::rotation_size = 10 * 1024 * 1024, // 10 MB
        keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),
        keywords::format = "%LineID% [%TimeStamp%]: %Message%");

    logging::core::get()->set_filter(
        logging::trivial::severity >= logging::trivial::info);
}

// void Log(string data, auto mode = info)
// {
//     BOOST_LOG_TRIVIAL(mode) << data;
// }

int main()
{
    initialize();
    BOOST_LOG_TRIVIAL(info) << "Hello-1";
    BOOST_LOG_TRIVIAL(warning) << "Hello-2";
    BOOST_LOG_TRIVIAL(error) << "Hello-3";
    BOOST_LOG_TRIVIAL(fatal) << "Hello-4";
    //Log("Hello1-Info", info);
    //Log("Hello1-Warn", warning);
    //Log("Hello1-Error", error);
    //Log("Hello1-Fatal", fatal);
}
