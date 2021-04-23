/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Base Property Class
----------------------------------------------------------------- */

// Standard Libs
#include <string>

using namespace std;

// Boost Libs
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

// Pre-processors Constants
#define DIMS 3
#define NUM_BATTERY 2
#define NUM_COLDGAS 4
#define NUM_THRUSTERS 6
#define DIMS_GIMBAL 2

namespace Properties
{
#ifndef _PROPERTIES_BASE_HPP
#define _PROPERTIES_BASE_HPP
    class Base
    {
    public:
        // \brief Pretty Print format for class
        // \param prop: Property Class object
        virtual void print() = 0;

        // \brief Default Constructor to initialize Logging
        Base()
        {
            // //logging::add_file_log("sample.log");
            // logging::add_file_log(
            //     keywords::file_name = "sample_%N.log",
            //     keywords::rotation_size = 10 * 1024 * 1024, // 10 MB
            //     keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),
            //     keywords::format = "%LineID% [%TimeStamp%]: %Message%");

            // logging::core::get()->set_filter(
            //     logging::trivial::severity >= logging::trivial::info);

            // this->Logger();
        }
        // TODO: Add Logging functionality
        void Logger()
        {
            // BOOST_LOG_TRIVIAL(trace) << "A trace severity message";
            // BOOST_LOG_TRIVIAL(debug) << "A debug severity message";
            // BOOST_LOG_TRIVIAL(info) << "An informational severity message";
            // BOOST_LOG_TRIVIAL(warning) << "A warning severity message";
            // BOOST_LOG_TRIVIAL(error) << "An error severity message";
            // BOOST_LOG_TRIVIAL(fatal) << "A fatal severity message";
        }

        // \brief Print the data on the terminal and write to file
        // \param data value to be logged
        void Log(string);
    };
#endif
} // namespace Properties