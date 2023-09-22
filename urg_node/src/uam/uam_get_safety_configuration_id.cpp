/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <json_transport/json_transport.hpp>
#include <ros/ros.h>
#include <uam/uam_driver.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/crc.hpp>

#include <string>

boost::program_options::variables_map parseArgs(int argc, char** argv)
{
  namespace po = boost::program_options;

  // Use Boost argparser to read commandline args
  auto options = po::options_description("Allowed arguments");
  // clang-format off
  options.add_options()
      ("help,h",
          "produce help message")
      ("lidar-ip", po::value<std::string>()->required(),
          "The IP Address of the lidar server in the form of XXX.XXX.XXX.XXX")
      ("lidar-port", po::value<unsigned int>()->default_value(10940),
          "The tcp client port number the client/robot is using to communicate with laser")
      ("timeout", po::value<double>()->default_value(10.0),
          "Timeout in seconds to wait for sensor reply.");
  // clang-format on
  po::variables_map args;
  po::store(po::parse_command_line(argc, argv, options), args);

  if (args.count("help"))
  {
    ROS_INFO_STREAM(
      "Get UAM Lidar FW version and safety configuration IDs.\n\n"
      << "usage: " << boost::filesystem::path(argv[0]).filename().string() << " [options]\n\n"
      << options);
    exit(EXIT_FAILURE);
  }

  try
  {
    po::notify(args);
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM(
      e.what() << "usage: " << boost::filesystem::path(argv[0]).filename().string() << " [options]\n\n"
               << options);
    exit(EXIT_FAILURE);
  }

  return args;
}

std::uint32_t getSafetyAreasCRC32(uam::UamDriver& lidar)
{
  ROS_INFO_STREAM("Going to read Laser Safety areas");
  std::string combined_crcs = "";
  for (size_t area_number = 1; area_number <= uam::protocol::c_max_safety_area_index; area_number++)
  {
    for (uint16_t area_type = static_cast<uint16_t>(uam::protocol::EYRAreaType::protection_1);
         area_type < static_cast<uint16_t>(uam::protocol::EYRAreaType::MAX);
         area_type++)
    {
      auto yr_area = lidar.getSafetyArea(
        static_cast<uam::protocol::EYRAreaType>(area_type),
        area_number,
        0,
        uam::protocol::c_nr_ranges);
      if (yr_area.has_value())
      {
        combined_crcs.append(std::to_string(yr_area->footer.crc));
      }
    }
  }

  boost::crc_32_type crc32;
  crc32.process_bytes(combined_crcs.data(), combined_crcs.size());
  return crc32.checksum();
}

template <typename T, size_t Size>
std::string byteArrayToString(const std::array<T, Size>& array)
{
  auto output_str = std::string(array.begin(), array.end());
  output_str.erase(std::remove_if(output_str.begin(), output_str.end(), ::isspace), output_str.end());
  return output_str;
}

std::string getConfigurationId(uam::UamDriver& lidar)
{
  try
  {
    return byteArrayToString(lidar.getConfigurationId().id_2);
  }
  catch (const std::exception& e)
  {
    return std::string("");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_lidar");
  ros::Time::init();
  auto args = parseArgs(argc, argv);

  try
  {
    auto lidar = uam::UamDriver();
    lidar.connect(args["lidar-ip"].as<std::string>(), args["lidar-port"].as<unsigned int>());
    lidar.setCommandReplyTimeout(args["timeout"].as<double>());
    // Wait for a heartbeat message to be received
    auto version_details = lidar.getVersionDetails();
    auto configuration_id = getConfigurationId(lidar);
    auto safey_areas_crc32 = getSafetyAreasCRC32(lidar);

    // Parse the resulting heartbeat message and populate a JSON document with the results
    auto document = json_transport::json_t::object();
    {
      document["model_number"] = byteArrayToString(version_details.version_details.sensor_model);
      document["serial_number"] = byteArrayToString(version_details.version_details.serial_number);
      document["firmware_version"] = byteArrayToString(version_details.version_details.firmware_version);
      document["configuration_id"] = configuration_id;
      document["safety_areas_crc32"] = safey_areas_crc32;
    }
    std::cout << document.dump(4) << std::endl;
    return EXIT_SUCCESS;
  }
  catch (const std::exception& e)
  {
    std::cout << "Failed to listen for connected lidars: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
