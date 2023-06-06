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

#ifndef UAM_SCAN_PARAMS_H
#define UAM_SCAN_PARAMS_H

#include <uam/protocol_types/scip_protocol_types.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace uam
{
/**
 * @brief Scan parameters structure
 */
class ScanParameters
{
public:
  /**
   * @brief Create Scan Params from PP reply
   *
   * @param[in] reply - Reply to the PP command
   * @return[out] ScanParameters
   */
  static ScanParameters fromMessage(const scip_protocol::PPReply& reply)
  {
    ScanParameters scan_details;
    // Copy some of the raw params and constant params
    scan_details.first_step_limit = reply.first_step;
    scan_details.last_step_limit = reply.last_step;
    scan_details.front_step = reply.front_data_step;
    scan_details.rpm = reply.rpm;
    scan_details.angular_resolution = reply.angular_resolution;
    scan_details.range_min = reply.min_distance / 1000.0;
    scan_details.range_max = reply.max_distance / 1000.0;

    scan_details.angle_min_limit =
      2.0 * M_PI * scan_details.step2index(scan_details.first_step_limit) / scan_details.angular_resolution;
    scan_details.angle_max_limit =
      2.0 * M_PI * scan_details.step2index(scan_details.last_step_limit) / scan_details.angular_resolution;
    scan_details.scan_period = 60.0 / static_cast<double>(scan_details.rpm);
    scan_details.angle_increment = 2 * M_PI / scan_details.angular_resolution;
    scan_details.time_increment = scan_details.scan_period / scan_details.angular_resolution;

    // The remaining params should vary according to angles
    scan_details.setAngleLimits(scan_details.angle_min_limit, scan_details.angle_max_limit);

    return scan_details;
  }

  /**
   * @brief Set new angle limits
   *
   * @param[in] min_angle - New min/start angle
   * @param[in] max_angle - New max/last angle
   */
  inline void setAngleLimits(const double new_angle_min, const double new_angle_max)
  {
    // Set step limits
    int tmp_first_step = angle2step(new_angle_min);
    int tmp_last_step = angle2step(new_angle_max);

    if (tmp_first_step == tmp_last_step)
    {
      if (tmp_first_step == first_step_limit)  // At beginning of range
      {
        tmp_last_step += 1;
      }
      else  // At end of range (or all other cases)
      {
        tmp_first_step -= 1;
      }
    }

    if (tmp_last_step < tmp_first_step)
      std::swap(tmp_last_step, tmp_first_step);

    first_step = tmp_first_step;
    last_step = tmp_last_step;

    // Update new angle min and angle max
    angle_min = step2angle(first_step);
    angle_max = step2angle(last_step);

    // Update angular time offset
    angular_time_offset = calculateTimeOffset();
  }

  /**
   * @brief Get min/fisrt limit angle
   * @return angle_min_limit
   */
  inline auto getAngleMinLimit() const { return angle_min_limit; }

  /**
   * @brief Get max/last limit angle
   * @return angle_max_limit
   */
  inline auto getAngleMaxLimit() const { return angle_max_limit; }

  /**
   * @brief Get min/first angle
   * @return angle_min
   */
  inline auto getAngleMin() const { return angle_min; }

  /**
   * @brief Get max/last angle
   * @return angle_max
   */
  inline auto getAngleMax() const { return angle_max; }

  /**
   * @brief Get angle increment
   * @return angle_increment
   */
  inline auto getAngleIncrement() const { return angle_increment; }

  /**
   * @brief Get Scan period
   * @return scan_period
   */
  inline auto getScanPeriod() const { return scan_period; }

  /**
   * @brief Get time increment
   * @return time_increment
   */
  inline auto getTimeIncrement() const { return time_increment; }

  /**
   * @brief Get min range
   * @return range_min
   */
  inline auto getRangeMin() const { return range_min; }

  /**
   * @brief Get max range
   * @return range_max
   */
  inline auto getRangeMax() const { return range_max; }

  /**
   * @brief Get Angular time offset
   * @return angular_time_offset
   */
  inline auto getAngularTimeOffset() const { return angular_time_offset; }

  /**
   * @brief Get first step (step associated with min_angle)
   * @return first_step
   */
  inline auto getFirstStep() const { return first_step; }

  /**
   * @brief Get last step (step associated with max_angle)
   * @return last_step
   */
  inline auto getLastStep() const { return last_step; }

private:
  /**
   * @brief Calculate time offset
   *
   * Hokuyo's timestamps start from the rear center of the device (at Pi according to ROS standards)
   * so correct it according to the configured angles
   *
   * (Function adapted from urg_node)
   *
   * @return time offset in seconds
   */
  inline double calculateTimeOffset() const
  {
    double circle_fraction = 0.0;
    if (first_step == 0 && last_step == 0)
    {
      circle_fraction = (angle_min_limit + M_PI) / (2.0 * M_PI);
    }
    else
    {
      circle_fraction = (getAngleMin() + M_PI) / (2.0 * M_PI);
    }
    return circle_fraction * getScanPeriod();
  }

  /**
   * @brief Return a step between [first_step_limit, last_step_limit]
   *
   * @param[in] angle - Input angle
   *
   * @return The closest step to the angle
   */
  int angle2step(const double angle) const
  {
    double tmp = angular_resolution * angle / (2.0 * M_PI);
    int index = 0;
    if (tmp < 0)
      index = static_cast<int>(std::floor(tmp));
    else
      index = static_cast<int>(std::ceil(tmp));
    int step = index + front_step;
    return std::min(std::max(first_step_limit, step), last_step_limit);
  }

  /**
   * @brief Get the angle from the input step
   *
   * @param[in] step - Input step
   * @return Corresponding angle in radians
   */
  double step2angle(const int step) const
  {
    return 2.0 * M_PI * static_cast<double>(step2index(step)) / static_cast<double>(angular_resolution);
  }

  /**
   * @brief Transform step to index (signed)
   * @param[in] step - Input step
   * @return Corresponding signed index
   */
  int step2index(const int step) const { return step - front_step; }

  /**
   * @brief Angular time offset in seconds
   */
  double angular_time_offset { 0. };

  /**
   * @brief Angular distance between measurements in radians
   */
  double angle_increment { 0. };

  /**
   * @brief Time between measurements in seconds
   */
  double time_increment { 0. };

  /**
   * @brief Minimum range value in meters
   */
  double range_min { 0. };

  /**
   * @brief Maximum range value in meters
   */
  double range_max { 0. };

  /**
   * @brief Scan period
   */
  double scan_period { 0. };

  /**
   * @brief Min/first angle in radians
   */
  double angle_min { 0. };

  /**
   * @brief Max/last angle in radians
   */
  double angle_max { 0. };

  /**
   * @brief The step corresponding to the angle_min
   */
  int first_step { 0 };

  /**
   * @brief The step corresponding to the angle_max
   */
  int last_step { 0 };

  /**
   * @brief Min angle limit in radians
   */
  double angle_min_limit { 0. };

  /**
   * @brief Max angle limit in radians
   */
  double angle_max_limit { 0. };

  /**
   * @brief The step corresponding to the angle_min_limit
   */
  int first_step_limit { 0 };

  /**
   * @brief The step corresponding to the angle_max_limit
   */
  int last_step_limit { 0 };

  /**
   * @brief Front measurement step (the step which corresponds to the yaw = 0 (ROS Convention))
   */
  int front_step { 0 };

  /**
   * @brief The angular resolution (Number of divisions in 360°)
   */
  int angular_resolution { 0 };

  /**
   * @brief The sensor rotations per minute
   */
  int rpm { 0 };
};

}  // namespace uam

#endif  // UAM_SCAN_PARAMS_H
