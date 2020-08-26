///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020, RT Cooparation
//
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#pragma once


#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include <ros/duration.h>

#include <hardware_interface/internal/resource_manager.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_interface_exception.h>

//#define DEBUG 1
namespace joint_limits_interface
{

/**
 * \brief A handle used to enforce position and velocity limits of a position-controlled joint.
 *
 * This class implements a very simple position and velocity limits enforcing policy, and tries to impose the least
 * amount of requisites on the underlying hardware platform.
 * This lowers considerably the entry barrier to use it, but also implies some limitations.
 *
 * <b>Requisites</b>
 * - Position (for non-continuous joints) and velocity limits specification.
 * - Soft limits specification. The \c k_velocity parameter is \e not used.
 *
 * <b>Open loop nature</b>
 *
 * Joint position and velocity limits are enforced in an open-loop fashion, that is, the command is checked for
 * validity without relying on the actual position/velocity values.
 *
 * - Actual position values are \e not used because in some platforms there might be a substantial lag
 *   between sending a command and executing it (propagate command to hardware, reach control objective,
 *   read from hardware).
 *
 */

// TODO: Leverage %Reflexxes Type II library for acceleration limits handling?
class PositionJointSoftLimitsHandleEx
{
public:
  PositionJointSoftLimitsHandleEx() {}

  PositionJointSoftLimitsHandleEx(const hardware_interface::JointHandle& jh,
                                  ros::NodeHandle& nh,
                                  const std::string param)
    : jh_(jh),
      scaling_(1.0)
  {
    bool has_max;
    double max_val;
    if (!nh.getParam(param+"has_velocity_limits", has_max)) {
      ROS_WARN("Failed to get limit parameter.");
    }else{
      limits_.has_velocity_limits = has_max;
    }

    if (has_max){
      if (!nh.getParam(param+"max_velocity", max_val)) {
        ROS_WARN("Failed to get limit parameter.");
      }else{
        limits_.max_velocity = max_val;
      }
    }

    if (!nh.getParam(param+"has_acceleration_limits", has_max)) {
      ROS_WARN("Failed to get limit parameter.");
    }else{
      limits_.has_acceleration_limits = has_max;
    }

    if (has_max){
      if (!nh.getParam(param+"max_acceleration", max_val)) {
        ROS_WARN("Failed to get limit parameter.");
      }else{
        limits_.max_acceleration = max_val;
      }
    }

    if (!limits_.has_velocity_limits)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                           "'. It has no velocity limits specification.");
    }
  }

  PositionJointSoftLimitsHandleEx(const hardware_interface::JointHandle& jh,
                                  const JointLimits&                     limits,
                                  const SoftJointLimits&                 soft_limits)
    : jh_(jh),
      limits_(limits),
      scaling_(1.0),
      soft_limits_(soft_limits)
  {
    if (!limits.has_velocity_limits)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                           "'. It has no velocity limits specification.");
    }
  }

  /** \return Joint name. */
  std::string getName() const {return jh_.getName();}

  /** \return Joint name. */
  void setScalingFactor(double val) 
  { 
    ROS_INFO("Set ScalingFactor of %s to %lf.", jh_.getName().c_str(), val);
    scaling_ = val;
    return;
  }


  /**
   * \brief Enforce position and velocity limits for a joint subject to soft limits.
   *
   * If the joint has no position limits (eg. a continuous joint), only velocity limits will be enforced.
   * \param period Control period.
   */
  void enforceLimits(const ros::Duration& period)
  {
    const double pos_cmd = getEnforceLimitValue(period);

    jh_.setCommand(pos_cmd);

    // Cache variables
    prev_cmd_ = jh_.getCommand();
  }

  ///
  const double getEnforceLimitValue(const ros::Duration& period)
  {
    assert(period.toSec() > 0.0);

    using internal::saturate;

    // Current position
    // TODO: Doc!
    if (std::isnan(prev_cmd_)) {prev_cmd_ = jh_.getPosition();} 
    const double pos = prev_cmd_;

    // Velocity bounds
    double soft_min_vel;
    double soft_max_vel;

    if (limits_.has_position_limits)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit
      soft_min_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.min_position),
                              -limits_.max_velocity, limits_.max_velocity);

      soft_max_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.max_position),
                              -limits_.max_velocity, limits_.max_velocity);
    }
    else
    {
      // No position limits, eg. continuous joints
      soft_min_vel = -limits_.max_velocity;
      soft_max_vel =  limits_.max_velocity;
    }

    // Position bounds
    const double dt = period.toSec();
    double pos_low  = pos + soft_min_vel * dt * scaling_;
    double pos_high = pos + soft_max_vel * dt * scaling_;

    if (limits_.has_position_limits)
    {
      // This extra measure safeguards against pathological cases, like when the soft limit lies beyond the hard limit
      pos_low  = std::max(pos_low,  limits_.min_position);
      pos_high = std::min(pos_high, limits_.max_position);
    }

 //   std::cerr << "--> " << jh_.getName() << ":" << jh_.getCommand() << ":" << pos_low << "," << pos_high << std::endl;

    // Saturate position command according to bounds
    const double pos_org = jh_.getCommand();
    const double pos_cmd = saturate(jh_.getCommand(), pos_low, pos_high);
#if DEBUG
    if (pos_org != pos_cmd){
       std::cerr << "-->   " << jh_.getName() << ":" << (pos_org - pos_cmd) << std::endl;
    }
#endif
    return pos_cmd;
  }

  /**
   * \brief Reset state, in case of mode switch or e-stop
   */
  void reset(){
    prev_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

public:
  double scaling_;

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
  SoftJointLimits soft_limits_;

  double prev_cmd_ = {std::numeric_limits<double>::quiet_NaN()};
};

/**
 * \brief Interface for enforcing joint limits.
 *
 * \tparam HandleType %Handle type. Must implement the following methods:
 *  \code
 *   void enforceLimits();
 *   std::string getName() const;
 *  \endcode
 */
/** Interface for enforcing limits on a position-controlled joint with soft position limits. */
class PositionJointSoftLimitsInterfaceEx : public JointLimitsInterface<PositionJointSoftLimitsHandleEx> {
public:
  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Reset all managed handles. */
  void reset()
  {
    for (auto&& resource_name_and_handle : this->resource_map_)
    {
      resource_name_and_handle.second.reset();
    }
  }
  /*\}*/

  void setScalingFactor(double val) 
  {
    for (auto&& resource_name_and_handle : this->resource_map_)
    {
      resource_name_and_handle.second.setScalingFactor(val);
    }
  }

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Enforce limits for all managed handles. */
  void enforceLimits(const ros::Duration& period)
  {
    //ROS_WARN("Call enforceLimits 1.");
    for (auto&& resource_name_and_handle : this->resource_map_)
    {
      resource_name_and_handle.second.enforceLimits(period);
    }
  }
  /*\}*/
};

}
