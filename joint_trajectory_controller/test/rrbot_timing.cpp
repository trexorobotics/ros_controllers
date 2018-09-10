///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018, Trexo Robotics Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Trexo Robotics Inc. nor the names of its
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

/// \author Matt Reynolds

// ROS
#include <ros/ros.h>
#include <std_msgs/Time.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

class RRbotTiming : public hardware_interface::RobotHW
{
public:
  RRbotTiming()
  {
    // Intialize raw data
    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    cmd_[0] = 0.0; cmd_[1] = 0.0;

    // Connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_1("joint1", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("joint2", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface_);

    // Connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_1(jnt_state_interface_.getHandle("joint1"), &cmd_[0]);
    jnt_pos_interface_.registerHandle(pos_handle_1);

    hardware_interface::JointHandle pos_handle_2(jnt_state_interface_.getHandle("joint2"), &cmd_[1]);
    jnt_pos_interface_.registerHandle(pos_handle_2);

    registerInterface(&jnt_pos_interface_);

    // Trajectory time subscriber (When to expect the trajectory)
    timing_sub_ = ros::NodeHandle().subscribe("traj_time", 1, &RRbotTiming::timingCB, this);
    traj_time_.initRT(ros::Time(0));
  }


  ros::Time getTrajTime() {return *traj_time_.readFromRT();}

  void read() {}

  void write()
  {
    pos_[0] = cmd_[0];
    pos_[1] = cmd_[1];
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];

  realtime_tools::RealtimeBuffer<ros::Time> traj_time_;
  void timingCB(const std_msgs::Time& time)
  {
    ROS_INFO_STREAM("Traj will start at: " << (int)time.data.toSec() << "." << time.data.toNSec() % 1000000000);
    traj_time_.writeFromNonRT(time.data);
  }

  ros::Subscriber timing_sub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrbot_timing");
  ros::NodeHandle nh;

  RRbotTiming robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(10);
  ros::Time now = ros::Time::now();
  ros::Time last_time;

  uint loop_state = 0; // 0=init, 1=countdown, 2=long period, 3=short period, 4=done
  int num_cycles_to_traj = -1;
  ros::Duration long_period(0.16);  // Ideal period length immediately before traj start: 0.16sec
  ros::Duration short_period(0.04); // Ideal period length during traj start: 0.04sec

  while (ros::ok())
  {
    last_time = now;
    now = ros::Time::now();
    ros::Duration update_period = now - last_time;

    ROS_INFO_STREAM("Calling update w/ period: " << update_period.toSec());
    robot.read();
    cm.update(now, update_period);
    robot.write();

    // Update at a steady 10hz until we are told when the trajectory will be started. Once we know this,
    // we can start to manipulate the timing such that we end up starting the trajectory during a short period.
    if (robot.getTrajTime() != ros::Time(0) && loop_state == 0)
    {
      // We want the traj to start partway through the short period, perhaps 50% through. This means we want
      // our countdown to end up being finished 0.16 + 0.02sec before the traj starts so that we can do one
      // long period before the short period.

      ros::Duration time_to_sequence = robot.getTrajTime() - ros::Time::now() - long_period - short_period * 0.5;
      num_cycles_to_traj = time_to_sequence.toSec() / 0.1; // How many ~10hz cycles
      rate = ros::Rate(num_cycles_to_traj / time_to_sequence.toSec());
      loop_state = 1;
    }

    if (loop_state == 1) // Countdown
    {
      if (num_cycles_to_traj == 0)
        loop_state = 2;
      else
        num_cycles_to_traj--;
    }
    
    if (loop_state == 2) // Long period
    {
      loop_state = 3;
      long_period.sleep();
    }
    else if (loop_state == 3) // Short period
    {
      loop_state = 4;
      short_period.sleep(); // Ideally, the trajectory will start during this sleep
      rate = ros::Rate(10);
    }
    else // Regular period
    {
      rate.sleep();
    }
  }
  spinner.stop();

  return 0;
}
