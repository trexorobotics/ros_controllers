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

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <std_msgs/Time.h>

#include <actionlib/client/simple_action_client.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>

// Floating-point value comparison threshold
const double EPS = 0.01;

using actionlib::SimpleClientGoalState;

class JointTrajectoryControllerTest : public ::testing::Test
{
public:
  JointTrajectoryControllerTest()
    : nh("rrbot_timing_controller"),
      short_timeout(1.0),
      long_timeout(10.0),
      controller_state()
  {
    n_joints = (2);
    joint_names.resize(n_joints);
    joint_names[0] = "joint1";
    joint_names[1] = "joint2";

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n_joints, 0.0);

    // Three-point trajectory
    points.resize(3, point);
    points[0].positions[0] =  M_PI / 4.0;
    points[0].positions[1] =  0.0;
    points[0].time_from_start = ros::Duration(1.0);

    points[1].positions[0] =  0.0;
    points[1].positions[1] = -M_PI / 4.0;
    points[1].time_from_start = ros::Duration(2.0);

    points[2].positions[0] = -M_PI / 4.0;
    points[2].positions[1] =  M_PI / 4.0;
    points[2].time_from_start = ros::Duration(4.0);

    traj.joint_names = joint_names;
    traj.points = points;

    // Action goals
    traj_goal.trajectory = traj;

    // Time publisher (Tells the RobotHW when to expect the trajectory)
    time_pub = ros::NodeHandle().advertise<std_msgs::Time>("traj_time", 1);

    // State subscriber
    state_sub = nh.subscribe<control_msgs::JointTrajectoryControllerState>("state",
                                                                           1,
                                                                           &JointTrajectoryControllerTest::stateCB,
                                                                           this);

    // Query state service client
    query_state_service = nh.serviceClient<control_msgs::QueryTrajectoryState>("query_state");

    // Action client
    const std::string action_server_name = nh.getNamespace() + "/follow_joint_trajectory";
    action_client.reset(new ActionClient(action_server_name));
  }

  ~JointTrajectoryControllerTest()
  {
    state_sub.shutdown(); // This is important, to make sure that the callback is not woken up later in the destructor
  }

protected:
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;
  typedef boost::shared_ptr<ActionClient> ActionClientPtr;
  typedef control_msgs::FollowJointTrajectoryGoal ActionGoal;
  typedef control_msgs::JointTrajectoryControllerStateConstPtr StateConstPtr;

  boost::mutex mutex;
  ros::NodeHandle nh;

  unsigned int n_joints;
  std::vector<std::string> joint_names;
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  trajectory_msgs::JointTrajectory traj;
  ActionGoal                       traj_goal;

  ros::Duration short_timeout;
  ros::Duration long_timeout;

  ros::Publisher     time_pub;
  ros::Subscriber    state_sub;
  ros::ServiceClient query_state_service;
  ActionClientPtr    action_client;

  StateConstPtr controller_state;

  void stateCB(const StateConstPtr& state)
  {
    boost::mutex::scoped_lock lock(mutex);
    controller_state = state;
  }

  StateConstPtr getState()
  {
    boost::mutex::scoped_lock lock(mutex);
    return controller_state;
  }

  bool initState(const ros::Duration& timeout = ros::Duration(5.0))
  {
    bool init_ok = false;
    ros::Time start_time = ros::Time::now();
    while (!init_ok && (ros::Time::now() - start_time) < timeout)
    {
      {
        boost::mutex::scoped_lock lock(mutex);
        init_ok = controller_state && !controller_state->joint_names.empty();
      }
      ros::Duration(0.1).sleep();
    }
    return init_ok;
  }

  static bool waitForState(const ActionClientPtr& action_client,
                           const actionlib::SimpleClientGoalState& state,
                           const ros::Duration& timeout)
  {
    using ros::Time;
    using ros::Duration;

    Time start_time = Time::now();
    while (action_client->getState() != state && ros::ok())
    {
      if (timeout >= Duration(0.0) && (Time::now() - start_time) > timeout) { return false; } // Timed-out
      ros::Duration(0.01).sleep();
    }
    return true;
  }
};

// Trajectory sampling with inconsistent control loop period ///////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, trajSampleInconsistentCtrlLoopPeriod)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Tell the HWIface when to expect the trajectory so that it can manipulate the period
  std_msgs::Time delay_time;
  delay_time.data = ros::Time::now() + ros::Duration(2.0);
  time_pub.publish(delay_time);

  // Sleep until it is time to send the trajectory
  (delay_time.data - ros::Time::now()).sleep();

  // This loop is not required for the issue to occur, but it helps more reliably reproduce the issue
  for(uint i = 0; i < 5; i++)
  {
    traj_goal.trajectory.header.stamp = ros::Time(0);
    action_client->sendGoal(traj_goal);
  }

  // Ensure the trajectory is accepted and completes successfully
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE,  short_timeout));
  EXPECT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, traj_goal.trajectory.points.back().time_from_start * 1.25));

  // Detecting the error is difficult. The action interface doesn't report anything unusal.
  // However, we know that this trajectory should take approx 4 sec when executed regularily, but will be cut short by when the issue occurs
  // Therefore, we can test for the issue by measuring the trajectory's duration
  // *** Actually it turns out that the duration isn't cut short, so this check is useless ***
  ros::Duration traj_duration = ros::Time::now() - traj_goal.trajectory.header.stamp;
  EXPECT_GT(traj_duration, traj_goal.trajectory.points.back().time_from_start);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "joint_trajectory_controller_timing_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int ret = RUN_ALL_TESTS();

  spinner.stop();
  ros::shutdown();
  return ret;
}
