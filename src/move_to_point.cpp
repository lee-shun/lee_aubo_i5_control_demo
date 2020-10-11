/*******************************************************************************
 *
 * @file move_to_point.cpp
 *
 * @author lee-shun
 *
 * @email 2015097272@qq.com
 *
 * @data today
 *
 * @version 1.0
 *
 * @brief control the lee_aubo_i5_arm to a certain point
 *
 * @copyright lee-shun
 *
 *******************************************************************************/
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <vector>

int main(int argc, char **argv) {

  /**
   * init the ros node
   */
  ros::init(argc, argv, "move_to_point");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("lee_aubo_i5");

  /**
   * get the end_effector_link name
   */
  std::string end_effector_link = arm.getEndEffectorLink();
  std::cout << "the end effector link is:" << end_effector_link << std::endl;

  /**
   * set the target pose reference frame
   */
  std::string reference_frame = "base_link";
  arm.setPoseReferenceFrame(reference_frame);
  std::cout << "the end reference_frame is:" << arm.getPoseReferenceFrame()
            << std::endl;
  /**
   * can replan after failing
   */
  arm.allowReplanning(true);

  arm.clearPoseTargets();

  /*设置位置(单位：米)和姿态（单位：弧度）的允许误差*/
  arm.setGoalPositionTolerance(0.001);
  arm.setGoalOrientationTolerance(0.01);

  /*设置允许的最大速度和加速度*/
  arm.setMaxAccelerationScalingFactor(0.2);
  arm.setMaxVelocityScalingFactor(0.2);

  /* 控制机械臂先回到初始化位置*/
  arm.setNamedTarget("zero");
  arm.move();
  sleep(1);

  geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;

  std::vector<geometry_msgs::Pose> waypoints;

  /*将初始位姿加入路点列表*/
  waypoints.push_back(start_pose);

  start_pose.position.z -= 0.2;
  waypoints.push_back(start_pose);

  start_pose.position.x += 0.1;
  waypoints.push_back(start_pose);

  start_pose.position.y += 0.1;
  waypoints.push_back(start_pose);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = 0.0;
  int maxtries = 100;
  int attempts = 0;

  while (fraction < 1.0 && attempts < maxtries) {
    fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold,
                                        trajectory);
    attempts++;

    if (attempts % 10 == 0)
      ROS_INFO("Still trying after %d attempts...", attempts);
  }

  if (true) { // fraction == 1
    ROS_INFO("Path computed successfully. Moving the arm.");

    /* 生成机械臂的运动规划数据*/
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    /* 执行运动*/
    arm.execute(plan);
    sleep(5);
  } else {
    ROS_INFO("Path planning failed with only %0.6f success after %d attempts.",
             fraction, maxtries);
  }

  /* 控制机械臂先回到初始化位置*/
  arm.setNamedTarget("home");
  arm.move();
  sleep(1);

  ros::shutdown();
  return 0;
}
