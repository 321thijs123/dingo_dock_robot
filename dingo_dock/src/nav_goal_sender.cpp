#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Bool.h"

#define M_PI 3.14159265359

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool docking = false;

void cmd_dockCallback(std_msgs::Bool cmd_dock_msg){
	docking = cmd_dock_msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_goal_sender");

  ros::NodeHandle node;

  ros::Subscriber cmd_dock_sub = node.subscribe("cmd_dock", 1, cmd_dockCallback);
  ros::Publisher cmd_precision_dock_pub = node.advertise<std_msgs::Bool>("cmd_precision_dock", 1);

  tf::TransformListener listener;

  bool found = false;

  tf::Vector3 v_legs, v_robot;
  tf::Quaternion q_legs;

  ros::Rate rate(1.0);

  bool precision_docking = false;

  while (ros::ok())
  {
	if (docking && !precision_docking){
		tf::StampedTransform transform_legs, transform_robot;

		std_msgs::Bool precision_dock_msg;

		precision_dock_msg.data = false;

		cmd_precision_dock_pub.publish(precision_dock_msg);

		try
		{

			listener.lookupTransform("odom", "leg_pair_0", ros::Time(0), transform_legs);
			listener.lookupTransform("odom", "base_link", ros::Time(0), transform_robot);

			v_legs = transform_legs.getOrigin();
			q_legs = transform_legs.getRotation();

			v_robot = transform_robot.getOrigin();

			ROS_INFO("Translation platform: %.3f, %.3f, %.3f", v_legs.getX(), v_legs.getY(), v_legs.getZ());

			//tell the action client that we want to spin a thread by default
			MoveBaseClient ac("move_base", true);

			//wait for the action server to come up
			while (!ac.waitForServer(ros::Duration(5.0)))
			{
				 ROS_INFO("Waiting for the move_base action server to come up");
			}

			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "odom";
			goal.target_pose.header.stamp = ros::Time::now();

 			double roll, pitch, yaw;

 			tf::Matrix3x3(q_legs).getRPY(roll, pitch, yaw);

			double pos1_x = v_legs.getX() + cos(yaw) * 1.0;
			double pos1_y = v_legs.getY() + sin(yaw) * 1.0;
			double pos2_x = v_legs.getX() - cos(yaw) * 1.0;
			double pos2_y = v_legs.getY() - sin(yaw) * 1.0;

			if (pow(pos1_x - v_robot.getX(), 2) + pow(pos1_y - v_robot.getY(), 2) < pow(pos2_x - v_robot.getX(), 2) + pow(pos2_y - v_robot.getY(), 2)) {

 				goal.target_pose.pose.position.x = pos1_x;
 				goal.target_pose.pose.position.y = pos1_y;

				tf::Quaternion q_target;

				q_target.setRPY(0,0,yaw+M_PI);

				goal.target_pose.pose.orientation.x = q_target[0];
				goal.target_pose.pose.orientation.y = q_target[1];
				goal.target_pose.pose.orientation.z = q_target[2];
				goal.target_pose.pose.orientation.w = q_target[3];
			}
			else {
				goal.target_pose.pose.position.x = pos2_x;
				goal.target_pose.pose.position.y = pos2_y;

 				goal.target_pose.pose.orientation.x = q_legs[0];
 				goal.target_pose.pose.orientation.y = q_legs[1];
 				goal.target_pose.pose.orientation.z = q_legs[2];
 				goal.target_pose.pose.orientation.w = q_legs[3];
			}

			goal.target_pose.pose.position.z = 0;

			ROS_INFO("Sending goal");
			ac.sendGoal(goal);

 			ac.waitForResult();

			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				ROS_INFO("Hooray, the robot moved to the goal");
				precision_docking = true;
			}
			else {
				ROS_INFO("The base failed to move to the goal for some reason");
			}
		}

		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
      			ros::Duration(1.0).sleep();
			continue;
		}

	}

	if (!docking) {
		precision_docking = false;
	}

	std_msgs::Bool precision_dock_msg;

	precision_dock_msg.data = precision_docking;

	cmd_precision_dock_pub.publish(precision_dock_msg);

	ros::spinOnce();
	rate.sleep();
  }

  return 0;
}
