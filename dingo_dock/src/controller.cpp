
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

std_msgs::Bool dock_msg, lift_msg;

void controllerCallback(sensor_msgs::Joy msg)
{
	if (msg.buttons[0]){ // X
		ROS_INFO("DOCKING"); 
		dock_msg.data = true;
	}
	if (msg.buttons[1]){ // O
		ROS_INFO("DOCKING STOPPING");
		dock_msg.data = false;
	}
	if (msg.axes[7] > 0.5){ // Up
		ROS_INFO("MANUAL UP");
		lift_msg.data = true; 
	}
	if (msg.axes[7] < -0.5){ // Down
		ROS_INFO("MANUAL DOWN");
		lift_msg.data = false; 
	}
}

void distanceCallback(std_msgs::Float32 distance){
	if (distance.data < 0.01 && dock_msg.data){
		dock_msg.data = false;
		ROS_INFO("DOCKING COMPLETE");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;

	ROS_INFO("Controller starting");

	ros::Subscriber sub = n.subscribe("bluetooth_teleop/joy", 1, controllerCallback);
	ros::Subscriber distance_sub = n.subscribe("/drivetest/center_distance", 1, distanceCallback);
	ros::Publisher dock_pub = n.advertise<std_msgs::Bool>("cmd_dock", 1);
	ros::Publisher lift_pub = n.advertise<std_msgs::Bool>("cmd_lift", 1);

	ros::Rate rate(20.0);

	while (ros::ok()){
		ros::spinOnce();

		dock_pub.publish(dock_msg);
		lift_pub.publish(lift_msg);

		rate.sleep();
	}

	return 0;
}
