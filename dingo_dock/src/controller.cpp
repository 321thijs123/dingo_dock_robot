
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

std_msgs::Bool dock_msg, lift_msg;

// Listenns for updates on Joy msg and makes corresponding command true
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

// listenes for update on distance topic and if the Dingo is under the platform it sends a lift command
void distanceCallback(std_msgs::Float32 distance){
	if (distance.data < 0.01 && dock_msg.data){
		dock_msg.data = false;
		lift_msg.data = true;
		ROS_INFO("DOCKING COMPLETE");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;

	ROS_INFO("Controller starting");
	
	// subsribers and publishers
	ros::Subscriber sub = n.subscribe("bluetooth_teleop/joy", 1, controllerCallback);
	ros::Subscriber distance_sub = n.subscribe("/drivetest/center_distance", 1, distanceCallback);
	ros::Publisher dock_pub = n.advertise<std_msgs::Bool>("cmd_dock", 1);
	ros::Publisher lift_pub = n.advertise<std_msgs::Bool>("lift/cmd", 1);

	ros::Rate rate(20.0);

	while (ros::ok()){
		ros::spinOnce();
		
		//publishes the dock and lift msgs
		dock_pub.publish(dock_msg);
		lift_pub.publish(lift_msg);

		rate.sleep();
	}

	return 0;
}
