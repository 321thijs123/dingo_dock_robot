#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "helpers/dingo_exception.cpp"
#define M_PI 3.14159265358979323846

// Determines the platform direction relative to the robot 
double determineDirection(tf::StampedTransform tf_platform, tf::StampedTransform tf_robot)
{
	// to do: niet allemaal tf's sturen naar de functies.

	tf::Quaternion q_platform = tf_platform.getRotation();

	double roll_platform, pitch_platform, yaw_platform;

	tf::Matrix3x3(q_platform).getRPY(roll_platform, pitch_platform, yaw_platform);
	
	tf::Vector3 v_leg_pair = tf_platform.getOrigin();
	tf::Vector3 v_robot = tf_robot.getOrigin();

	double deltaX = v_leg_pair.getX() - v_robot.getX();
	double deltaY = v_leg_pair.getY() - v_robot.getY();

	//ROS_INFO("%f",atan2(deltaY, deltaX)-yaw_platform);
	return atan2(deltaY, deltaX)-yaw_platform;
}

// Determines the the distance between the robot and the front of the platform
double determineDistance( tf::StampedTransform tf_platform, tf::StampedTransform tf_robot)
{
	tf::Vector3 v_leg_pair = tf_platform.getOrigin();
	tf::Vector3 v_robot = tf_robot.getOrigin();

	double deltaX = v_leg_pair.getX() - v_robot.getX();
	double deltaY = v_leg_pair.getY() - v_robot.getY();

	return sqrt((deltaY * deltaY) + (deltaX * deltaX));
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "drivetest");

	ros::NodeHandle n;

	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 20);

	tf::TransformListener listener;
	
	// Variable to remember if the platform is found
	bool platformFound = false;

	tf::Vector3 v;
	tf::Quaternion q;

	tf::StampedTransform tf_platform;
	tf::StampedTransform tf_robot;
	tf::StampedTransform tf_chassis;
	
	// Looking for platform
	ros::Rate rate(20.0);
	while (!platformFound)
	{
		try
		{

			listener.lookupTransform("odom", "platform_0", ros::Time(0), tf_platform);

			platformFound = true;
		}

		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		rate.sleep();
	}

	// Driving under platform
	while (ros::ok())
	{

		geometry_msgs::Twist msg;
		
		// get robot position
		try
		{
			listener.lookupTransform("odom", "chassis_link", ros::Time(0), tf_chassis);
			listener.lookupTransform("odom", "base_link", ros::Time(0), tf_robot);
		}
		// if there is a transformexception, catch it and display error.
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}
		// try to update the platform position
		try
		{
			listener.lookupTransform("odom", "platform_0", ros::Time(0), tf_platform);

		}
		catch (tf::TransformException &ex){}

		//get quaternion platform and robot rotation
		tf::Quaternion q_platform = tf_platform.getRotation();
		tf::Quaternion q_robot = tf_robot.getRotation();
		
		double roll_platform, pitch_platform, yaw_platform;
		double roll_robot, pitch_robot, yaw_robot;

		//convert quaternions to RPY
		tf::Matrix3x3(q_platform).getRPY(roll_platform, pitch_platform, yaw_platform);
		tf::Matrix3x3(q_robot).getRPY(roll_robot, pitch_robot, yaw_robot);

		double platformDistance = determineDistance(tf_robot, tf_platform);				//Distance from point between wheels to platform
		double platform_direction = determineDirection(tf_platform, tf_robot);			//Direction in which the platform is relative to robot frame
		double platformCenterDistance = determineDistance(tf_chassis, tf_platform);		//Distance from center of robot to platform
		double targetDirection;															//Direction in which we want to go
		
		// Determines targetDirection, the direction in which we want to go
		if (platformDistance > 0.12)
		{
			// If the robot is further away than 12 cm
			// Target direction as a combination of platform yaw and driving to the line in front of the platform
			targetDirection = yaw_platform + platform_direction * 2.0;
		}
		else
		{
			// if the robot is within 12 cm keep straight 
			targetDirection = yaw_platform;
		}
		
		// difference between current yaw and target yaw
		double yawDiff = targetDirection - yaw_robot;

		// The rotation speed of the robot is calculated here (P controller)
		msg.angular.z = (yawDiff) * 5.0;

		//Determine drive speed
		//When more than 50 centimeters away, the speed is determined using distance to the point between the wheels
		if (platformDistance > 0.5)
		{
			// If the robot is close to the target direction
			if (abs(yawDiff) < 1.0/50.0)
			{
				//Robot farther away form platform-> faster & bigger deviation to yaw target -> slower
				msg.linear.x =  platformDistance * 1.0 / (1.0-50.0*abs(yawDiff));
			}
			// If the deviation to the target direction is large
			else {
				//Dont drive forward
				msg.linear.x = 0.0;
			}
		}
		//When closer than 50 centimeters the speed is determined using distance to center of robot
		else
		{
			msg.linear.x = platformCenterDistance * 1.0;
		}

		//Limit linear velocity
		if (msg.linear.x > 0.5) msg.linear.x = 0.5;

		// error states

		// check if robot is under platform
		bool underPlatform;
		if (platformDistance < 0.3) underPlatform = true;
		
		// Error for rotating much under platform
		if (underPlatform && ( abs(yawDiff) > 5 * M_PI / 180)  )	
		{
			throw(DingoException("Dingo tried to turn around under platform", 0, 148)); // error: dingo turning under platform, error number = 0, linenumber = 148
		}	


		//Show debugging info
		system("clear");
		std::cout << "\n----------------\n"
				"platform dir:    " << platform_direction << "\n" <<
				"target dir:      " << targetDirection << "\n" <<
				"cur dir:         " << yaw_robot << "\n" <<
				"distance:        " << platformDistance << "\n" <<
				"center distance: " << platformCenterDistance << "\n" <<
				"angular:         " << msg.angular.z << "\n" <<
				"linear:          " << msg.linear.x << "\n" <<
				"under platform:  " << underPlatform << "\n" <<
				"----------------\n";

		//Publish velocity commands
		cmd_vel_pub.publish(msg);

		ros::spinOnce();

		rate.sleep();
	}
}
