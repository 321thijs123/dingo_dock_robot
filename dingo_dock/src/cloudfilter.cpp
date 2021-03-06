
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <vector>

class CloudFilter
{

public:
	//Constructor (Called when the CloudFilter object is created)
	CloudFilter()
	{
		layer_pub = n.advertise<sensor_msgs::PointCloud>("cloudfilter/layer", 100);		//Setup publishers for output point clouds
		direction_pub = n.advertise<sensor_msgs::PointCloud>("cloudfilter/direction", 100);
		object_pub = n.advertise<sensor_msgs::PointCloud>("cloudfilter/object", 100);
		group_pub = n.advertise<sensor_msgs::PointCloud>("cloudfilter/group", 100);

		sub = n.subscribe("velodyne_points", 100, &CloudFilter::cloudCallback, this);	//Subscribe to velodyne pointcloud, call CloudFilter::cloudCallback if a new pointcloud is published
	}

	// Only keep points between min_height and max_height
	std::vector<geometry_msgs::Point32> filterLayers(std::vector<geometry_msgs::Point32> points)
	{
		std::vector<geometry_msgs::Point32> layer_points = {};

		for (int i = 0; i < points.size(); i++)
		{
			if (points[i].z < max_height && points[i].z > min_height)
			{
				layer_points.push_back(points[i]);
			}
		}

		return layer_points;
	}

	// Only keep points in front of the robot
	std::vector<geometry_msgs::Point32> filterDirection(std::vector<geometry_msgs::Point32> points)
	{
		std::vector<geometry_msgs::Point32> direction_points = {};

		for (int i = 0; i < points.size(); i++)
		{

			if (points[i].x > 0 )
			{
				direction_points.push_back(points[i]);
			}
		}

		return direction_points;
	}

	// Only keep objects smaller than [max_size] with [clearance] of clearance from its center
	std::vector<geometry_msgs::Point32> filterObjects(std::vector<geometry_msgs::Point32> points)
	{
		std::vector<geometry_msgs::Point32> object_points = {};

		for (int i = 0; i < points.size(); i++)
		{
			bool keep = true;

			for (int j = 0; j < points.size(); j++)
			{
				float distance =(points[i].x - points[j].x) *
								(points[i].x - points[j].x) +
								(points[i].y - points[j].y) *
								(points[i].y - points[j].y);

				if (distance > max_size && distance < clearance)
				{
					keep = false;
					break;
				}
			}

			if (keep)
				object_points.push_back(points[i]);
		}

		return object_points;
	}

	// Remove groups of less than [min_points] points then calculate group centers
	std::vector<geometry_msgs::Point32> getGroups(std::vector<geometry_msgs::Point32> points)
	{
		std::vector<geometry_msgs::Point32> group_points = {};

		for (int i = 0; i < points.size(); i++)
		{
			int count = 0;
			geometry_msgs::Point32 current_group; 

			for (int j = 0; j < points.size(); j++)
			{
				float distance =(points[i].x - points[j].x) *
								(points[i].x - points[j].x) +
								(points[i].y - points[j].y) *
								(points[i].y - points[j].y);

				if (distance < clearance)
				{
					current_group.x += points[j].x;
					current_group.y += points[j].y;

					count++;
				}
			}

			if (count > min_points)
			{

				current_group.x = current_group.x / count;
				current_group.y = current_group.y / count;

				bool is_new = true;

				for(int k = 0; k < group_points.size(); k++)
				{
					if( current_group == group_points[k] )
					{
						is_new = false;
						break;
					}
				}
				if (is_new)
				{
					group_points.push_back(current_group);
				}
			}
		}
		return group_points;
	}

	// Find leg pairs and publish transforms
	std::vector<geometry_msgs::Point32> getLegPairs(std::vector<geometry_msgs::Point32> points)
	{
		static tf::TransformBroadcaster br;

		int num = 0;

		std::vector<geometry_msgs::Point32> leg_pairs = {};

		for (int i = 0; i < points.size(); i++)
		{
			for (int j = 0; j < points.size(); j++)
			{
				float distance =(points[i].x - points[j].x) *
								(points[i].x - points[j].x) +
								(points[i].y - points[j].y) *
								(points[i].y - points[j].y);


				if (distance > y_min_leg_distance && distance < y_max_leg_distance)
				{
					tf::Transform transform;

					geometry_msgs::Point32 leg_pair;

					leg_pair.x = (points[i].x + points[j].x)/2;
					leg_pair.y = (points[i].y + points[j].y)/2;
					leg_pair.z = (points[i].z + points[j].z)/2;
					leg_pairs.push_back(leg_pair);

					transform.setOrigin(tf::Vector3(leg_pair.x, leg_pair.y, leg_pair.z));

					tf::Quaternion q;
					float yaw = atan2(points[i].x-points[j].x,-points[i].y+points[j].y);

					q.setRPY(0, 0 , yaw);
					transform.setRotation(q);

					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "leg_pair_" + std::to_string(num)));

					num++;

					points.erase(points.begin()+i);
					points.erase(points.begin()+j-1);

					i--;
					break;
				}
			}
		}

		return leg_pairs;
	}

	// Find platforms and publish transforms
	std::vector<geometry_msgs::Point32> getPlatforms(std::vector<geometry_msgs::Point32> leg_pairs)
	{
		static tf::TransformBroadcaster br;

		int num = 0;

		std::vector<geometry_msgs::Point32> platforms = {};

	
		// order the leg_pairs array by distance so leg_pair_0 is the most close to the dingo
		std::sort(leg_pairs.begin(), leg_pairs.end(), [](geometry_msgs::Point32 const & a, geometry_msgs::Point32 const & b){
			return sqrt((a.y * a.y) + (a.x * a.x)) < sqrt((b.y * b.y) + (b.x * b.x)); });
		
		for (int i = 0; i < leg_pairs.size(); i++)
		{
			for (int j = 0; j < leg_pairs.size(); j++)
			{
				float distance =(leg_pairs[i].x - leg_pairs[j].x) *
								(leg_pairs[i].x - leg_pairs[j].x) +
								(leg_pairs[i].y - leg_pairs[j].y) *
								(leg_pairs[i].y - leg_pairs[j].y);
				
				
				if (distance > x_min_leg_distance && distance < x_max_leg_distance)
				{
					tf::Transform transform;
					
					geometry_msgs::Point32 platform;
					
					platform.x = (leg_pairs[i].x + leg_pairs[j].x)/2;
					platform.y = (leg_pairs[i].y + leg_pairs[j].y)/2;
					platform.z = (leg_pairs[i].z + leg_pairs[j].z)/2;
					platforms.push_back(platform);

					transform.setOrigin(tf::Vector3(platform.x, platform.y, platform.z));
					
					tf::Quaternion q;
					float yaw = atan2(-leg_pairs[i].y+leg_pairs[j].y ,-leg_pairs[i].x+leg_pairs[j].x);
					q.setRPY(0, 0 , yaw);
					transform.setRotation(q);

					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "platform_" + std::to_string(num)));

					num++;

					leg_pairs.erase(leg_pairs.begin()+i);
					leg_pairs.erase(leg_pairs.begin()+j-1);

					i--;
					break;
				}
			}
		}
		
		return platforms;
	}

	void cloudCallback(const sensor_msgs::PointCloud2 &msg)
	{
		double startTime = ros::Time::now().toSec();
		sensor_msgs::PointCloud cloud;
		
		//Convert pointcloud2 to pointcloud, easier to work with
		convertPointCloud2ToPointCloud(msg, cloud);

		sensor_msgs::PointCloud layer_cloud;
		sensor_msgs::PointCloud direction_cloud;
		sensor_msgs::PointCloud object_cloud;
		sensor_msgs::PointCloud group_cloud;

		// Give all clouds same header so time is correct and rviz doesn't complain
		layer_cloud.header = direction_cloud.header = object_cloud.header = group_cloud.header = cloud.header;

		layer_cloud.points = filterLayers(cloud.points);				//Take a slice of the world so we only look at the heights at which we expect the platform
		direction_cloud.points = filterDirection(layer_cloud.points);	//Only look in front of the robot
		object_cloud.points = filterObjects(direction_cloud.points);	//Find groups of points
		group_cloud.points = getGroups(object_cloud.points);			//Throw away small groups and get center of groups (Average X & Y)

		layer_pub.publish(layer_cloud);				//Publish each pointcloud step for debugging purposes
		direction_pub.publish(direction_cloud);
		object_pub.publish(object_cloud);
		group_pub.publish(group_cloud);

		std::vector<geometry_msgs::Point32> leg_pairs = getLegPairs(group_cloud.points);	//Find groups at correct distance to be a pair of legs (Side by side)
		std::vector<geometry_msgs::Point32> platforms = getPlatforms(leg_pairs);			//Find leg_pairs that are at the correct distance to be a platform

		// Output for debugging
		system("clear");
		std::cout <<"------------------\n" << 
					"Input pts: " << cloud.points.size() << "\n" <<
					"Time:      " << ros::Time::now().toSec() - startTime << "\n"
					"Leg Pairs: " << leg_pairs.size() << "\n" <<
					"Platforms: " << platforms.size() << "\n"
					"------------------\n";
	}

private:
	ros::NodeHandle n;
	ros::Publisher layer_pub;
	ros::Publisher direction_pub;
	ros::Publisher object_pub;
	ros::Publisher group_pub;
	ros::Subscriber sub;

	const float max_height = 0.015;			//Height of platform legs relative to lidar
	const float min_height = -0.055;

	const float max_size = 0.06 * 0.06;		//Maximum size of legs, larger objects will be disgarded (Squared to avoid square root for pythagoras)
	const float clearance = 0.1 * 0.1;		//Minimum distance to other objects to be regarded a seperate object (Squared to avoid square root for pythagoras) 

	const int min_points = 5;				//Minimum poins per object

	const float y_min_leg_distance = 0.610*0.610;		//Minimum distance (left to right) between two possible legs to be a leg same platform (Squared to avoid square root for pythagoras)
	const float y_max_leg_distance = 0.640*0.640;		//Maximum distance (left to right) between two possible legs to be a leg pair of the same platform (Squared to avoid square root for pythagoras)

	const float x_min_leg_distance = 0.540*0.540;		//Minimum distance (front to back) between two possible leg pairs to be the same platform (Squared to avoid square root for pythagoras)
	const float x_max_leg_distance = 0.600*0.600;		//Maximum distance (front to back) between two possible leg pairs to be the same platform (Squared to avoid square root for pythagoras)
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_filter");

	CloudFilter filterObject;	//Create cloud filter object

	ros::spin();

	return 0;
}
