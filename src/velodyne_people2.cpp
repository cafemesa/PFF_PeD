#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#define PI 3.14159265


class pff_sem
{
public:
	pff_sem();

	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
	void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL);	

	void filter(const sensor_msgs::PointCloud2ConstPtr &input);
	pcl::PointCloud<pcl::PointXYZ> cloudfilter(pcl::PointCloud<pcl::PointXYZ> LegsCloud, float separation, float minWidth, float maxWidth);
	pcl::PointCloud<pcl::PointXYZ> cloudmatcher(pcl::PointCloud<pcl::PointXYZ> PeopleCloud, pcl::PointCloud<pcl::PointXYZ> LegsCloud, pcl::PointCloud<pcl::PointXYZ> TrunkCloud, float separation);
	pcl::PointCloud<pcl::PointXYZ> mapfilter(pcl::PointCloud<pcl::PointXYZ> PeopleCloud);

	float robotheight, sensorheight, resolution, legs_begin, legs_end, trunk_begin, trunk_end, pose_x, pose_y, initial_angle;
	int visionangle, position, Ncloud;
	std::string topic_pub1, topic_pub2, topic_pub3, sensor_topic_sub, pose_topic_sub, map_topic_sub, frame_id;
	
	int** maparray;
	int  map_width, map_height;

private:
	ros::NodeHandle node_, ns_;
	ros::Publisher people_pub, legs_pub, trunk_pub;
	ros::Subscriber SensorSub, PoseSub, MapSub;
};

pff_sem::pff_sem() : node_("~"),
					 robotheight(1.0),
					 sensorheight(0.57),
					 visionangle(360),
					 resolution(0.2),
					 legs_begin(0.25),
					 legs_end(0.60),
					 trunk_begin(0.80),
					 trunk_end(1.50),
					 topic_pub1("/People_PC2"),
					 topic_pub2("/Legs_PC2"),
					 topic_pub3("/Trunk_PC2"),
					 sensor_topic_sub("/livox/lidar"),
					 pose_topic_sub("/amcl_pose"),
					 map_topic_sub("/map"),
					 frame_id("map")
{
	/** Get parameters from the launch file */
	node_.param("robot_height", robotheight, robotheight);
	node_.param("sensor_height", sensorheight, sensorheight);
	node_.param("horizontal_fov", visionangle, visionangle);
	node_.param("resolution", resolution, resolution);
	node_.param("legs_begin", legs_begin, legs_begin);
	node_.param("legs_end", legs_end, legs_end);
	node_.param("trunk_begin", trunk_begin, trunk_begin);
	node_.param("trunk_end", trunk_end, trunk_end);

	node_.param("topic_pub_people", topic_pub1, topic_pub1);
	node_.param("topic_pub_legs", topic_pub2, topic_pub2);
	node_.param("topic_pub_trunk", topic_pub3, topic_pub3);
	node_.param("sensor_topic_sub", sensor_topic_sub, sensor_topic_sub);
	node_.param("pose_topic_sub", pose_topic_sub, pose_topic_sub);
	node_.param("map_topic_sub", map_topic_sub, map_topic_sub);
	
	node_.param("frame_id", frame_id, frame_id);

	/** Define Subscriber */
	SensorSub = ns_.subscribe(sensor_topic_sub, 50, &pff_sem::filter, this);
	PoseSub = ns_.subscribe(pose_topic_sub, 50, &pff_sem::poseAMCLCallback, this);
	MapSub = ns_.subscribe(map_topic_sub, 50, &pff_sem::mapCallback, this);

	/** Define Publisher */
	people_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub1, 1, false);
	legs_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub2, 1, false);
	trunk_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub3, 1, false);

	Ncloud = 0;
	pose_x = 0;
	pose_y = 0;
	initial_angle = 0;
}

void pff_sem::filter(const sensor_msgs::PointCloud2ConstPtr &input)
{
	Ncloud++;

	pcl::PointCloud<pcl::PointXYZ> laserCloudIn, LegsCloud, TrunkCloud, PeopleCloud;
	pcl::PointXYZ pointlegs1, pointlegs2, pointtrunk1, pointtrunk2;
	pcl::fromROSMsg(*input, laserCloudIn);

	unsigned int num_readings = 360 / resolution;
	LegsCloud.points.resize(num_readings);
	TrunkCloud.points.resize(num_readings);
	PeopleCloud.points.resize(num_readings);

	float addpos = 0;
	double LegsRanges[num_readings] = {0}, TrunkRanges[num_readings] = {0};
	int position = 0, postrunk10 = 0, postrunk30 = 0, poslegs10 = 0, poslegs30 = 0;

	for (int i = 0; i < laserCloudIn.size(); i++)
	{
		float hip = sqrt(pow(laserCloudIn[i].x, 2) + pow(laserCloudIn[i].y, 2));
		float hangle = (asin((laserCloudIn[i].x) / hip)) * 180 / PI;

		/** Discard points */
		if ((laserCloudIn[i].z < -sensorheight + 0.10) || isnan(laserCloudIn[i].x)==1 || isnan(laserCloudIn[i].y)==1 ||
			fabs(laserCloudIn[i].x)<0.01 || fabs(laserCloudIn[i].y)<0.01 ||
			(visionangle <= 180 && laserCloudIn[i].x <= 0) ||
			(visionangle < 180 && laserCloudIn[i].x > 0 && hangle < 90 - (visionangle / 2)) ||
			(visionangle > 180 && visionangle < 360 && laserCloudIn[i].x < 0 && abs(hangle) > (visionangle - 180) / 2))
		{
			continue;
		}

		// Get the position in the arrays for get the nearest points
		if (laserCloudIn[i].y > 0)
		{
			position = (180.0 + hangle) / resolution;
		}
		else if (laserCloudIn[i].x > 0 && laserCloudIn[i].y <= 0)
		{
			position = (360.0 - hangle) / resolution;
		}
		else
		{
			position = -hangle / resolution;
		}
		
		// Generate  Legs Point Cloud
		if (-sensorheight + legs_begin < laserCloudIn[i].z && laserCloudIn[i].z < -sensorheight + legs_end)
		{
			// Extraction of nearest points
			if (LegsRanges[position] == 0 || hip < LegsRanges[position])
			{
				
				LegsCloud.points[position].x = laserCloudIn[i].x;
				LegsCloud.points[position].y = laserCloudIn[i].y;
				LegsCloud.points[position].z = 0; // Projection onto 2D
				LegsRanges[position] = hip;
			}
		}

		// Generate  Trunk Point Cloud
		if (-sensorheight + trunk_begin < laserCloudIn[i].z && laserCloudIn[i].z < -sensorheight + trunk_end)
		{
			// Extraction of nearest points
			if (TrunkRanges[position] == 0 || hip < TrunkRanges[position])
			{
				TrunkCloud.points[position].x = laserCloudIn[i].x;
				TrunkCloud.points[position].y = laserCloudIn[i].y;
				TrunkCloud.points[position].z = 0; // Projection onto 2D
				TrunkRanges[position] = hip;
			}
		}
	}


	LegsCloud = cloudfilter(LegsCloud, 0.25, 0.05, 0.65);
	TrunkCloud = cloudfilter(TrunkCloud, 0.25, 0.15, 0.65);
	PeopleCloud = cloudmatcher(PeopleCloud, LegsCloud, TrunkCloud, 0.35);
	PeopleCloud = mapfilter(PeopleCloud);

	//Publish LegsCloud
	sensor_msgs::PointCloud2 LegsCloud_output;
	pcl::toROSMsg(LegsCloud, LegsCloud_output);
	LegsCloud_output.header.frame_id = frame_id;
	legs_pub.publish(LegsCloud_output);

	//Publish TrunkCloud
	sensor_msgs::PointCloud2 TrunkCloud_output;
	pcl::toROSMsg(TrunkCloud, TrunkCloud_output);
	TrunkCloud_output.header.frame_id = frame_id;
	trunk_pub.publish(TrunkCloud_output);

	//Publish PeopleCloud
	sensor_msgs::PointCloud2 PeopleCloud_output;
	pcl::toROSMsg(PeopleCloud, PeopleCloud_output);
	PeopleCloud_output.header.frame_id = frame_id;
	people_pub.publish(PeopleCloud_output);
}

/** Legs and Trunk filters */
pcl::PointCloud<pcl::PointXYZ> pff_sem::cloudfilter(pcl::PointCloud<pcl::PointXYZ> Cloud, float separation, float minWidth, float maxWidth)
{
	pcl::PointXYZ point1, point2;
	int point1pos, point2pos, pos1 = 0, pos2 = 0;

	for (int i = 0; i < Cloud.size(); i++)
	{
		if (fabs(Cloud.points[i].x) < 0.1 && fabs(Cloud.points[i].y) < 0.1)
		{
			continue;
		}
		// Save the first point of the sample
		if (pos2 == 0)
		{
			point1.x = Cloud.points[i].x;
			point1.y = Cloud.points[i].y;
			point1pos = i;
			point2.x = Cloud.points[i].x;
			point2.y = Cloud.points[i].y;
			point2pos = i;
			pos2++;
		}
		else
		{
			float hiplegs1 = sqrt(pow(Cloud.points[i].x - point1.x, 2.0) + pow(Cloud.points[i].y - point1.y, 2.0));

			// Check if a new cluster appear in the field of view
			if (hiplegs1 > separation)
			{
				float hiplegs2 = sqrt(pow(point1.x - point2.x, 2.0) + pow(point1.y - point2.y, 2.0));
				// Chech if the detecte object have the posible dimension for a leg
				if (minWidth < hiplegs2 && hiplegs2 < maxWidth)
				{
					//Save de average   between the first and the last point in the group
					Cloud.points[pos1].x = (point1.x + point2.x) / 2;
					Cloud.points[pos1].y = (point1.y + point2.y) / 2;
					Cloud.points[pos1].z = 0;
					pos1++;
				}
				// Store the first point of the new detection
				point2.x = Cloud.points[i].x;
				point2.y = Cloud.points[i].y;
				point2pos = i;
			}
			//Store the last point to compare in the nest round
			point1.x = Cloud.points[i].x;
			point1.y = Cloud.points[i].y;
			point1pos = i;
		}
	}
	Cloud.points.resize(pos1);
	return Cloud;
}

/** Cloud matcher */
pcl::PointCloud<pcl::PointXYZ> pff_sem::cloudmatcher(pcl::PointCloud<pcl::PointXYZ> PeopleCloud, pcl::PointCloud<pcl::PointXYZ> LegsCloud, pcl::PointCloud<pcl::PointXYZ> TrunkCloud, float separation)
{
	int pospos = 0;
	for (int i = 1; i < TrunkCloud.size(); i++)
	{
		for (int j = 1; j < LegsCloud.size(); j++)
		{
			// Get the distance between the points in Trunk PointCloud and Legs PointCloud
			float hip1 = sqrt(pow(TrunkCloud.points[i].x - LegsCloud.points[j].x, 2.0) + pow(TrunkCloud.points[i].y - LegsCloud.points[j].y, 2.0));
			// If exist a match between the two clouds the point in Trunk PointCloud us stored in People PointCloud
			if (hip1 < separation)
			{
				// Store the first point
				if (pospos == 0)
				{
					PeopleCloud.points[pospos].x = TrunkCloud.points[i].x;
					PeopleCloud.points[pospos].y = TrunkCloud.points[i].y;
					PeopleCloud.points[pospos].z = 0;
					pospos++;
				}
				else
				{
					float hip20 = 10;
					float hip10 = sqrt(pow(PeopleCloud.points[pospos - 1].x - TrunkCloud.points[i].x, 2.0) + pow(PeopleCloud.points[pospos - 1].y - TrunkCloud.points[i].y, 2.0));
					if (pospos > 1)
					{
						float hip20 = sqrt(pow(PeopleCloud.points[pospos - 2].x - TrunkCloud.points[i].x, 2.0) + pow(PeopleCloud.points[pospos - 2].y - TrunkCloud.points[i].y, 2.0));
					}
					if (hip10 > 0.6 && hip20 > 0.6)
					{
						PeopleCloud.points[pospos].x = TrunkCloud.points[i].x;
						PeopleCloud.points[pospos].y = TrunkCloud.points[i].y;
						PeopleCloud.points[pospos].z = 0;
						pospos++;
					}
				}
			}
		}
	}

	// People PointCloud resize
	PeopleCloud.points.resize(pospos);
	return PeopleCloud;
}

/** Map filter */
pcl::PointCloud<pcl::PointXYZ> pff_sem::mapfilter(pcl::PointCloud<pcl::PointXYZ> PeopleCloud)
{
	int lastpos = 0;
	for (int i = 0; i < PeopleCloud.size(); i++)
	{
		int position = 0;
		float angle_min = 3 * PI / 2;
		float angle_increment = -2 * PI / (360 / resolution);
		float hip = sqrt((PeopleCloud[i].x) * (PeopleCloud[i].x) + ((PeopleCloud[i].y) * (PeopleCloud[i].y)));
		float hangle = (asin((PeopleCloud[i].x) / hip)) * 180 / PI;

		if (PeopleCloud[i].y > 0)
		{
			position = (180.0 + hangle) / resolution;
		}
		else if (PeopleCloud[i].x > 0 && PeopleCloud[i].y <= 0)
		{
			position = (360.0 - hangle) / resolution;
		}
		else
		{
			position = -hangle / resolution;
		}

		float finalangle = (position * angle_increment) + angle_min;

		float laserpointx = hip * cos(-finalangle - initial_angle - PI);
		float laserpointy = hip * sin(-finalangle - initial_angle - PI);

		PeopleCloud[i].x = -laserpointx + pose_x;
		PeopleCloud[i].y = laserpointy + pose_y;

		int posxpx = ((/** - origin X in map.yaml */-PeopleCloud[i].x) / (-0.05));
		int posypx = ((/** - origin Y in map.yaml */-PeopleCloud[i].y) / (-0.05));
		int spacecount = 0;

		//ROS_INFO("1. x=%d, y=%d, mapw=%d maph=%d", posxpx, posypx, PeopleCloud[i].x, PeopleCloud[i].y);
		for (int j = posxpx - 4; j <= posxpx + 5; j++)
		{
			for (int k = posypx - 5; k <= posypx + 5; k++)
			{
				if (maparray[j][k] == 100 || maparray[j][k] == -1)
				{
					spacecount++;
				}
			}
		}
		//ROS_INFO("2. x=%d, y=%d, mapw=%d, maph=%d", posxpx, posypx, map_width, map_height);

		if (spacecount < 15)
		{
			PeopleCloud[lastpos].x = PeopleCloud[i].x;
			PeopleCloud[lastpos].y = PeopleCloud[i].y;
			lastpos++;
		}
	}
	PeopleCloud.resize(lastpos);
	return PeopleCloud;
}

/** Robot Pose Callback */
void pff_sem::poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL)
{
	pose_x = msgAMCL->pose.pose.position.x;
	pose_y = msgAMCL->pose.pose.position.y;
	float siny_cosp = 2 * (msgAMCL->pose.pose.orientation.w * msgAMCL->pose.pose.orientation.z);
	float cosy_cosp = 1 - (2 * (msgAMCL->pose.pose.orientation.z * msgAMCL->pose.pose.orientation.z));
	initial_angle = atan2(siny_cosp, cosy_cosp);
}

/** Map Callback */
void pff_sem::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	std_msgs::Header header = msg->header;
	nav_msgs::MapMetaData info = msg->info;

	maparray = new int*[info.width];
    for(int i = 0; i < info.width; ++i)
    {
        maparray[i] = new int[info.height];
    }
    map_width = info.width;
    map_height = info.height;

	for (int x = 0; x < info.width; x++)
	{
		for (int y = 0; y < info.height; y++)
		{
			maparray[x][y] = msg->data[x + info.width * y];
		}
	}
}

/** Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_people");
	pff_sem filter;
	ros::spin();
	return 0;
}
