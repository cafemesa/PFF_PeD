#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#define PI 3.14159265

/** Set map size */
int posout = 0;
pcl::PointCloud<pcl::PointXYZ> FinalDoorCloud, laserCloudout;

class pff_sem
{
	public:
		pff_sem();
		void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL);
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
		pcl::PointCloud<pcl::PointXYZ> mapfilter(pcl::PointCloud<pcl::PointXYZ> PeopleCloud);

		void filter(const sensor_msgs::PointCloud2ConstPtr &input);
		void peopleCallback(const sensor_msgs::PointCloud2ConstPtr &input);

		float robotheight, sensorheight, resolution, pose_x, pose_y, initial_angle;
		int visionangle, position;

		std::string topic_pub1, sensor_topic_sub, pose_topic_sub, map_topic_sub, people_topic_sub, frame_id;
		pcl::PointCloud<pcl::PointXYZ> PeopleCloudIn;

		int** maparray;
		int  map_width, map_height;

	private:
		ros::NodeHandle node_, ns_;
		ros::Publisher doors_pub;
		ros::Subscriber SensorSub, PoseSub, MapSub, PeopleSub;
};

pff_sem::pff_sem() : node_("~"),
					 robotheight(1.0),
					 sensorheight(0.57),
					 visionangle(360),
					 resolution(0.2),
					 topic_pub1("/Doors_PC"),
					 sensor_topic_sub("/velodyne_points"),
					 pose_topic_sub("/amcl_pose"),
					 map_topic_sub("/map"),
					 people_topic_sub("/People_PC"),
					 frame_id("map")
{
	/** Get parameters from the launch file */
	node_.param("robot_height", robotheight, robotheight);
	node_.param("sensor_height", sensorheight, sensorheight);
	node_.param("horizontal_fov", visionangle, visionangle);
	node_.param("resolution", resolution, resolution);
	node_.param("topic_pub_doors", topic_pub1, topic_pub1);
	node_.param("sensor_topic_sub", sensor_topic_sub, sensor_topic_sub);
	node_.param("pose_topic_sub", pose_topic_sub, pose_topic_sub);
	node_.param("map_topic_sub", map_topic_sub, map_topic_sub);
	node_.param("people_topic_sub", people_topic_sub, people_topic_sub);
	node_.param("frame_id", frame_id, frame_id);

	/** Define Subscriber */
	SensorSub = ns_.subscribe(sensor_topic_sub, 50, &pff_sem::filter, this);
	PeopleSub = ns_.subscribe(people_topic_sub, 50, &pff_sem::peopleCallback, this);
	PoseSub = ns_.subscribe(pose_topic_sub, 50, &pff_sem::poseAMCLCallback, this);
	MapSub = ns_.subscribe(map_topic_sub, 50, &pff_sem::mapCallback, this);

	/** Define Publisher */
	doors_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub1, 1, false);

	pose_x = 0;
	pose_y = 0;
	initial_angle = 0;
	laserCloudout.points.resize(0);
}

void pff_sem::peopleCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
	pcl::fromROSMsg(*input, PeopleCloudIn);
}

void pff_sem::filter(const sensor_msgs::PointCloud2ConstPtr &input)
{

	pcl::PointCloud<pcl::PointXYZ> laserCloudIn, FirstCloud, SecondCloud, ThirdCloud;
	pcl::fromROSMsg(*input, laserCloudIn);

	unsigned int num_readings = 360 / resolution;
	FirstCloud.points.resize(num_readings);
	SecondCloud.points.resize(num_readings);
	ThirdCloud.points.resize(num_readings);
	double FirstRanges[num_readings] = {0};
	int StartCluster[num_readings] = {0};
	int EndCluster[num_readings] = {0};

	/** Filter by angle, Extraction of nearest point and prjection onto 2D plane*/
	for (int i = 0; i < laserCloudIn.size(); i++)
	{
		float hip = sqrt(pow(laserCloudIn[i].x, 2) + pow(laserCloudIn[i].y, 2));
		float hangle = (asin((laserCloudIn[i].x) / hip)) * 180 / PI;

		// Dicard points outside the area of interest (Horizontal Filter)
		if ((laserCloudIn[i].z < -sensorheight + 0.05) || isnan(laserCloudIn[i].x)==1 || isnan(laserCloudIn[i].y)==1 ||
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

		// Extraction of nearest points
		if (FirstRanges[position] == 0 || hip < FirstRanges[position])
		{
			FirstCloud.points[position].x = laserCloudIn[i].x;
			FirstCloud.points[position].y = laserCloudIn[i].y;
			FirstCloud.points[position].z = 0; // Projection onto 2D
			FirstRanges[position] = hip;
		}
	}

	int newpos = 0;
	for (int i = 0; i < FirstCloud.size(); i++)
	{
		if (fabs(FirstCloud.points[i].x) < 0.1 && fabs(FirstCloud.points[i].y) < 0.1)
		{
			continue;
		}
		else
		{
			FirstCloud.points[newpos].x = FirstCloud.points[i].x;
			FirstCloud.points[newpos].y = FirstCloud.points[i].y;
			newpos++;
		}
	}
	FirstCloud.resize(newpos);

	// Define clusters
	int cluster_position = 0;
	StartCluster[cluster_position] = 0;
	for (int i = 0; i < FirstCloud.size() - 1; i++)
	{
		float hip_cluster = sqrt(pow(FirstCloud.points[i + 1].x - FirstCloud.points[i].x, 2) + pow(FirstCloud.points[i + 1].y - FirstCloud.points[i].y, 2));
		if (hip_cluster > 0.6)
		{
			EndCluster[cluster_position] = i;
			cluster_position++;
			StartCluster[cluster_position] = i + 1;
		}
	}
	EndCluster[cluster_position] = FirstCloud.size() - 1;

	// Define doors
	float size1min = 0.8;
	float size1max = 1.0;
	int doorpos = 0;
	for (int i = 0; i < cluster_position + 1; i++)
	{
		for (int j = 0; j < cluster_position + 1; j++)
		{
			if (i == j)
			{
				continue;
			}
			float init_second_x = FirstCloud.points[StartCluster[j]].x;
			float init_second_y = FirstCloud.points[StartCluster[j]].y;
			float end_first_x = FirstCloud.points[EndCluster[i] - 1].x;
			float end_first_y = FirstCloud.points[EndCluster[i] - 1].y;

			float hip = sqrt(pow(init_second_x - end_first_x, 2) + pow(init_second_y - end_first_y, 2));

			if (hip > 0.9 && hip < 0.98)
			{
				SecondCloud.points[doorpos].x = (init_second_x + end_first_x) / 2;
				SecondCloud.points[doorpos].y = (init_second_y + end_first_y) / 2;
				SecondCloud.points[doorpos].z = 0;
				doorpos++;
			}
		}
	}
	SecondCloud.resize(doorpos);

	// Filter by the distance between the robot and the posible door to reduce errors
	int doorposfinal = 0;
	for (int i = 0; i < SecondCloud.size(); i++)
	{
		float hip = sqrt(pow(SecondCloud.points[i].x, 2) + pow(SecondCloud.points[i].y, 2));
		if (hip < 1.3 && hip > 0.5)
		{
			SecondCloud.points[doorposfinal].x = SecondCloud.points[i].x;
			SecondCloud.points[doorposfinal].y = SecondCloud.points[i].y;
			SecondCloud.points[doorposfinal].z = 0;
			doorposfinal++;
		}
	}
	SecondCloud.resize(doorposfinal);

	SecondCloud = mapfilter(SecondCloud);

	//Publish PeopleCloud
	sensor_msgs::PointCloud2 DoorsCloud_output;
	pcl::toROSMsg(SecondCloud, DoorsCloud_output);
	DoorsCloud_output.header.frame_id = frame_id;
	doors_pub.publish(DoorsCloud_output);
}

/** Map filter */
pcl::PointCloud<pcl::PointXYZ> pff_sem::mapfilter(pcl::PointCloud<pcl::PointXYZ> PeopleCloud)
{
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

		int posxpx = ((-100 - PeopleCloud[i].x) / (-0.05));
		int posypx = ((-100 - PeopleCloud[i].y) / (-0.05));
		int spacecount = 0;

		for (int j = posxpx - 7; j < posxpx + 7; j++)
		{
			for (int k = posypx - 7; k < posypx + 7; k++)
			{
				if (maparray[j][k] == 100 || maparray[j][k] == -1)
				{
					spacecount++;
				}
			}
		}

		if (spacecount < 15)
		{
			int prueba = 0;
			for (int j = 0; j < laserCloudout.size(); j++)
			{
				float hiprem = sqrt(pow(laserCloudout[j].x - PeopleCloud[i].x, 2) + pow(laserCloudout[j].y - PeopleCloud[i].y, 2));
				if (hiprem < 0.9)
				{
					break;
				}
				else
				{
					prueba++;
				}
			}
			if (prueba == laserCloudout.size())
			{
				int counter = 0;
				for (int j = 0; j < PeopleCloudIn.size(); j++)
				{
					float hiprem = sqrt(pow(PeopleCloudIn[j].x - PeopleCloud[i].x, 2) + pow(PeopleCloudIn[j].y - PeopleCloud[i].y, 2));
					if (hiprem < 1.0)
					{
						counter++;
					}
				}
				if (counter == 0)
				{
					laserCloudout.resize(laserCloudout.size() + 1);
					laserCloudout[posout].x = PeopleCloud[i].x;
					laserCloudout[posout].y = PeopleCloud[i].y;
					posout++;
				}
			}
		}
	}

	return laserCloudout;
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
	ros::init(argc, argv, "velodyne_doors");
	pff_sem filter;
	ros::spin();
	return 0;
}
