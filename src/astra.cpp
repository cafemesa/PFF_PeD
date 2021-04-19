#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
#define PI 3.14159265

class pff_ped
{
public:
	pff_ped();
	void filter(const sensor_msgs::PointCloud2ConstPtr &input);
	pcl::PointCloud<pcl::PointXYZ> cloudfilter(pcl::PointCloud<pcl::PointXYZ> LegsCloud, float separation, float minWidth, float maxWidth);
	pcl::PointCloud<pcl::PointXYZ> cloudmatcher(pcl::PointCloud<pcl::PointXYZ> PeopleCloud, pcl::PointCloud<pcl::PointXYZ> LegsCloud, pcl::PointCloud<pcl::PointXYZ> TrunkCloud, float separation);
	pcl::PointCloud<pcl::PointXYZ> speedfilter(pcl::PointCloud<pcl::PointXYZ> PeopleCloud, int Ncloud);
		pcl::PointCloud<pcl::PointXYZ> PeopleCloud1,
		PeopleCloud2, PeopleCloud3, PeopleCloud4;
	float robotheight, sensorheight, resolution, legs_begin, legs_end, trunk_begin, trunk_end;
	int visionangle, position, Ncloud;
	std::string topic_pub1, topic_pub2, topic_pub3, topic_sub, frame_id;
	struct Point3D
	{

		float x;
		float y;
		float z;
		Point3D(float x_, float y_, float z_)
			: x(x_), y(y_), z(z_)
		{
		}
	};

private:
	ros::NodeHandle node_, ns_;
	ros::Publisher people_pub, legs_pub, trunk_pub;
	ros::Subscriber sub;
};

pff_ped::pff_ped() : node_("~"),
					 robotheight(1.0),
					 sensorheight(0.57),
					 visionangle(360),
					 resolution(0.2),
					 legs_begin(0.25),
					 legs_end(0.60),
					 trunk_begin(0.80),
					 trunk_end(1.50),
					 topic_pub1("/People_PC"),
					 topic_pub2("/Legs_PC"),
					 topic_pub3("/Trunk_PC"),
					 topic_sub("/velodyne_points"),
					 frame_id("velodyne")
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
	node_.param("topic_sub", topic_sub, topic_sub);
	node_.param("frame_id", frame_id, frame_id);

	/** Define Subscriber */
	sub = ns_.subscribe(topic_sub, 50, &pff_ped::filter, this);

	/** Define Publisher */
	people_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub1, 1, false);
	legs_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub2, 1, false);
	trunk_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub3, 1, false);

	Ncloud=0;
}

void pff_ped::filter(const sensor_msgs::PointCloud2ConstPtr &input)
{

	int numPts = input->height * input->width;
	char* raw3DPtsData = (char*)(input->data.data());
	std::vector<Point3D> pc(numPts, Point3D(0.,0.,0.));
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
		float* base = (float*)(raw3DPtsData + i * input->point_step);
		Point3D point3d(0.,0.,0.);       

		laserCloudIn[i].x=base[0];
		laserCloudIn[i].y=base[2];
		laserCloudIn[i].z=-base[1];

		float hip = sqrt(pow(laserCloudIn[i].x, 2) + pow(laserCloudIn[i].y, 2));
		float hangle = (asin((laserCloudIn[i].x) / hip)) * 180 / PI;

		/** Discard points */
		if (laserCloudIn[i].z > robotheight - sensorheight ||
			(-sensorheight - 0.1 <= laserCloudIn[i].z && laserCloudIn[i].z < -sensorheight + 0.05) ||
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
	PeopleCloud = speedfilter(PeopleCloud,Ncloud);

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
pcl::PointCloud<pcl::PointXYZ> pff_ped::cloudfilter(pcl::PointCloud<pcl::PointXYZ> Cloud, float separation, float minWidth, float maxWidth)
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
pcl::PointCloud<pcl::PointXYZ> pff_ped::cloudmatcher(pcl::PointCloud<pcl::PointXYZ> PeopleCloud, pcl::PointCloud<pcl::PointXYZ> LegsCloud, pcl::PointCloud<pcl::PointXYZ> TrunkCloud, float separation)
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

/** Speed filter */
pcl::PointCloud<pcl::PointXYZ> pff_ped::speedfilter(pcl::PointCloud<pcl::PointXYZ> PeopleCloud, int Ncloud)
{	
	// Store the last 4 People PointClouds
	PeopleCloud4 = PeopleCloud3;
	PeopleCloud3 = PeopleCloud2;
	PeopleCloud2 = PeopleCloud1;
	PeopleCloud1 = PeopleCloud;
	int newpos = 0;

	//Speed Filter
	if (Ncloud > 4)
	{
		for (int i = 0; i < PeopleCloud.size(); i++)
		{
			int conteo = 0;
			for (int j = 0; j < PeopleCloud2.size(); j++)
			{
				float hip2 = sqrt(pow(PeopleCloud.points[i].x - PeopleCloud2.points[j].x, 2) + pow(PeopleCloud.points[i].y - PeopleCloud2.points[j].y, 2));
				if (hip2 < 0.3 && hip2 > 0.1)
				{
					conteo++;
				}
			}
			for (int j = 0; j < PeopleCloud3.size(); j++)
			{
				float hip3 = sqrt(pow(PeopleCloud.points[i].x - PeopleCloud3.points[j].x, 2) + pow(PeopleCloud.points[i].y - PeopleCloud3.points[j].y, 2));
				if (hip3 < 0.3 && hip3 > 0.1)
				{
					conteo++;
				}
			}
			for (int j = 0; j < PeopleCloud4.size(); j++)
			{
				float hip4 = sqrt(pow(PeopleCloud.points[i].x - PeopleCloud4.points[j].x, 2) + pow(PeopleCloud.points[i].y - PeopleCloud4.points[j].y, 2));
				if (hip4 < 0.3 && hip4 > 0.1)
				{
					conteo++;
				}
			}

			if (conteo > 0)
			{
				PeopleCloud.points[newpos].x = PeopleCloud.points[i].x;
				PeopleCloud.points[newpos].y = PeopleCloud.points[i].y;
				newpos++;
			}
		}
		PeopleCloud.points.resize(newpos);
	}
	return PeopleCloud;
}

/** Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "astra_ped");
	pff_ped filter;
	ros::spin();
	return 0;
}
