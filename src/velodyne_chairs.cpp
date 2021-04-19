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
	pcl::PointCloud<pcl::PointXYZ> rotate(pcl::PointCloud<pcl::PointXYZ> Cloud);
	pcl::PointCloud<pcl::PointXYZ> cloudfilter(pcl::PointCloud<pcl::PointXYZ> Cloud, float separation, float minWidth, float maxWidth);
	pcl::PointCloud<pcl::PointXYZ> cloudmatcher(pcl::PointCloud<pcl::PointXYZ> ChairCloud, pcl::PointCloud<pcl::PointXYZ> FirstCloud, pcl::PointCloud<pcl::PointXYZ> SecondCloud, float separation);

	float robotheight, sensorheight, resolution, legs_begin, legs_end, trunk_begin, trunk_end, pose_x, pose_y, initial_angle;
	int visionangle, position, Ncloud;
	std::string topic_pub1, topic_pub2, topic_pub3, topic_pub4, sensor_topic_sub, pose_topic_sub, map_topic_sub, frame_id;

	int** maparray;
	int  map_width, map_height;

private:
	ros::NodeHandle node_, ns_;
	ros::Publisher empty_pub, legs_pub, back_pub, chairs_pub;
	ros::Subscriber SensorSub, PoseSub, MapSub;
};

pff_sem::pff_sem() : node_("~"),
					 robotheight(1.0),
					 sensorheight(0.39),
					 visionangle(360),
					 resolution(0.2),
					 legs_begin(0.25),
					 legs_end(0.60),
					 trunk_begin(0.80),
					 trunk_end(1.50),
					 topic_pub1("/Chairs_PC"),
					 topic_pub2("/First_PC"),
					 topic_pub3("/Second_PC"),
					 topic_pub4("/Third_PC"),
					 sensor_topic_sub("/velodyne_points"),
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

	node_.param("topic_pub_chairs", topic_pub1, topic_pub1);
	node_.param("topic_pub_legs", topic_pub2, topic_pub2);
	node_.param("topic_pub_back", topic_pub3, topic_pub3);
	node_.param("topic_pub_empty", topic_pub4, topic_pub4);
	node_.param("sensor_topic_sub", sensor_topic_sub, sensor_topic_sub);
	node_.param("pose_topic_sub", pose_topic_sub, pose_topic_sub);
	node_.param("map_topic_sub", map_topic_sub, map_topic_sub);
	
	node_.param("frame_id", frame_id, frame_id);

	/** Define Subscriber */
	SensorSub = ns_.subscribe(sensor_topic_sub, 50, &pff_sem::filter, this);
	PoseSub = ns_.subscribe(pose_topic_sub, 50, &pff_sem::poseAMCLCallback, this);
	MapSub = ns_.subscribe(map_topic_sub, 50, &pff_sem::mapCallback, this);

	/** Define Publisher */
	chairs_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub1, 1, false);
	legs_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub2, 1, false);
	back_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub3, 1, false);
	empty_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub4, 1, false);
	
	pose_x = 0;
	pose_y = 0;
	initial_angle = 0;
}

void pff_sem::filter(const sensor_msgs::PointCloud2ConstPtr &input)
{
	pcl::PointCloud<pcl::PointXYZ> laserCloudIn, LegsCloud, BackCloud, EmptyCloud, ChairCloud;
	pcl::PointXYZ pointlegs1, pointlegs2, pointtrunk1, pointtrunk2;
	pcl::fromROSMsg(*input, laserCloudIn);

	unsigned int num_readings = 360 / resolution;
	LegsCloud.points.resize(num_readings);
	BackCloud.points.resize(num_readings);
	EmptyCloud.points.resize(num_readings);
	ChairCloud.points.resize(num_readings);

	float addpos=0;
	double LegsRanges[num_readings]={0}, BackRanges[num_readings]={0}, EmptyRanges[num_readings]={0}, ChairRanges[num_readings]={0};
	int position = 0;


	for (int i = 0; i < laserCloudIn.size(); i++) {
		
		float hip = sqrt((laserCloudIn[i].x)*(laserCloudIn[i].x)+((laserCloudIn[i].y)*(laserCloudIn[i].y)));
		float hangle = (asin ((laserCloudIn[i].x)/hip))*180/PI;

		float hipz = sqrt(pow(hip,2)+pow(laserCloudIn[i].z,2));
		float hanglez = (asin((laserCloudIn[i].z/hipz)*sin(90*PI/180)))*180/PI;
		//ROS_INFO("anglez %f",hanglez*180/PI);

		// Dicard points outside the area of interest (Horizontal Filter)
		if ((laserCloudIn[i].z < -sensorheight + 0.10) || isnan(laserCloudIn[i].x)==1 || isnan(laserCloudIn[i].y)==1 ||
			fabs(laserCloudIn[i].x)<0.01 || fabs(laserCloudIn[i].y)<0.01 ||
			(visionangle <= 180 && laserCloudIn[i].x <= 0) ||
			(visionangle < 180 && laserCloudIn[i].x > 0 && hangle < 90 - (visionangle / 2)) ||
			(visionangle > 180 && visionangle < 360 && laserCloudIn[i].x < 0 && abs(hangle) > (visionangle - 180) / 2))
		{
			continue;
		}


		// Get the position in the arrays for get the nearest points
		if (laserCloudIn[i].y>0) {
			position=(180.0+hangle)/resolution;
		}
		else if (laserCloudIn[i].x>0 && laserCloudIn[i].y<=0) {
			position=(360.0-hangle)/resolution;
		}
		else {
			position=-hangle/resolution;			
		}	

		// Generate  Legs PointCloud
		if (hanglez>-5.5 && hanglez<-4.5 && hip>2.25 && hip <2.4) 
		{
            // Extraction of nearest points
			if (LegsRanges[position]==0 || hip < LegsRanges[position])
			{
				LegsCloud.points[position].x=laserCloudIn[i].x;
				LegsCloud.points[position].y=laserCloudIn[i].y;
				LegsCloud.points[position].z=0;	                    // Projection onto 2D
				LegsRanges[position]=hip;
			}
		}
		// Generate  Back PointCloud
		if (hanglez>8.5 && hanglez<9.5 && hip>1.9 && hip <2.80) 
		{
            // Extraction of nearest points
			if (BackRanges[position]==0 || hip < BackRanges[position])
			{
				BackCloud.points[position].x=laserCloudIn[i].x;
				BackCloud.points[position].y=laserCloudIn[i].y;
				BackCloud.points[position].z=0;	                    // Projection onto 2D
				BackRanges[position]=hip;
			}	
		}

		// Generate  Empty PointCloud
		if (hanglez>14.5) 
		{
            // Extraction of nearest points
			if (EmptyRanges[position]==0 || hip < EmptyRanges[position])
			{
				EmptyCloud.points[position].x=laserCloudIn[i].x;
				EmptyCloud.points[position].y=laserCloudIn[i].y;
				EmptyCloud.points[position].z=0;	                    // Projection onto 2D
				EmptyRanges[position]=hip;
			}	
		}
	}
	
	LegsCloud = cloudfilter(LegsCloud,0.4,0.01,0.1);
	BackCloud = cloudfilter(BackCloud,0.4,0.1,0.6);
	ChairCloud = cloudmatcher(ChairCloud,LegsCloud,BackCloud,0.4);
	

	//cleare taking in a count the EmptyCloud
	int newpos =0 ;
	for (int i = 0 ; i < ChairCloud.size();i++)
	{
		float hip = sqrt(pow(ChairCloud.points[i].x,2)+pow(ChairCloud.points[i].y,2));
		float hangle = (asin ((ChairCloud.points[i].x)/hip))*180/PI;

		if (ChairCloud.points[i].y>0) {
			position=(180.0+hangle)/resolution;
		}
		else if (ChairCloud.points[i].x>0 && ChairCloud.points[i].y<=0) {
			position=(360.0-hangle)/resolution;
		}
		else {
			position=-hangle/resolution;			
		}	

		int cleanacept=0;

		for (int j = position-20 ; j < position+20 ; j++)
		{
			
			if(EmptyRanges[j]-0.1<hip)
			{
				cleanacept++;
			}
		}
		if(cleanacept==0)
		{
			ChairCloud.points[newpos].x=ChairCloud.points[i].x;
			ChairCloud.points[newpos].y=ChairCloud.points[i].y;
			newpos++;
		}
	}
	ChairCloud.resize(newpos);



	
	EmptyCloud = rotate(EmptyCloud);
	LegsCloud = rotate(LegsCloud);
	BackCloud = rotate(BackCloud);
	ChairCloud = rotate(ChairCloud);

	//Publish EmptyPC
	sensor_msgs::PointCloud2 EmptyCloud_output;
	pcl::toROSMsg(EmptyCloud, EmptyCloud_output);
	EmptyCloud_output.header.frame_id = frame_id;
	empty_pub.publish(EmptyCloud_output);

	//Publish LegsPC
	sensor_msgs::PointCloud2 LegsCloud_output;
	pcl::toROSMsg(LegsCloud, LegsCloud_output);
	LegsCloud_output.header.frame_id = frame_id;
	legs_pub.publish(LegsCloud_output);

	//Publish BackPC
	sensor_msgs::PointCloud2 BackCloud_output;
	pcl::toROSMsg(BackCloud, BackCloud_output);
	BackCloud_output.header.frame_id = frame_id;
	back_pub.publish(BackCloud_output);

	//Publish ChairPC
	sensor_msgs::PointCloud2 ChairsCloud_output;
	pcl::toROSMsg(ChairCloud, ChairsCloud_output);
	ChairsCloud_output.header.frame_id = frame_id;
	chairs_pub.publish(ChairsCloud_output);
}

/** Cloud matcher */
pcl::PointCloud<pcl::PointXYZ> pff_sem::cloudmatcher(pcl::PointCloud<pcl::PointXYZ> ChairCloud, pcl::PointCloud<pcl::PointXYZ> FirstCloud, pcl::PointCloud<pcl::PointXYZ> SecondCloud, float separation)
{
	int pospos = 0;
	for (int i = 0; i < SecondCloud.size(); i++)
	{
		for (int j = 0; j < FirstCloud.size(); j++)
		{
			// Get the distance between the points in Trunk PointCloud and Legs PointCloud
			float hip1 = sqrt(pow(SecondCloud.points[i].x - FirstCloud.points[j].x, 2.0) + pow(SecondCloud.points[i].y - FirstCloud.points[j].y, 2.0));
			// If exist a match between the two clouds the point in Trunk PointCloud us stored in People PointCloud
			if (hip1 < separation)
			{
				int count = 0;
				for (int k = 1; k < ChairCloud.size(); k++)
				{
					float hip2 = sqrt(pow(SecondCloud.points[i].x - ChairCloud.points[k].x, 2.0) + pow(SecondCloud.points[i].y - ChairCloud.points[k].y, 2.0));
					if (hip2<0.4){count++;}
				}
				if (count == 0)
				{
					ChairCloud.points[pospos].x = SecondCloud.points[i].x;
					ChairCloud.points[pospos].y = SecondCloud.points[i].y;
					ChairCloud.points[pospos].z = 0;
					pospos++;
				}
			}
		}
	}	
	
	ChairCloud.points.resize(pospos);
	
	
	return ChairCloud;
}


/** clouds filters */
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


/** Rotate */
pcl::PointCloud<pcl::PointXYZ> pff_sem::rotate(pcl::PointCloud<pcl::PointXYZ> Cloud)
{
	for (int i = 0; i < Cloud.size(); i++)
	{
		int position = 0;
		float angle_min = 3 * PI / 2;
		float angle_increment = -2 * PI / (360 / resolution);
		float hip = sqrt((Cloud[i].x) * (Cloud[i].x) + ((Cloud[i].y) * (Cloud[i].y)));
		float hangle = (asin((Cloud[i].x) / hip)) * 180 / PI;

		if (Cloud[i].y > 0)
		{
			position = (180.0 + hangle) / resolution;
		}
		else if (Cloud[i].x > 0 && Cloud[i].y <= 0)
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

		Cloud[i].x = -laserpointx + pose_x;
		Cloud[i].y = laserpointy + pose_y;
	}

	return Cloud;
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
	ros::init(argc, argv, "velodyne_chairs");
	pff_sem filter;
	ros::spin();
	return 0;
}
