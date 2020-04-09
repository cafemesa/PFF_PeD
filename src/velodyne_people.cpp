#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#define PI 3.14159265

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>



#include "std_msgs/Int32.h"

#include "std_msgs/String.h"
#include <math.h>
#include <iostream>
#include <nav_msgs/Odometry.h>

ros::Publisher PeopleCloud_pub,TrunkCloud_pub,LegsCloud_pub,Markers;	
pcl::PointCloud<pcl::PointXYZ> PeopleCloud1, PeopleCloud2, PeopleCloud3, PeopleCloud4,laserCloudout;
int posout=0;
float sensorheight, minvisionrange, maxvisionrange, resolution, hip, hangle, legs_begin, legs_end, trunk_begin, trunk_end;
int visionangle;
int Ncloud=0;
int point1pos, point2pos,point3pos, point4pos;

float pose_x=0, pose_y=0, initial_angle=0;
int maparray [4000][4000];

const std::string now_str();
std::ofstream ptime;
std::string abegtime;
std::string aendtime;

/**
 * This function recieve the Velodyne PointCloud. 
 * Create the Trunk PointCloud and the Legs PointCloud.
 * Matching between Trunk PointCloud and Legs PointCloud to create People PointCloud
 * Run a speed filter to get the mooving people.
 * publish th final People PointCLoud
 *
 * @param input Velodyne PointCloud directly from the sensor
 */

void 
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& input)
{ 
	Ncloud++;
	abegtime=now_str();
	
	pcl::PointCloud<pcl::PointXYZ> laserCloudIn,LegsCloud, TrunkCloud, PeopleCloud;
	pcl::PointXYZ pointlegs1, pointlegs2, pointtrunk1, pointtrunk2;
	pcl::fromROSMsg(*input, laserCloudIn);

    unsigned int num_readings = 360/resolution;
	LegsCloud.points.resize(num_readings);
	TrunkCloud.points.resize(num_readings);
	PeopleCloud.points.resize(num_readings);

	float addpos=0;
	double LegsRanges[num_readings]={0}, TrunkRanges[num_readings]={0};
	int position = 0, postrunk10=0,  postrunk30=0, poslegs10=0, poslegs30=0;

	
	for (int i = 0; i < laserCloudIn.size(); i++) {
		
		hip = sqrt((laserCloudIn[i].x)*(laserCloudIn[i].x)+((laserCloudIn[i].y)*(laserCloudIn[i].y)));
		hangle = (asin ((laserCloudIn[i].x)/hip))*180/PI;

		// Dicard points outside the area of interest (Horizontal Filter)
		if (visionangle==180 && laserCloudIn[i].x<=0){continue;}
		else if ((visionangle<180 && laserCloudIn[i].x>0 && hangle<90-(visionangle/2)) || (visionangle<180 && laserCloudIn[i].x<=0)){continue;}
		else if (visionangle>180 && visionangle<360 && laserCloudIn[i].x<0 && abs(hangle)>(visionangle-180)/2){continue;}
		
        // Dicard points outside the area of interest (Range filter)
        else if (hip < minvisionrange || hip > maxvisionrange ){continue;}

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

		// Generate  Legs Point Cloud
		if (-sensorheight+legs_begin<laserCloudIn[i].z && laserCloudIn[i].z < -sensorheight + legs_end) 
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
		// Generate  Trunk Point Cloud
		if (-sensorheight+trunk_begin<laserCloudIn[i].z && laserCloudIn[i].z < -sensorheight + trunk_end) 
		{
            // Extraction of nearest points
			if (TrunkRanges[position]==0 || hip < TrunkRanges[position])
			{
				TrunkCloud.points[position].x=laserCloudIn[i].x;
				TrunkCloud.points[position].y=laserCloudIn[i].y;
				TrunkCloud.points[position].z=0;	                    // Projection onto 2D
				TrunkRanges[position]=hip;
			}	
		}
	}
	
	//Legs Filter
	for (int i = 0; i < LegsCloud.size(); i++) {
		if (fabs(LegsCloud.points[i].x)<0.1 && fabs(LegsCloud.points[i].y)<0.1){continue;}		
		// Save the first point of the sample	
        if (poslegs30 ==0)
		{
			pointlegs1.x=LegsCloud.points[i].x;
			pointlegs1.y=LegsCloud.points[i].y;
			point1pos=i;
			pointlegs2.x=LegsCloud.points[i].x;
			pointlegs2.y=LegsCloud.points[i].y;
			point2pos=i;
			poslegs30++;
		}
		else
		{
			float hiplegs1 = sqrt(pow(LegsCloud.points[i].x-pointlegs1.x,2.0) + pow(LegsCloud.points[i].y-pointlegs1.y,2.0));
			
            // Check if a new cluster appear in the field of view
			if (hiplegs1>0.25)
			{
				float hiplegs2 = sqrt(pow(pointlegs1.x-pointlegs2.x,2.0) + pow(pointlegs1.y-pointlegs2.y,2.0));
				// Chech if the detecte object have the posible dimension for a leg
                if (0.05<hiplegs2 && hiplegs2<0.65)
				{
					//Save de average   between the first and the last point in the group
					LegsCloud.points[poslegs10].x=(pointlegs1.x+pointlegs2.x)/2;
					LegsCloud.points[poslegs10].y=(pointlegs1.y+pointlegs2.y)/2;				
					LegsCloud.points[poslegs10].z=0;
					poslegs10++;		
				}
                // Store the first point of the new detection
				pointlegs2.x=LegsCloud.points[i].x;
				pointlegs2.y=LegsCloud.points[i].y;	
				point2pos=i;
			}
            //Store the last point to compare in the nest round
			pointlegs1.x=LegsCloud.points[i].x;
			pointlegs1.y=LegsCloud.points[i].y;
			point1pos=i;
		}
	}

	//Trunk Filter
	for (int i = 0; i < TrunkCloud.size(); i++) {
        if (fabs(TrunkCloud.points[i].x)<0.1 && fabs(TrunkCloud.points[i].y)<0.1){continue;}
		// Save the first point of the sample
		if (postrunk30 ==0)
		{
			pointtrunk1.x=TrunkCloud.points[i].x;
			pointtrunk1.y=TrunkCloud.points[i].y;
			point3pos=i;
			pointtrunk2.x=TrunkCloud.points[i].x;
			pointtrunk2.y=TrunkCloud.points[i].y;
			point4pos=i;
			postrunk30++;
		}
		else
		{
			float hiptrunk1 = sqrt(pow(TrunkCloud.points[i].x-pointtrunk1.x,2.0) + pow(TrunkCloud.points[i].y-pointtrunk1.y,2.0));
			// Check if a new detection appear in the field of view
            if (hiptrunk1>0.25)
			{
				float hiptrunk2 = sqrt(pow(pointtrunk1.x-pointtrunk2.x,2.0) + pow(pointtrunk1.y-pointtrunk2.y,2.0));
				// Chech if the detecte object have the posible dimension for a Trunk
                if (0.15<hiptrunk2 && hiptrunk2<0.65)
				{	
					//Save de average between the first and the last point in the group 	
					TrunkCloud.points[postrunk10].x=(pointtrunk1.x+pointtrunk2.x)/2;
					TrunkCloud.points[postrunk10].y=(pointtrunk1.y+pointtrunk2.y)/2;
					TrunkCloud.points[postrunk10].z=0;
					postrunk10++;											
				}
                // Store the first point of the new detection
				pointtrunk2.x=TrunkCloud.points[i].x;
				pointtrunk2.y=TrunkCloud.points[i].y;	
				point4pos=i;
			}
            //Store the last point to compare in the nest round
			pointtrunk1.x=TrunkCloud.points[i].x;
			pointtrunk1.y=TrunkCloud.points[i].y;
			point3pos=i;
		}
	}

	// Trunk and Legs PointCloud resize
	TrunkCloud.points.resize(postrunk10);
	LegsCloud.points.resize(poslegs10);	
		
	//People Matcher 
	int pospos=0;
	for (int i = 1 ; i < TrunkCloud.size();i++)
	{
		for (int j = 1 ; j < LegsCloud.size();j++)
		{
            // Get the distance between the points in Trunk PointCloud and Legs PointCloud
			float hip1 = sqrt(pow(TrunkCloud.points[i].x-LegsCloud.points[j].x,2.0) + pow(TrunkCloud.points[i].y-LegsCloud.points[j].y,2.0));
			// If exist a match between the two clouds the point in Trunk PointCloud us stored in People PointCloud
            if(hip1<0.35)
			{
                // Store the first point
				if (pospos==0)
				{
					PeopleCloud.points[pospos].x=TrunkCloud.points[i].x;
					PeopleCloud.points[pospos].y=TrunkCloud.points[i].y;
					PeopleCloud.points[pospos].z=0;
					pospos++;
				}
				else
				{
					float hip20=10;
					float hip10 = sqrt(pow(PeopleCloud.points[pospos-1].x-TrunkCloud.points[i].x,2.0) + pow(PeopleCloud.points[pospos-1].y-TrunkCloud.points[i].y,2.0));
					if (pospos>1)
					{
					float hip20 = sqrt(pow(PeopleCloud.points[pospos-2].x-TrunkCloud.points[i].x,2.0) + pow(PeopleCloud.points[pospos-2].y-TrunkCloud.points[i].y,2.0));
					}
					if (hip10>0.6 && hip20>0.6)
					{
						PeopleCloud.points[pospos].x=TrunkCloud.points[i].x;
						PeopleCloud.points[pospos].y=TrunkCloud.points[i].y;
						PeopleCloud.points[pospos].z=0;
						pospos++;
					}
				}				
			}	
		}
	}

    // People PointCloud resize
	PeopleCloud.points.resize(pospos);
   
	


	std::cout << "Begin=" <<  abegtime << "Finish=" <<  now_str() << std::endl;

	for (int i = 0; i < PeopleCloud.size(); i++) {
		
		//ROS_INFO("x=%f, y=%f", PeopleCloud[i].x,PeopleCloud[i].y);
		float hip = sqrt((PeopleCloud[i].x)*(PeopleCloud[i].x)+((PeopleCloud[i].y)*(PeopleCloud[i].y)));
		float hangle = fabs(asin ((PeopleCloud[i].x)/hip));
		
		if (PeopleCloud[i].x>0 && PeopleCloud[i].y>0){hangle=(PI/2)-hangle;}
		if (PeopleCloud[i].x>0 && PeopleCloud[i].y<0){hangle=(3*PI/2)+hangle;}
		if (PeopleCloud[i].x<0 && PeopleCloud[i].y<0){hangle=(3*PI/2)-hangle;}
		if (PeopleCloud[i].x<0 && PeopleCloud[i].y>0){hangle=(PI/2)+hangle;}

		float finalangle=0;
		if (initial_angle<0){
			finalangle=hangle+initial_angle+2*PI;
		}
		else{
			finalangle=hangle+initial_angle;
		}


		if (finalangle>2*PI){finalangle=finalangle-(2*PI);}

		
		if (finalangle>0 && finalangle<PI/2)
		{
			PeopleCloud[i].y= hip*sin(finalangle);
			PeopleCloud[i].x= sqrt(-((PeopleCloud[i].y)*(PeopleCloud[i].y))+((hip)*(hip)));
		}
		else if (finalangle>PI/2 && finalangle<PI)
		{
			PeopleCloud[i].y= hip*sin(PI-finalangle);
			PeopleCloud[i].x= -sqrt(-((PeopleCloud[i].y)*(PeopleCloud[i].y))+((hip)*(hip)));
		}
		else if (finalangle>PI && finalangle<3*PI/2)
		{
			PeopleCloud[i].y= -hip*sin(finalangle-PI);
			PeopleCloud[i].x= -sqrt(-((PeopleCloud[i].y)*(PeopleCloud[i].y))+((hip)*(hip)));
		}
		else if (finalangle>3*PI/2 && finalangle<2*PI)
		{
			PeopleCloud[i].y= -hip*sin(2*PI-finalangle);
			PeopleCloud[i].x= sqrt(-((PeopleCloud[i].y)*(PeopleCloud[i].y))+((hip)*(hip)));
		
		}
		PeopleCloud[i].x=PeopleCloud[i].x+pose_x;
		PeopleCloud[i].y=PeopleCloud[i].y+pose_y;	

		int posxpx=((-100-PeopleCloud[i].x)/(-0.05));
		int posypx=((-100-PeopleCloud[i].y)/(-0.05));
		int spacecount=0;
		//ROS_INFO("pos, %d, %d, %d, %d",posxpx-7,posxpx+7,posypx-7,posypx+7);

		for (int j = posxpx-7; j < posxpx+7; j++) {
			for (int k = posypx-7; k < posypx+7; k++) {
				if (maparray[j][k]==100 || maparray[j][k]==-1)
				{
					spacecount++;
				}
			}		
		}

		if (spacecount>15)
		{
			PeopleCloud[i].x=0;
			PeopleCloud[i].y=0;			
		}
	}
	//Publish PeopleCloud
	sensor_msgs::PointCloud2 PeopleCloud_output;
	pcl::toROSMsg(PeopleCloud, PeopleCloud_output);
    PeopleCloud_output.header.frame_id = "map";
	PeopleCloud_pub.publish (PeopleCloud_output);

}


void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    pose_x = msgAMCL->pose.pose.position.x;
    pose_y = msgAMCL->pose.pose.position.y;
    //poseAMCLa = msgAMCL->pose.pose.orientation.w;   
	initial_angle = 2.0*asin(msgAMCL->pose.pose.orientation.z);	
    //ROS_INFO(msgAMCL);
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;  
  for (unsigned int x = 0; x < info.width; x++){
    for (unsigned int y = 0; y < info.height; y++){
		maparray[x][y]=msg->data[x+ info.width * y];
	}  
  }
}


int
main (int argc, char **argv)
{
	// Initialize ROS
	ros::init (argc, argv, "velodyne_people");
	ros::NodeHandle nh;
	laserCloudout.points.resize(1000);
	// Get launch parameters
	if(!nh.getParam("/velodyne_people/sensor_height",sensorheight)){sensorheight = 0.37;}
	if(!nh.getParam("/velodyne_people/min_vision_range",minvisionrange)){minvisionrange = 0.3;}
	if(!nh.getParam("/velodyne_people/max_vision_range",maxvisionrange)){maxvisionrange = 50.0;}
	if(!nh.getParam("/velodyne_people/horizontal_fov",visionangle)){visionangle = 360;}
	if(!nh.getParam("/velodyne_people/resolution",resolution)){resolution = 0.40;}
	if(!nh.getParam("/velodyne_people/legs_begin",legs_begin)){legs_begin=0.25;}
	if(!nh.getParam("/velodyne_people/legs_end",legs_end)){legs_end=0.60;}
	if(!nh.getParam("/velodyne_people/trunk_begin",trunk_begin)){trunk_begin=0.80;}
	if(!nh.getParam("/velodyne_people/trunk_end",trunk_end)){trunk_end=1.50;}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb2);
	ros::Subscriber sub_amcl = nh.subscribe("amcl_pose", 1, poseAMCLCallback);	
	ros::Subscriber map_sub = nh.subscribe("/map",10,mapCallback);
	
	// Create a ROS publisher for the outputs	
	PeopleCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/PeopleCloud", 1);
	TrunkCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/TrunkCloud", 1);
	LegsCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/LegsCloud", 1);
	Markers = nh.advertise<visualization_msgs::Marker>( "marker", 1 );
	
	// Spin
	ros::spin ();	
}



const std::string now_str()
{
    // Get current time from the clock, using microseconds resolution
    const boost::posix_time::ptime now = 
        boost::posix_time::microsec_clock::local_time();

    // Get the time offset in current day
    const boost::posix_time::time_duration td = now.time_of_day();

    //
    // Extract hours, minutes, seconds and milliseconds.
    //
    // Since there is no direct accessor ".milliseconds()",
    // milliseconds are computed _by difference_ between total milliseconds
    // (for which there is an accessor), and the hours/minutes/seconds
    // values previously fetched.
    //
    const long hours        = td.hours();
    const long minutes      = td.minutes();
    const long seconds      = td.seconds();
    const long milliseconds = td.total_milliseconds() -
                              ((hours * 3600 + minutes * 60 + seconds) * 1000);
	const long microseconds = td.total_microseconds() -
                              (((hours * 3600 + minutes * 60 + seconds) * 1000000))-(milliseconds*1000);

    //
    // Format like this:
    //
    //      hh:mm:ss.SSS
    //
    // e.g. 02:15:40:321
    //
    //      ^          ^
    //      |          |
    //      123456789*12
    //      ---------10-     --> 12 chars + \0 --> 13 chars should suffice
    //  
    // 
    char buf[40];
    //sprintf(buf, "%02ld:%02ld:%02ld.%03ld.%03ld",hours, minutes, seconds, milliseconds, microseconds);
	sprintf(buf, "%02ld:%03ld:%03ld",seconds, milliseconds, microseconds);

    return buf;
}
