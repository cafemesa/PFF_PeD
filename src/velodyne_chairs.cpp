#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#define PI 3.14159265

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
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





ros::Publisher ChairCloud_pub,Markers;	
pcl::PointCloud<pcl::PointXYZ> laserCloudout;
float sensorheight, minvisionrange, maxvisionrange, resolution, hip, hangle;
int visionangle;

int posout=0;
float pose_x=0, pose_y=0, initial_angle=0;

const std::string now_str();
std::ofstream ptime;
std::string abegtime;
std::string aendtime;

void 
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& input)
{ 

	
	//abegtime=now_str();

	pcl::PointCloud<pcl::PointXYZ> laserCloudIn,LegsCloud, BackCloud, EmptyCloud, ChairCloud;
	pcl::fromROSMsg(*input, laserCloudIn);

    unsigned int num_readings = 360/resolution;
	LegsCloud.points.resize(num_readings);
	BackCloud.points.resize(num_readings);
	EmptyCloud.points.resize(num_readings);
	ChairCloud.points.resize(num_readings);

	float addpos=0;
	double LegsRanges[num_readings]={0}, BackRanges[num_readings]={0}, EmptyRanges[num_readings]={0}, ChairRanges[num_readings]={0};
	int position = 0;
	for (int i = 0; i < sizeof(EmptyRanges); i++) {EmptyRanges[i]=100;}
	
	for (int i = 0; i < laserCloudIn.size(); i++) {
		
		hip = sqrt((laserCloudIn[i].x)*(laserCloudIn[i].x)+((laserCloudIn[i].y)*(laserCloudIn[i].y)));
		hangle = (asin ((laserCloudIn[i].x)/hip))*180/PI;

		float hipz = sqrt(pow(hip,2)+pow(laserCloudIn[i].z,2));
		float hanglez = (asin((laserCloudIn[i].z/hipz)*sin(90*PI/180)))*180/PI;
		//ROS_INFO("anglez %f",hanglez*180/PI);

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
			if (EmptyRanges[position]==100 || hip < EmptyRanges[position])
			{
				EmptyCloud.points[position].x=laserCloudIn[i].x;
				EmptyCloud.points[position].y=laserCloudIn[i].y;
				EmptyCloud.points[position].z=0;	                    // Projection onto 2D
				EmptyRanges[position]=hip;
			}	
		}
	}
	
	/* Legs filter*/
	int poslegsfilter=0;
	for (int i = 0; i < LegsCloud.size(); i++) 
	{
		float hiplegs1 = sqrt(pow(LegsCloud.points[i].x,2.0) + pow(LegsCloud.points[i].y,2.0));	
		if (hiplegs1>1.5 && hiplegs1 <3.0)
		{
			if (poslegsfilter==0)
			{
				LegsCloud.points[poslegsfilter].x=LegsCloud.points[i].x;
				LegsCloud.points[poslegsfilter].y=LegsCloud.points[i].y;
				poslegsfilter++;
			}
			else
			{
				int legspointrep=0;
				for (int j = 0; j < poslegsfilter; j++) 
				{
					float hiplegs2 = sqrt(pow(LegsCloud.points[i].x-LegsCloud.points[j].x,2.0) + pow(LegsCloud.points[i].y-LegsCloud.points[j].y,2.0));	
					if (hiplegs2<0.3)
					{
						legspointrep++;
					}
				}
				if (legspointrep==0)
				{
					LegsCloud.points[poslegsfilter].x=LegsCloud.points[i].x;
					LegsCloud.points[poslegsfilter].y=LegsCloud.points[i].y;
					poslegsfilter++;
				}
			}
		}
	}
	LegsCloud.points.resize(poslegsfilter);

	/* backfilter */
	int posbackfilter=0;
	for (int i = 0; i < BackCloud.size(); i++) 
	{
		float hipback1 = sqrt(pow(BackCloud.points[i].x,2.0) + pow(BackCloud.points[i].y,2.0));	
		if (hipback1>1.5 && hipback1 <3.0)
		{
			if (posbackfilter==0)
			{
				BackCloud.points[posbackfilter].x=BackCloud.points[i].x;
				BackCloud.points[posbackfilter].y=BackCloud.points[i].y;
				posbackfilter++;
			}
			else
			{
				int legspointrep=0;
				for (int j = 0; j < posbackfilter; j++) 
				{
					float hipback2 = sqrt(pow(BackCloud.points[i].x-BackCloud.points[j].x,2.0) + pow(BackCloud.points[i].y-BackCloud.points[j].y,2.0));	
					if (hipback2<0.3)
					{
						legspointrep++;
					}
				}
				if (legspointrep==0)
				{
					BackCloud.points[posbackfilter].x=BackCloud.points[i].x;
					BackCloud.points[posbackfilter].y=BackCloud.points[i].y;
					posbackfilter++;
				}
			}
		}
	}
	BackCloud.points.resize(posbackfilter);	

	//Chair Matcher 
	int pospos=0;
	for (int i = 0 ; i < LegsCloud.size();i++)
	{
		for (int j = 0 ; j < BackCloud.size();j++)
		{
			float hip1 = sqrt(pow(LegsCloud.points[i].x-BackCloud.points[j].x,2.0) + pow(LegsCloud.points[i].y-BackCloud.points[j].y,2.0));
			if (hip1<30 && pospos==0)
			{
				ChairCloud.points[pospos].x=LegsCloud.points[i].x;
				ChairCloud.points[pospos].y=LegsCloud.points[i].y;
				pospos++;
			}
			else if (hip1<30 && pospos>0)
			{
				int chairpointrep=0;
				for (int k = 0; k < pospos; k++) 
				{
					float hip2 = sqrt(pow(LegsCloud.points[i].x-ChairCloud.points[k].x,2.0) + pow(LegsCloud.points[i].y-ChairCloud.points[k].y,2.0));
					if (hip2<0.3)
					{
						chairpointrep++;
					}
				}
				if (chairpointrep==0)
				{
					ChairCloud.points[pospos].x=LegsCloud.points[i].x;
					ChairCloud.points[pospos].y=LegsCloud.points[i].y;
					pospos++;
				}
			}
		}
	}
	ChairCloud.points.resize(pospos);

	//cleare taking in a count the EmptyCloud
	for (int i = 0 ; i < ChairCloud.size();i++)
	{
		hip = sqrt(pow(ChairCloud.points[i].x,2)+pow(ChairCloud.points[i].y,2));
		hangle = (asin ((ChairCloud.points[i].x)/hip))*180/PI;

		// Get the position in the arrays for get the nearest points
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
		if(cleanacept>0)
		{
			ChairCloud.points[i].x=0;
			ChairCloud.points[i].y=0;
		}
	}

	// clean cloud
	int finlposchair=0;
	for (int i = 0 ; i < ChairCloud.size();i++)
	{
		hip = sqrt(pow(ChairCloud.points[i].x,2)+pow(ChairCloud.points[i].y,2));
		if (hip>1.8 and hip <2.8)
		{
			ChairCloud.points[finlposchair].x=ChairCloud.points[i].x;
			ChairCloud.points[finlposchair].y=ChairCloud.points[i].y;
			finlposchair++;
		}
	}
	ChairCloud.points.resize(finlposchair);

	//set position in map
	for (int i = 0; i < ChairCloud.size(); i++) {
		
		//ROS_INFO("x=%f, y=%f", ChairCloud[i].x,ChairCloud[i].y);
		float hip = sqrt((ChairCloud[i].x)*(ChairCloud[i].x)+((ChairCloud[i].y)*(ChairCloud[i].y)));
		float hangle = fabs(asin ((ChairCloud[i].x)/hip));
		
		if (ChairCloud[i].x>0 && ChairCloud[i].y>0){hangle=(PI/2)-hangle;}
		if (ChairCloud[i].x>0 && ChairCloud[i].y<0){hangle=(3*PI/2)+hangle;}
		if (ChairCloud[i].x<0 && ChairCloud[i].y<0){hangle=(3*PI/2)-hangle;}
		if (ChairCloud[i].x<0 && ChairCloud[i].y>0){hangle=(PI/2)+hangle;}

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
			ChairCloud[i].y= hip*sin(finalangle);
			ChairCloud[i].x= sqrt(-((ChairCloud[i].y)*(ChairCloud[i].y))+((hip)*(hip)));
		}
		else if (finalangle>PI/2 && finalangle<PI)
		{
			ChairCloud[i].y= hip*sin(PI-finalangle);
			ChairCloud[i].x= -sqrt(-((ChairCloud[i].y)*(ChairCloud[i].y))+((hip)*(hip)));
		}
		else if (finalangle>PI && finalangle<3*PI/2)
		{
			ChairCloud[i].y= -hip*sin(finalangle-PI);
			ChairCloud[i].x= -sqrt(-((ChairCloud[i].y)*(ChairCloud[i].y))+((hip)*(hip)));
		}
		else if (finalangle>3*PI/2 && finalangle<2*PI)
		{
			ChairCloud[i].y= -hip*sin(2*PI-finalangle);
			ChairCloud[i].x= sqrt(-((ChairCloud[i].y)*(ChairCloud[i].y))+((hip)*(hip)));
		
		}

		ChairCloud[i].x=ChairCloud[i].x+pose_x;
		ChairCloud[i].y=ChairCloud[i].y+pose_y;

		if (posout<1)
		{
			laserCloudout[posout].x=ChairCloud[i].x;
			laserCloudout[posout].y=ChairCloud[i].y;
			posout++;
		}	
		else
		{
			int prueba=0;
			for (int m = 0; m < posout; m++) {
				ROS_INFO("SIZE %d",m);
				hip = sqrt(pow(laserCloudout.points[m].x-ChairCloud.points[i].x,2)+pow(laserCloudout.points[m].y-ChairCloud.points[i].y,2));
				if (hip<0.6){break;}
				else {prueba++;}
			}
			if (prueba==posout)
			{
				laserCloudout[posout].x=ChairCloud[i].x;
				laserCloudout[posout].y=ChairCloud[i].y;
				posout++;
			}
		}		
		
	}

	ROS_INFO("SIZE %d",finlposchair);

	sensor_msgs::PointCloud2 ChairCloud_output;
	pcl::toROSMsg(laserCloudout, ChairCloud_output);
    ChairCloud_output.header.frame_id = "map";
	ChairCloud_pub.publish (ChairCloud_output);


	visualization_msgs::MarkerArray marker;
	marker.markers.resize(posout);

	for (int j = 0 ; j < posout;j++)
	{		
		marker.markers[j].header.frame_id = "map";
		marker.markers[j].header.stamp = ros::Time();
		marker.markers[j].ns = "my_namespace";
		marker.markers[j].id =j;
		marker.markers[j].type = visualization_msgs::Marker::CYLINDER;
		marker.markers[j].action = visualization_msgs::Marker::ADD;
		marker.markers[j].pose.position.x = laserCloudout.points[j].x;
		marker.markers[j].pose.position.y = laserCloudout.points[j].y;
		marker.markers[j].pose.position.z = 0;
		marker.markers[j].pose.orientation.x = 0.0;
		marker.markers[j].pose.orientation.y = 0.0;
		marker.markers[j].pose.orientation.z = 0.0;
		marker.markers[j].pose.orientation.w = 1.0;
		marker.markers[j].scale.x = 0.50;
		marker.markers[j].scale.y = 0.50;
		marker.markers[j].scale.z = 0.001;
		marker.markers[j].color.a = 0.5; 
		//marker.markers[j].lifetime = ros::Duration(0.2);
		marker.markers[j].color.r = 0.0; 
		marker.markers[j].color.g = 1.0; 
		marker.markers[j].color.b = 0.0;
		marker.markers[j].mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	}
	Markers.publish(marker);


}

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    pose_x = msgAMCL->pose.pose.position.x;
    pose_y = msgAMCL->pose.pose.position.y;   
	initial_angle = 2.0*asin(msgAMCL->pose.pose.orientation.z);	
}

int
main (int argc, char **argv)
{
	// Initialize ROS
	ros::init (argc, argv, "velodyne_chairs");
	ros::NodeHandle nh;

	// Get launch parameters
	if(!nh.getParam("/velodyne_chairs/sensor_height",sensorheight)){sensorheight = 0.36;}
	if(!nh.getParam("/velodyne_chairs/min_vision_range",minvisionrange)){minvisionrange = 0.1;}
	if(!nh.getParam("/velodyne_chairs/max_vision_range",maxvisionrange)){maxvisionrange = 50.0;}
	if(!nh.getParam("/velodyne_chairs/horizontal_fov",visionangle)){visionangle = 120;}
	if(!nh.getParam("/velodyne_chairs/resolution",resolution)){resolution = 0.1;}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb2);
	ros::Subscriber sub_amcl = nh.subscribe("amcl_pose", 100, poseAMCLCallback);	
	laserCloudout.points.resize(1000);
	// Create a ROS publisher for the outputs	
	ChairCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/ChairCloud", 1);
	Markers = nh.advertise<visualization_msgs::MarkerArray>( "/ChairsMarkers", 1 );
	
	// Spin
	ros::spin ();	
}

const std::string now_str()
{
    // Get current time from the clock, using microseconds resolution
    const boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    // Get the time offset in current day
    const boost::posix_time::time_duration td = now.time_of_day();

    const long hours        = td.hours();
    const long minutes      = td.minutes();
    const long seconds      = td.seconds();
    const long milliseconds = td.total_milliseconds() -
                              ((hours * 3600 + minutes * 60 + seconds) * 1000);
	const long microseconds = td.total_microseconds() -
                              (((hours * 3600 + minutes * 60 + seconds) * 1000000))-(milliseconds*1000);

    char buf[40];
    //sprintf(buf, "%02ld:%02ld:%02ld.%03ld.%03ld",hours, minutes, seconds, milliseconds, microseconds);
	sprintf(buf, "%02ld:%03ld:%03ld",seconds, milliseconds, microseconds);

    return buf;
}
