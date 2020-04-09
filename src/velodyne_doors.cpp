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

ros::Publisher ThirdCloud_pub,Markers,TestCloud_pub;
float resolution, hip, hangle;
int visionangle;
pcl::PointCloud<pcl::PointXYZ> FinalDoorCloud,laserCloudout;
int posout=0;
float pose_x=0, pose_y=0, initial_angle=0;
int maparray [4000][4000];	

const std::string now_str();
std::ofstream ptime;
std::string abegtime;
std::string aendtime;


void 
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& input)
{ 

	abegtime=now_str();

	pcl::PointCloud<pcl::PointXYZ> laserCloudIn,FirstCloud, SecondCloud, ThirdCloud;
	pcl::fromROSMsg(*input, laserCloudIn);

    unsigned int num_readings = 360/resolution;
	FirstCloud.points.resize(num_readings);
	SecondCloud.points.resize(num_readings);
	ThirdCloud.points.resize(num_readings);
	
	double FirstRanges[num_readings]={0};
	int StartCluster[num_readings]={0};
	int EndCluster[num_readings]={0};

	int position = 0;
	
	for (int i = 0; i < laserCloudIn.size(); i++) {
		
		hip = sqrt((laserCloudIn[i].x)*(laserCloudIn[i].x)+((laserCloudIn[i].y)*(laserCloudIn[i].y)));
		hangle = (asin ((laserCloudIn[i].x)/hip))*180/PI;

		// Dicard points outside the area of interest (Horizontal Filter)
		if (visionangle==180 && laserCloudIn[i].x<=0){continue;}
		else if ((visionangle<180 && laserCloudIn[i].x>0 && hangle<90-(visionangle/2)) || (visionangle<180 && laserCloudIn[i].x<=0)){continue;}
		else if (visionangle>180 && visionangle<360 && laserCloudIn[i].x<0 && abs(hangle)>(visionangle-180)/2){continue;}
		
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

		// Generate  first layer
		if (-0.20<laserCloudIn[i].z && laserCloudIn[i].z < 0.40) 
		{
            // Extraction of nearest points
			if (FirstRanges[position]==0 || hip < FirstRanges[position])
			{
				FirstCloud.points[position].x=laserCloudIn[i].x;
				FirstCloud.points[position].y=laserCloudIn[i].y;
				FirstCloud.points[position].z=0;	                    // Projection onto 2D
				FirstRanges[position]=hip;
			}
		}		
	}

	// Define clusters
	int cluster_position=0;
	StartCluster[cluster_position]=0;
	for (int i = 0; i < FirstCloud.size()-1; i++) {
		if (FirstCloud.points[i].x<0.1 && FirstCloud.points[i].y<0.1){continue;}	
		float hip_cluster = sqrt(pow(FirstCloud.points[i+1].x-FirstCloud.points[i].x,2)+pow(FirstCloud.points[i+1].y-FirstCloud.points[i].y,2));
		if (hip_cluster>0.5)
		{
			EndCluster[cluster_position]=i;
			cluster_position++;
			StartCluster[cluster_position]=i+1;
		}
	}
	EndCluster[cluster_position]=FirstCloud.size()-1;	

	// Define doors
	float size1min=0.8;
	float size1max=1.0;
	int doorpos=0;
	for (int i = 0; i < cluster_position-1; i++) {
		for (int j = i+1; j < cluster_position; j++) 
		{			
			float init_first_x = FirstCloud.points[StartCluster[i]].x;
			float init_first_y = FirstCloud.points[StartCluster[i]].y;
			float init_second_x = FirstCloud.points[StartCluster[j]].x;
			float init_second_y = FirstCloud.points[StartCluster[j]].y;
			float end_first_x = FirstCloud.points[EndCluster[i]].x;
			float end_first_y = FirstCloud.points[EndCluster[i]].y;
			float end_second_x = FirstCloud.points[EndCluster[j]].x;
			float end_second_y = FirstCloud.points[EndCluster[j]].y;

			float hip_clusters1 = sqrt(pow(init_first_x-end_second_x,2)+pow(init_first_y-end_second_y,2));
			float hip_clusters2 = sqrt(pow(init_second_x-end_first_x,2)+pow(init_second_y-end_first_y,2));

			float hip_clusters3 = sqrt(pow(init_first_x-init_second_x,2)+pow(init_first_y-init_second_y,2));
			float hip_clusters4 = sqrt(pow(end_second_y-end_first_x,2)+pow(end_second_y-end_first_y,2));

			/*if (hip_clusters1>0.9 && hip_clusters1<0.95)
			{
				SecondCloud.points[doorpos].x=(init_first_x+end_second_x)/2;
				SecondCloud.points[doorpos].y=(init_first_y+end_second_y)/2;
				SecondCloud.points[doorpos].z=0;
				//ROS_INFO("Primer initx=%f, inity=%f, endx=%f, endy=%f, midx=%f, midy=%f", init_first_x,init_first_y,end_second_x,end_second_y,SecondCloud.points[doorpos].x,SecondCloud.points[doorpos].y);
				doorpos++;
			}*/
			if ((hip_clusters2>0.92 && hip_clusters2<0.98) )
			{
				SecondCloud.points[doorpos].x=(init_second_x+end_first_x)/2;
				SecondCloud.points[doorpos].y=(init_second_y+end_first_y)/2;
				SecondCloud.points[doorpos].z=0;
				//ROS_INFO("Segundo initx=%f, inity=%f, endx=%f, endy=%f, midx=%f, midy=%f", init_second_x,init_second_y,end_first_x,end_first_y,SecondCloud.points[doorpos].x,SecondCloud.points[doorpos].y);
				doorpos++;
			}	
			/*if (hip_clusters3>0.9 && hip_clusters3<0.95)
			{
				SecondCloud.points[doorpos].x=(init_first_x+init_second_x)/2;
				SecondCloud.points[doorpos].y=(init_first_y+init_second_y)/2;
				SecondCloud.points[doorpos].z=0;
				//ROS_INFO("Tercero initx=%f, inity=%f, endx=%f, endy=%f, midx=%f, midy=%f", init_first_x,init_first_y,init_second_x,init_second_y,SecondCloud.points[doorpos].x,SecondCloud.points[doorpos].y);
				doorpos++;
			}	
			if (hip_clusters4>0.9 && hip_clusters4<0.95)
			{
				SecondCloud.points[doorpos].x=(end_second_x+end_first_x)/2;
				SecondCloud.points[doorpos].y=(end_second_y+end_first_y)/2;
				SecondCloud.points[doorpos].z=0;
				//ROS_INFO("Cuarto initx=%f, inity=%f, endx=%f, endy=%f, midx=%f, midy=%f", end_second_x,end_second_y,end_first_x,end_first_y,SecondCloud.points[doorpos].x,SecondCloud.points[doorpos].y);
				doorpos++;
			}	*/	
			
		}		
	}

	// Filter by the distance between the robot and the posible door to reduce errors
	SecondCloud.resize(doorpos);
	int doorposfinal=0;
	for (int i = 0; i < SecondCloud.size(); i++) {
		hip = sqrt(pow(SecondCloud.points[i].x,2)+pow(SecondCloud.points[i].y,2));
		if (hip<1.3 && hip>0.5)
		{
			ThirdCloud.points[doorposfinal].x=SecondCloud.points[i].x;
			ThirdCloud.points[doorposfinal].y=SecondCloud.points[i].y;
			ThirdCloud.points[doorposfinal].z=0;
			doorposfinal++;
		}
	}
	ThirdCloud.resize(doorposfinal);


	//std::cout << "Begin=" <<  abegtime << "Finish=" <<  now_str() << std::endl;

	
	for (int i = 0; i < ThirdCloud.size(); i++) {
		
		//ROS_INFO("x=%f, y=%f", ThirdCloud[i].x,ThirdCloud[i].y);
		float hip = sqrt((ThirdCloud[i].x)*(ThirdCloud[i].x)+((ThirdCloud[i].y)*(ThirdCloud[i].y)));
		float hangle = fabs(asin ((ThirdCloud[i].x)/hip));
		
		if (ThirdCloud[i].x>0 && ThirdCloud[i].y>0){hangle=(PI/2)-hangle;}
		if (ThirdCloud[i].x>0 && ThirdCloud[i].y<0){hangle=(3*PI/2)+hangle;}
		if (ThirdCloud[i].x<0 && ThirdCloud[i].y<0){hangle=(3*PI/2)-hangle;}
		if (ThirdCloud[i].x<0 && ThirdCloud[i].y>0){hangle=(PI/2)+hangle;}

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
			ThirdCloud[i].y= hip*sin(finalangle);
			ThirdCloud[i].x= sqrt(-((ThirdCloud[i].y)*(ThirdCloud[i].y))+((hip)*(hip)));
		}
		else if (finalangle>PI/2 && finalangle<PI)
		{
			ThirdCloud[i].y= hip*sin(PI-finalangle);
			ThirdCloud[i].x= -sqrt(-((ThirdCloud[i].y)*(ThirdCloud[i].y))+((hip)*(hip)));
		}
		else if (finalangle>PI && finalangle<3*PI/2)
		{
			ThirdCloud[i].y= -hip*sin(finalangle-PI);
			ThirdCloud[i].x= -sqrt(-((ThirdCloud[i].y)*(ThirdCloud[i].y))+((hip)*(hip)));
		}
		else if (finalangle>3*PI/2 && finalangle<2*PI)
		{
			ThirdCloud[i].y= -hip*sin(2*PI-finalangle);
			ThirdCloud[i].x= sqrt(-((ThirdCloud[i].y)*(ThirdCloud[i].y))+((hip)*(hip)));
		
		}
		ThirdCloud[i].x=ThirdCloud[i].x+pose_x;
		ThirdCloud[i].y=ThirdCloud[i].y+pose_y;	

		int posxpx=((-100-ThirdCloud[i].x)/(-0.05));
		int posypx=((-100-ThirdCloud[i].y)/(-0.05));
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

		if (spacecount<15)
		{
			if (posout==0)
			{
				laserCloudout[posout].x=ThirdCloud[i].x;
				laserCloudout[posout].y=ThirdCloud[i].y;
				posout++;
			}
			else
			{
				int prueba=0;
				for (int j = 0; j < laserCloudout.size(); j++) {
					float hiprem = sqrt(pow(laserCloudout[j].x-ThirdCloud[i].x,2)+pow(laserCloudout[j].y-ThirdCloud[i].y,2));
					if (hiprem<0.9){break;}
					else {prueba++;}
				}
				if (prueba==laserCloudout.size())
				{
					laserCloudout[posout].x=ThirdCloud[i].x;
					laserCloudout[posout].y=ThirdCloud[i].y;
					posout++;
				}
			}
		}
	}
	//laserCloudout.resize(posout);

	//Publish ThirdCloud
	sensor_msgs::PointCloud2 ThirdCloud_output;
	pcl::toROSMsg(laserCloudout, ThirdCloud_output);
    ThirdCloud_output.header.frame_id = "map";
	ThirdCloud_pub.publish (ThirdCloud_output);


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
		marker.markers[j].color.r = 1.0; 
		marker.markers[j].color.g = 0.0; 
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
	ros::init (argc, argv, "velodyne_doors");
	ros::NodeHandle nh;

	// Get launch parameters
	if(!nh.getParam("/velodyne_doors/horizontal_fov",visionangle)){visionangle = 120;}
	if(!nh.getParam("/velodyne_doors/resolution",resolution)){resolution = 0.2;}
	
	laserCloudout.points.resize(1000);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb2);
	ros::Subscriber sub_amcl = nh.subscribe("amcl_pose", 100, poseAMCLCallback);	
	ros::Subscriber map_sub = nh.subscribe("/map",10,mapCallback);

	// Create a ROS publisher for the outputs	
	ThirdCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/DoorCloud", 1);
	TestCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/TestCloud", 1);
	Markers = nh.advertise<visualization_msgs::MarkerArray>( "/markerArray", 1 );
	
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