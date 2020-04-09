#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#define PI 3.14159265

ros::Publisher PeopleCloud_pub,TrunkCloud_pub,LegsCloud_pub,Markers;	
pcl::PointCloud<pcl::PointXYZ> PeopleCloud1, PeopleCloud2, PeopleCloud3, PeopleCloud4;
float sensorheight, minvisionrange, maxvisionrange, resolution, hip, hangle, legs_begin, legs_end, trunk_begin, trunk_end;
int visionangle;
int Ncloud=0;
int point1pos, point2pos,point3pos, point4pos;

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

	

    // Store the last 4 People PointClouds
	PeopleCloud4=PeopleCloud3;
	PeopleCloud3=PeopleCloud2;
	PeopleCloud2=PeopleCloud1;
	PeopleCloud1=PeopleCloud;
	int newpos=0;

    //Speed Filter
	if (Ncloud>4)
	{		
		for (int i = 0 ; i < PeopleCloud.size();i++)
		{
			int conteo=0;
			for (int j = 0 ; j < PeopleCloud2.size();j++)
			{
				float hip2 = sqrt(pow(PeopleCloud.points[i].x-PeopleCloud2.points[j].x,2)+pow(PeopleCloud.points[i].y-PeopleCloud2.points[j].y,2));				
				if (hip2<0.3 && hip2>0.1){conteo++;}								
			}
			for (int j = 0 ; j < PeopleCloud3.size();j++)
			{
				float hip3 = sqrt(pow(PeopleCloud.points[i].x-PeopleCloud3.points[j].x,2)+pow(PeopleCloud.points[i].y-PeopleCloud3.points[j].y,2));
				if (hip3<0.3 && hip3>0.1){conteo++;}				
			}
			for (int j = 0 ; j < PeopleCloud4.size();j++)
			{
				float hip4 = sqrt(pow(PeopleCloud.points[i].x-PeopleCloud4.points[j].x,2)+pow(PeopleCloud.points[i].y-PeopleCloud4.points[j].y,2));
				if (hip4<0.3 && hip4>0.1){conteo++;}				
			}
						
			if (conteo>0)
			{
				PeopleCloud.points[newpos].x=PeopleCloud.points[i].x;
				PeopleCloud.points[newpos].y=PeopleCloud.points[i].y;
				newpos++;
			}
		}
	}

	PeopleCloud.points.resize(newpos);

	//Publish LegsCloud
	sensor_msgs::PointCloud2 LegsCloud_output;
	pcl::toROSMsg(LegsCloud, LegsCloud_output);
    LegsCloud_output.header.frame_id = "base_link";
	LegsCloud_pub.publish (LegsCloud_output);

	//Publish TrunkCloud
	sensor_msgs::PointCloud2 TrunkCloud_output;
	pcl::toROSMsg(TrunkCloud, TrunkCloud_output);
    TrunkCloud_output.header.frame_id = "base_link";
	TrunkCloud_pub.publish (TrunkCloud_output);

	//Publish PeopleCloud
	sensor_msgs::PointCloud2 PeopleCloud_output;
	pcl::toROSMsg(PeopleCloud, PeopleCloud_output);
    PeopleCloud_output.header.frame_id = "base_link";
	PeopleCloud_pub.publish (PeopleCloud_output);	


	visualization_msgs::Marker marker;
	for (int j = 0 ; j < PeopleCloud.points.size();j++)
	{		
		marker.header.frame_id = "velodyne";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id =j;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = PeopleCloud.points[j].x;
		marker.pose.position.y = PeopleCloud.points[j].y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 0.001;
		marker.color.a = 0.5; // Don't forget to set the alpha!
		marker.lifetime = ros::Duration(0.2);;
		if (j==0) { marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;}
		else if (j==1) { marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; }
		else if (j==2) { marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; }
		else if (j==3) { marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; }
		else if (j==4) { marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 1.0; }
		else if (j==5) { marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0; }
		else { marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0; }
		//only if using a MESH_RESOURCE marker type:
		marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		Markers.publish(marker);		
	}
}


int
main (int argc, char **argv)
{
	// Initialize ROS
	ros::init (argc, argv, "velodyne_people");
	ros::NodeHandle nh;

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
	
	// Create a ROS publisher for the outputs	
	PeopleCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/PeopleCloud", 1);
	TrunkCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/TrunkCloud", 1);
	LegsCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/LegsCloud", 1);
	Markers = nh.advertise<visualization_msgs::Marker>( "marker", 1 );
	
	// Spin
	ros::spin ();	
}
