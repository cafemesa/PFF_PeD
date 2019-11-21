#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#define PI 3.14159265

ros::Publisher PeopleCloud_pub, LegsCloud_pub, TrunkCloud_pub;
pcl::PointCloud<pcl::PointXYZ> PeopleCloud1, PeopleCloud2;
float sensorheight, resolution, legs_begin, legs_end, trunk_begin, trunk_end, hip, hangle;
int camera_fov, point1pos, point2pos, point3pos, point4pos;
int Ncloud=0, numSamples=0;

//Struct to access for each point cordenates
struct Point3D {

  float x;
  float y;
  float z;
  Point3D(float x_, float y_, float z_)
    : x(x_), y(y_), z(z_) {

  }
};

/**
 * This function recieve the Astra Camera PointCloud. 
 * Create the Trunk PointCloud and the Legs PointCloud.
 * Matching between Trunk PointCloud and Legs PointCloud to create People PointCloud
 * Run a speed filter to get the mooving people.
 * publish th final People PointCLoud
 *
 * @param input Astra Camera PointCloud directly from the sensor
 */

void 
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& input)
{ 	
	int numPts = input->height * input->width;
	char* raw3DPtsData = (char*)(input->data.data());
    std::vector<Point3D> pc(numPts, Point3D(0.,0.,0.));

	Ncloud++;
	pcl::PointXYZI point;		
	pcl::PointCloud<pcl::PointXYZ> laserCloudIn,LegsCloud, TrunkCloud, PeopleCloud;
	pcl::PointXYZ pointlegs1, pointlegs2, pointtrunk1, pointtrunk2;
	
	numSamples=camera_fov/resolution;
	LegsCloud.points.resize(numSamples);
	TrunkCloud.points.resize(numSamples);
	PeopleCloud.points.resize(numSamples);
	
	float addpos=0;
	double LegsRanges[numSamples]={0}, TrunkRanges[numSamples]={0};	
	int position = 0, postrunk10=0,  postrunk30=0, poslegs10=0, poslegs30=0;

	
	// Generate Trunk and Legs PointCloud
	for (int i = 0; i <  numPts; i++) {

		// Access the cordenates 
		float* base = (float*)(raw3DPtsData + i * input->point_step);
    	Point3D point3d(0.,0.,0.);
		point.x=base[2];
		point.y=-base[0];
		point.z=-base[1];
		
		// Dicard points outside the area of interest
		if (-sensorheight+legs_begin>point.z || point.z > -sensorheight + trunk_end || (point.z>-sensorheight+legs_end && point.z < -sensorheight+trunk_begin) || isnan(point.x)==1 || isnan(point.y)==1){ continue;}
		
		hip = sqrt((point.x)*(point.x)+((point.y)*(point.y)));
		hangle = (asin ((point.x)/hip))*180/PI;
		
		// position in the arrays for get the nearest points
		if (point.x>0 && point.y>0) {			
			position=(hangle-60.0)/resolution;
		}
		else {
			position=(120.0-hangle)/resolution;
		}		
		
		// Generate  Legs Point Cloud
		if (-sensorheight+legs_begin<point.z && point.z < -sensorheight + legs_end) 
		{
			// Extraction of nearest points
			if (LegsRanges[position]==0 || hip < LegsRanges[position])
			{
				LegsCloud.points[position].x=point.x;
				LegsCloud.points[position].y=point.y;
				LegsCloud.points[position].z=0;	 				// Projection onto 2D
				LegsRanges[position]=hip;
			}
		}
		// Generate  Trunk Point Cloud
		if (-sensorheight+trunk_begin<point.z && point.z < -sensorheight + trunk_end) 
		{
			// Extraction of nearest points
			if (TrunkRanges[position]==0 || hip < TrunkRanges[position])
			{
				TrunkCloud.points[position].x=point.x;
				TrunkCloud.points[position].y=point.y;
				TrunkCloud.points[position].z=0;				// Projection onto 2D
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
				if (0.05<hiplegs2 && hiplegs2<0.35)
				{
					//Save de average between the first and the last point of the detection
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

	// Trunk Filter
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
	
	// Store the last 2 People PointClouds
	PeopleCloud2=PeopleCloud1;
	PeopleCloud1=PeopleCloud;
	int newpos=0;

	//Speed Filter
	if (Ncloud>1)
	{		int conteo=0;
		for (int i = 0 ; i < PeopleCloud.size();i++)
		{			
			for (int j = 0 ; j < PeopleCloud2.size();j++)
			{
				float hip2 = sqrt(pow(PeopleCloud.points[i].x-PeopleCloud2.points[j].x,2)+pow(PeopleCloud.points[i].y-PeopleCloud2.points[j].y,2));			
				if (hip2<0.05 && hip2>0.005)
				{
					PeopleCloud.points[newpos].x=PeopleCloud.points[i].x;
					PeopleCloud.points[newpos].y=PeopleCloud.points[i].y;
					newpos++;
				}								
			}		
		}
	}
	PeopleCloud.points.resize(newpos);

	//Publish PeopleCloud
	sensor_msgs::PointCloud2 PeopleCloud_output;
	pcl::toROSMsg(PeopleCloud, PeopleCloud_output);
    PeopleCloud_output.header.frame_id = "camera_depth_frame";
	PeopleCloud_pub.publish (PeopleCloud_output);	

	//Publish TrunkCloud
	sensor_msgs::PointCloud2 TrunkCloud_output;
	pcl::toROSMsg(TrunkCloud, TrunkCloud_output);
    TrunkCloud_output.header.frame_id = "camera_depth_frame";
	TrunkCloud_pub.publish (TrunkCloud_output);	

	//Publish LegsCloud
	sensor_msgs::PointCloud2 LegsCloud_output;
	pcl::toROSMsg(LegsCloud, LegsCloud_output);
    LegsCloud_output.header.frame_id = "camera_depth_frame";
	LegsCloud_pub.publish (LegsCloud_output);	
}


int
main (int argc, char **argv)
{
	// Initialize ROS
	ros::init (argc, argv, "astra_people");
	ros::NodeHandle nh;

	// Get launch parameters
	if(!nh.getParam("/astra_people/sensor_height",sensorheight)){sensorheight = 0.6;}
	if(!nh.getParam("/astra_people/camera_fov",camera_fov)){camera_fov = 60;}
	if(!nh.getParam("/astra_people/resolution",resolution)){resolution = 0.4;}
	if(!nh.getParam("/astra_people/legs_begin",legs_begin)){legs_begin=0.50;}
	if(!nh.getParam("/astra_people/legs_end",legs_end)){legs_end=0.60;}
	if(!nh.getParam("/astra_people/trunk_begin",trunk_begin)){trunk_begin=1.00;}
	if(!nh.getParam("/astra_people/trunk_end",trunk_end)){trunk_end=1.10;}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb2);
	
	// Create a ROS publisher for the outputs	
	PeopleCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/PeopleCloud", 1);
	TrunkCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/TrunkCloud", 1);
	LegsCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/LegsCloud", 1);
	
	// Spin
	ros::spin ();	
}