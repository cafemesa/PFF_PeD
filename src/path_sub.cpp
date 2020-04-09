#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#define PI 3.14159265

ros::Publisher Mypath1, Mypath2, Mypath3;

void PathCallback (const nav_msgs::Path::ConstPtr& msgAMCL) {
    float posesx[msgAMCL->poses.size()];
    float posesy[msgAMCL->poses.size()];
    float orienx[msgAMCL->poses.size()];
    float orieny[msgAMCL->poses.size()];
    float orienz[msgAMCL->poses.size()];
    float orienw[msgAMCL->poses.size()];
    for (int i = 0 ; i < msgAMCL->poses.size() ; i++)
    {
        posesx[i]=msgAMCL->poses[i].pose.position.x;
        posesy[i]=msgAMCL->poses[i].pose.position.y;
        orienx[i]=msgAMCL->poses[i].pose.orientation.x;
        orieny[i]=msgAMCL->poses[i].pose.orientation.y;
        orienz[i]=msgAMCL->poses[i].pose.orientation.z;
        orienw[i]=msgAMCL->poses[i].pose.orientation.w;
        ROS_INFO("posex %f", msgAMCL->poses[i].pose.position.x);
        ROS_INFO("posey %f", msgAMCL->poses[i].pose.position.y);
    }    
} 

int
main (int argc, char **argv)
{
	// Initialize ROS
	ros::init (argc, argv, "psth_subscriber");
	ros::NodeHandle nh;   

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub_amcl = nh.subscribe("/move_base/NavfnROS/plan", 1, PathCallback);
    //ros::Subscriber sub_amcl = nh.subscribe("/mypath", 1, PathCallback);

	// Spin
	ros::spin ();	
}

