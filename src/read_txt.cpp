
#include <ros/ros.h>
#include <iostream>
#include <fstream>

float DoorBeginx[100]={0}, DoorBeginy[100]={0}, DoorEndx[100]={0}, DoorEndy[100]={0};

int main(int argc, char * argv[])
{
    std::fstream myfile("src/pff_ped/src/info.txt", std::ios_base::in);
	char* pend; 
    float a;
	int col=0;
	int position=0;
    while (myfile >> a)
    {
		if (col==0)
		{
			DoorBeginx[position]=a;
			//printf("col1 : %f \n", a);
			col++;
		}
		else if (col==1)
		{
			DoorBeginy[position]=a;
			//printf("col2 : %f \n", a);
			col++;
		}
		else if (col==2)
		{
			DoorEndx[position]=a;
			//printf("col3 : %f \n", a);
			col++;
		}
		else if (col==3)
		{
			DoorEndy[position]=a;
			//printf("col4 : %f \n", a);
			col=0;
			position++;
		}
    }

	for (int i=0 ; i<position ; i++)
	{
		ROS_INFO("col 1 : %f , col 2 : %f , col 3 : %f , col 4 : %f ", DoorBeginx[i], DoorBeginy[i], DoorEndx[i], DoorEndy[i]);
	}

    return 0;
}