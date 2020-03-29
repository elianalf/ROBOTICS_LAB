//client
//client get index and length
#include "ros/ros.h"
#include "ros_fibonacci/service.h"
#include <iostream>
#include <sstream>
#include <vector>
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv, "node2");
	
	ros::NodeHandle n;
	ros::ServiceClient client =  n.serviceClient<ros_fibonacci::service>("fibonacci_service");
	ros_fibonacci::service srv;
	srv.request.i = atoi(argv[1]);
         srv.request.l = atoi(argv[2]);
  if (client.call(srv))
  { 
      for (int j= 0; j<srv.request.l; j++){
      
	ROS_INFO("Sequence value in position %d: %d",srv.request.i+j,srv.response.seq[j]);
      }
      
    }
  else
   {
  ROS_ERROR("Failed to call service ");
   return 1;
   }
   
    return 0;
}
