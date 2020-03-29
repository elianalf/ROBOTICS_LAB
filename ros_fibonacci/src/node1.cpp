//server
#include "ros/ros.h"
#include "ros_fibonacci/service.h" 
#include <iostream>
#include <sstream>
#include <vector>
using namespace std;

bool fibonacci_callback(ros_fibonacci::service::Request &req, ros_fibonacci::service::Response &res)
{
  
  int first=0;
  int second=1;
  int buff=0;
  int k=2;
  if((req.i+req.l)==1){
     res.seq.push_back(first);
  }
  else if((req.i+req.l)==2){
  res.seq.push_back(second);
  }
  else{
    for(k=2; k<(req.l+req.i); k++){
         buff=first+second;
        if(k>=(req.i)){
     
	    res.seq.push_back(buff);
          }
  	first=second;
	second=buff;
    }
  }
  ROS_INFO("request: index=%d, length=%d", req.i, req.l);
 for(int j=0; j<(req.l);j++){ 
  ROS_INFO("response sequence: [%d]", res.seq[j]);
 }
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"node1");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("fibonacci_service", fibonacci_callback);
  ROS_INFO("Ready to compute the sequence. Add the index and length of the Fibonacci sequence you want (length>0)");
  ros::spin();
  return 0;
}


