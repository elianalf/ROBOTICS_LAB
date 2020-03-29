#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <actionlib/server/simple_action_server.h>
#include <iostream>
#include <sstream>
#include "ros_controller/controlAction.h"

using namespace std;
class control_class
{
	private:
	ros::NodeHandle n;
	actionlib::SimpleActionServer<ros_controller::controlAction> as; //define a action server
	ros_controller::controlFeedback feedback;
	ros_controller::controlResult result;
	
	string action_name;
	int goal;
   float step;
   float progress_p;
   
	public:
	
	control_class(string name) :
	as(n, name, boost::bind(&control_class::executeCB, this, _1), false),
	action_name(name)
	{ //In the action constructor as, an action server is created
	as.registerPreemptCallback(boost::bind(&control_class::preemptCB, this));
	as.start();
	}

	void preemptCB(){
		ROS_WARN("%s got preempted!", action_name.c_str());
		result.reached_p = progress_p; 
		as.setPreempted(); 
  	}
  
	void executeCB(const ros_controller::controlGoalConstPtr &goal) {
	
	   if(!as.isActive() || as.isPreemptRequested())  return;
	   
	   
	   progress_p=goal->ip;
	   float w=(goal->fp - goal->ip)/(goal->T * 360); //giri al secondo
	   if(w>(goal->wmax)){w=goal->wmax;}
	   step=w*0.1*360; //step value in degree (velocity*control_time*360)
	    ROS_INFO("%s is processing the goal", action_name.c_str());
	   
	   ros::Rate r(10);
	  while(progress_p<(goal->fp)){ 
	     
	    if(!as.isActive() || as.isPreemptRequested()){
	          feedback.current_p=progress_p;
					return;
		  }	
		  
      progress_p=step+progress_p;
          
      if (!ros::ok()) {result.reached_p = progress_p;
					as.setAborted(result,"I failed !");
					ROS_INFO("%s failed",action_name.c_str());
					break;}
   
		feedback.current_p = progress_p;
		ROS_INFO("The current position is %f ",progress_p);
		as.publishFeedback(feedback);// publish the feedback
	   r.sleep();
	  
	  }
	   ROS_INFO("%s Succeeded at getting to goal %f", action_name.c_str(), goal->fp);
					result.reached_p =progress_p;
					as.setSucceeded(result);
	}
	
};

int main(int argc,  char** argv){

   ros::init(argc,argv, "control_server_name");
   control_class control_node_obj(ros::this_node::getName());
   ros::spin();
   return 0;



}
