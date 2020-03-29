/* Client get in input (float)initial_position[degrees] (float)final_position[degrees] (float)time[seconds] (float)maximum_rotational_velocity[rps]*/
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros_controller/controlAction.h>
using namespace ros_controller;
void doneCb(const actionlib::SimpleClientGoalState &state,const ros_controller::controlResultConstPtr &result){
   ROS_INFO("The server has done");
   ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %f", result->reached_p);
}

void activeCb(){
   ROS_INFO("the motor is rotating");
}
void feedbackCb(const ros_controller::controlFeedbackConstPtr &feedback){ 

   ROS_INFO("the current position is %f",feedback->current_p);
}


int main(int argc, char** argv)
{
   ros::init(argc,argv,"control_client_name");
   actionlib::SimpleActionClient<ros_controller::controlAction> ac("control_server_name",true);
   
  
   ROS_INFO("Waiting for controller to start.");
    ac.waitForServer();
    
    ROS_INFO("CONTROLLER STARTED");
    ros_controller::controlGoal goal;
    goal.ip=atof(argv[1]);
    goal.fp=atof(argv[2]);
    goal.T=atof(argv[3]);
    goal.wmax=atof(argv[4]);
    
    
    float timeout=atof(argv[3])+0.09;
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    bool finish_intime= ac.waitForResult(ros::Duration(timeout));
    
    ac.cancelGoal();
    
    if(finish_intime){
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("POSITION REACHED %s",state.toString().c_str());
    }
    else{
      ROS_INFO("Motor unable to reach the desired postion in the desired time");
    }
    return 0;
}
