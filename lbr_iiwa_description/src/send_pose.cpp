#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include "Eigen/Dense"

using namespace std;

/*The client can choose the trajectory (in terms of points, we don't care about velocity ecc right now). Actually method publish/subscrib is not the best way to send the "oneshot information", we could use service for example*/

geometry_msgs::Pose starting_pose;
bool first_eef_pose = false;

void c_pose_cb(geometry_msgs::Pose eef_pose){
   starting_pose = eef_pose;
   first_eef_pose = true;
}


int main(int argc, char **argv){
   ros::init(argc,argv, "send_pose");
   
   ros::NodeHandle nh;
   ros::Publisher pub;
   ros::Subscriber sub;
   sub = nh.subscribe("/lbr_iiwa/eef_pose", 0 , c_pose_cb); //get the cartisian pose from gazebo
   pub = nh.advertise<nav_msgs::Path> ("/nav/path",0); //The topic is defined by the plugin to send the desired postion
   cout<<"Begin "<<endl;
   
   while(!first_eef_pose){
      usleep(0.1*1e6);
      ros::spinOnce(); 
   }
   
   cout<<"first_eef_pose true"<<endl;
   
   ros::Rate rate(300);
   geometry_msgs::Pose pose;
   tf::Quaternion quat;
   
   Eigen::Vector3d curr_p;
   Eigen::Vector3d des_p;
   curr_p << starting_pose.position.x, starting_pose.position.y, starting_pose.position.z;
   des_p << starting_pose.position.x, starting_pose.position.y, starting_pose.position.z - 0.2;
   Eigen::Vector3d dir = des_p - curr_p;
   dir = dir / dir.norm(); //versor(normalized vector)
   double t = 0.0;
   
   nav_msgs::Path p;
   geometry_msgs::PoseStamped c_p;
 /*Inside nav_msgs/Path:
    geometry_msgs/PoseStamped[] poses
         std_msgs/Header header
                  uint32 seq
                  time stamp
                  string frame_id
              geometry_msgs/Pose pose
                 geometry_msgs/Point position
                   float64 x
                   float64 y
                   float64 z
                 geometry_msgs/Quaternion orientation
                   float64 x
                   float64 y
                   float64 z
                   float64 w      */
    cout<<"Start the while"<<endl;
     
   while((des_p - curr_p).norm() > 0.001){
      curr_p+= dir*(1.0/300.0);
      cout<< "Current position "<< curr_p.transpose()<<endl;
      c_p.pose.position.x = curr_p(0);
      c_p.pose.position.y = curr_p(1);
      c_p.pose.position.z = curr_p(2);
      c_p.pose.orientation = starting_pose.orientation;
      p.poses.push_back(c_p);
      rate.sleep();
   }
   
   pub.publish(p); //publish the vector of the discrete trajectory
   
cout<<"Publish"<<endl;


}
