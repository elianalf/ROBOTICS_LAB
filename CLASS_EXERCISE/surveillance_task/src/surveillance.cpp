#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <actionlib/server/simple_action_server.h>
#include "surveillance_task/navAction.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>


using namespace std;

class SURV {
    public:
        SURV(std::string name);
        void laser_cb( sensor_msgs::LaserScanConstPtr );
        void odom_cb( nav_msgs::OdometryConstPtr );
        void run();
        void executeCB( const surveillance_task::navGoalConstPtr &goal );
        void preemptCb( );
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _laser_sub;
        actionlib::SimpleActionServer<surveillance_task::navAction> _as; 
        string action_name;
        ros::Publisher _cmd_vel_pub;
        double _yaw;
        float xp;
        float yp;
        
};

SURV::SURV(std::string name) :
    _as(_nh, name, boost::bind(&SURV::executeCB, this, _1), false),
    action_name(name) {
        _as.registerPreemptCallback( boost::bind(&SURV::preemptCb, this) );
        _laser_sub = _nh.subscribe("/laser/scan",0,&SURV::laser_cb,this);
        _odom_sub = _nh.subscribe("/odom", 0, &SURV::odom_cb, this);
        _cmd_vel_pub = _nh.advertise< geometry_msgs::Twist>("/cmd_vel", 0);
        _as.start();

}



void SURV::preemptCb() {
	surveillance_task::navResult result;
	_as.setPreempted(result, "Preempt");
    cout<<"got preempted"<<endl;
    geometry_msgs::Twist cmd;
	cmd.angular.z = 0.0;
	cmd.linear.x = 0.0;
	_cmd_vel_pub.publish(cmd);
}


void SURV::executeCB( const surveillance_task::navGoalConstPtr &goal ) {

    //TODO: Get current position    
    //  

    //TARGET: goal->x_dest, goal->y_dest
    //MES     _x, _y
    ros::Rate r(10);
    bool moving = false;
    bool done = false;    
    float kp_o = 0.5;
    float kp_p = 0.5;
    double z_vel, x_vel;
    double max_lin_vel = 1.5;
    double max_ang_vel = M_PI;
    geometry_msgs::Twist cmd;
    surveillance_task::navResult result;
    
    while (!done && !_as.isPreemptRequested()) {
         // Retrieve error
        double des_yaw = atan2( goal->y_dest - yp, goal->x_dest - xp);
        double yaw_e = des_yaw - _yaw;
        /*if(fabs(yaw_e) > M_PI)
           yaw_e = yaw_e - 2*M_PI* ((yaw_e>0)?1:-1);*/
        double pos_e = sqrt(pow(goal->y_dest-yp,2)+pow(goal->x_dest-xp,2));
        if(fabs((des_yaw+3.14) - (_yaw+3.14)) > 3.14){ z_vel = kp_o * yaw_e; }
        else { z_vel = -kp_o * yaw_e; }

        // Drive
        if(fabs(yaw_e) < 0.1)
            moving = true;

        //z_vel = -kp_o * yaw_e;
        
        if(moving)
            x_vel = kp_p * pos_e;
        else
            x_vel = 0;

        // Check max vel
        if (fabs(x_vel) > max_lin_vel)
            x_vel = max_lin_vel * ((x_vel>0)?1:-1);
        if (fabs(z_vel) > max_ang_vel)
            z_vel = max_ang_vel * ((z_vel>0)?1:-1);
        
        // Check if robot is arrived
        if(fabs(pos_e) < 0.05){
            done = true;
            z_vel=0;
            x_vel=0;
            result.x_curr=xp;
            result.y_curr=yp;
        }

        // Publish cmd
        cmd.angular.z = z_vel;
        cmd.linear.x = x_vel;
        _cmd_vel_pub.publish(cmd);

        cout << "yaw_e: " << yaw_e << ", pos_e: " << pos_e << endl;
				r.sleep();
    }
    if(done){
        _as.setSucceeded(result);
        cout << "SUCCEEDED" << endl;}

    
}


void SURV::laser_cb( sensor_msgs::LaserScanConstPtr ) {
}
 
void SURV::odom_cb( nav_msgs::OdometryConstPtr odom ) {
    
    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, _yaw);

    xp = odom->pose.pose.position.x;
    yp = odom->pose.pose.position.y;
      
    
}
 
//main loop! 

 
 
void SURV::run() {
   
    ros::spin();
}



int main( int argc, char** argv) {

    ros::init(argc, argv, "surv_task" );
    SURV s("auto_nav_server"); //SURV(std::string name);
    s.run();
    
    return 0;

}   


