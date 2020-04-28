#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "boost/thread.hpp"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
using namespace std;

class FUNCS {

	public: 
		FUNCS();
		void run();
		void odom_cb( nav_msgs::OdometryConstPtr );
		void loop();
	private:	
		ros::NodeHandle 	_nh;
		ros::Subscriber _odom_sub;
		ros::Publisher  _vel_pub;
		double _yaw;	
		float p[2];
		bool _first_mis_ready;
};




FUNCS::FUNCS() {
	

	_odom_sub = _nh.subscribe("/odom", 0, &FUNCS::odom_cb, this);
	_vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 0);

	_yaw = 0.0;
	_first_mis_ready = false;
}


void FUNCS::odom_cb( nav_msgs::OdometryConstPtr odom) {
	p[0] = odom->pose.pose.position.x;
	p[1] = odom->pose.pose.position.y;
	tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
	double roll, pitch;
	tf::Matrix3x3(q).getRPY(roll, pitch, _yaw);
	_first_mis_ready = true;
} 


void FUNCS::loop() {

	ros::Rate r(10);
	
	float target[2];
	target[0] = -1.0;
	target[1] = 0.0;

	geometry_msgs::Twist cmd;
	float kp_o = 0.5;
	while( ros::ok() ) {

		float versore[2];
		versore[0] = target[0] - p[0];
		versore[1] = target[1] - p[1];
		float d_a = atan2( 	versore[1], versore[0] );
		float a_e = _yaw - d_a;
		

		cmd.angular.z = a_e*kp_o;
		
		_vel_pub.publish( cmd );
		r.sleep();
	}

}

void FUNCS::run() {
	boost::thread loop_t( &FUNCS::loop, this );	
	ros::spin();
}






int main( int argc, char** argv ) {

	ros::init(argc, argv, "test_func");
	FUNCS f;
	f.run();
	return 0;
}

